# wall_explore.py
# Навигация "правило правой стены" + детектор двери + центр второй комнаты.
# Протокол телеметрии: WBTG + 9 float32 + lidar (n float32). Управление: UDP 2 float32 (v,w).

import math, os, socket, struct, time
from enum import Enum

# ------------------- сеть/окружение -------------------
CMD_HOST  = os.getenv("CMD_HOST", "127.0.0.1")     # для Docker Desktop: host.docker.internal
CMD_PORT  = int(os.getenv("CMD_PORT", "5555"))
TEL_HOST  = os.getenv("TEL_HOST", "0.0.0.0")
TEL_PORT  = int(os.getenv("TEL_PORT", "5600"))
PROTO     = os.getenv("PROTO", "tcp")              # "tcp" или "udp"

# ------------------- тюнинг -------------------
LIDAR_HALF_FOV = math.pi/4        # обзор ~180° (если у тебя ~90° суммарно — поставь pi/4)
MAX_RANGE_CLIP = 6.0
SMOOTH_WIN     = 5

# Безопасность
SAFE_DIST   = 0.70
CRASH_DIST  = 0.35
WALL_SET    = 0.55                # желаемая дистанция до стены при wall-follow
WALL_NEAR   = 1.0                 # "мы рядом со стеной" (включаем режим wall-follow)

# Кинематика
V_MAX   = 0.80
V_MIN   = 0.12
K_V     = 0.45
W_MAX   = 1.2
Kp_wall = 1.4                     # усиление по дистанции до стены
Ka_wall = 0.9                     # усиление по углу стены (наклон)

# Застревание и эскейпы
STUCK_TIME       = 2.2
RECOVER_TIME     = 0.8
ESC_BACK_TIME    = 0.8
ESC_TURN_ANGLE   = math.pi/2      # 90°
ESC_FWD_TIME     = 0.6
ESC_W            = 0.9 * W_MAX
CRASH_WIN        = 2.0            # окно для "серийных" CRASH
TURN_POLICY      = "right"        # "right" | "free" (по свободной стороне)

# Детекторы окружения
DOOR_FRONT_MIN = 3.2
DOOR_SIDE_MAX  = 0.9
ROOM_FRONT_MIN = 4.0
ROOM_SIDE_MIN  = 1.8

# Вторая комната / центр
PERIM_TIME_MIN  = 6.0             # минимум времени обхода периметра перед расчётом центра
CENTER_RADIUS   = 0.30

# Реже панорамы (для фильма, не обязательно):
SPIN_DISTANCE   = 2.0

# ------------------- служебные -------------------
def idx_to_angle(i, n):
    return (i - n//2) * (2*LIDAR_HALF_FOV) / max(1, (n-1))

def smooth(arr, win=3):
    if win <= 1: return arr[:]
    out=[]; m=len(arr); h=win//2
    for i in range(m):
        lo=max(0,i-h); hi=min(m,i+h+1)
        out.append(sum(arr[lo:hi])/(hi-lo))
    return out

def dist2(a,b): return math.hypot(a[0]-b[0], a[1]-b[1])
def ang_wrap(a): return (a + math.pi)%(2*math.pi) - math.pi

# ------------------- сеть -------------------
sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def _tcp_accept():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((TEL_HOST, TEL_PORT))
    s.listen(1)
    print(f"[client] waiting TCP {TEL_HOST}:{TEL_PORT} …")
    c,_ = s.accept()
    print("[client] connected to udp_diff telemetry")
    return c

sock_tel = None
if PROTO == "udp":
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_tel.bind((TEL_HOST, TEL_PORT))
else:
    sock_tel = _tcp_accept()

last_cmd_v = 0.0
def send_cmd(v,w):
    global last_cmd_v
    last_cmd_v = float(v)
    sock_cmd.sendto(struct.pack("<2f", float(v), float(w)), (CMD_HOST, CMD_PORT))

def _recv_all(s, n):
    buf=b""
    while len(buf)<n:
        chunk=s.recv(n-len(buf))
        if not chunk: return None
        buf+=chunk
    return buf

def recv_tel():
    if PROTO == "udp":
        data,_ = sock_tel.recvfrom(65535)
    else:
        sz = sock_tel.recv(4)
        if not sz: return None
        data = _recv_all(sock_tel, struct.unpack("<I", sz)[0])
    if not data or not data.startswith(b"WBTG"): return None
    hdr = 4 + 9*4
    x,y,th,vx,vy,vth,wx,wy,wz = struct.unpack("<9f", data[4:hdr])
    n = struct.unpack("<I", data[hdr:hdr+4])[0]
    rng=[]
    if n>0: rng = list(struct.unpack(f"<{n}f", data[hdr+4:hdr+4+4*n]))
    # 0.0 → "нет замера": заменим на MAX_RANGE_CLIP
    rng = [MAX_RANGE_CLIP if r<=0.0 else min(MAX_RANGE_CLIP, r) for r in rng]
    return (x,y,th),(vx,vy,vth),(wx,wy,wz),rng

# ------------------- режимы -------------------
class Mode(Enum):
    FOLLOW_WALL   = 1
    DOORLOCK      = 2
    ROOM2_PERIM   = 3
    GO_CENTER     = 4
    CORNER_ESCAPE = 5
    STOP          = 6

# ------------------- логика -------------------
def looks_like_door(sm):
    n=len(sm); front=sm[n//2]; left=sm[int(n*0.25)]; right=sm[int(n*0.75)]
    return front>DOOR_FRONT_MIN and left<DOOR_SIDE_MAX and right<DOOR_SIDE_MAX

def looks_like_room(sm):
    n=len(sm); front=sm[n//2]; left=sm[int(n*0.25)]; right=sm[int(n*0.75)]
    return front>ROOM_FRONT_MIN and left>ROOM_SIDE_MIN and right>ROOM_SIDE_MIN

def side_metrics(sm, side='right'):
    """Две выборки на стороне дают оценку дистанции до стены и её наклона."""
    n=len(sm)
    if side=='right':
        i1=int(0.12*n); i2=int(0.28*n)  # ~[-65°, -35°]
    else:
        i1=int(0.88*n); i2=int(0.72*n)  # ~[+65°, +35°]
    d1=sm[i1]; d2=sm[i2]
    a1=idx_to_angle(i1,n); a2=idx_to_angle(i2,n)
    p1=(d1*math.cos(a1), d1*math.sin(a1))
    p2=(d2*math.cos(a2), d2*math.sin(a2))
    wall_ang = math.atan2(p2[1]-p1[1], p2[0]-p1[0])  # угол линии стены в СК робота
    dist = (d1+d2)/2.0
    return dist, wall_ang

def wall_follow(sm, prefer_right=True):
    n=len(sm); front=sm[n//2]
    # Выбор стороны: если правая явно ближе — держим правую; иначе — левую
    rnear=sum(sm[:max(5,n//8)])/max(5,n//8)
    lnear=sum(sm[-max(5,n//8):])/max(5,n//8)
    use_right = prefer_right if abs(rnear-lnear)<0.15 else (rnear<lnear)
    dist, wang = side_metrics(sm, 'right' if use_right else 'left')

    # Ошибка по дистанции
    e = WALL_SET - min(dist, MAX_RANGE_CLIP)
    # Угол: для правой стены хотим wang≈0 (параллельно), знак инвертируем под сторону
    w = Kp_wall*e + (Ka_wall*wang * (1 if use_right else -1))
    w = max(-W_MAX, min(W_MAX, w))

    v = V_MIN + K_V*min(front, 2.0)
    v *= max(0.25, 1.0 - abs(w)/W_MAX)

    # фронт очень близко — "обнимем" дальнюю сторону
    if front < SAFE_DIST:
        bias = math.copysign(W_MAX*0.9, (rnear - lnear))
        return 0.0, bias, use_right
    return v, w, use_right

# ------------------- main -------------------
mode = Mode.FOLLOW_WALL
prefer_right = True

last_pos = None
last_move = time.time()
started_at = time.time()

# crash streak
crash_streak = 0
crash_t0 = 0.0

# escape state
esc_phase = None
esc_until = 0.0
esc_dir = +1
esc_target = 0.0

# doorlock
door_until = 0.0
passed_corridor = False

# room2 perim bbox
bbox = [float("+inf"), float("-inf"), float("+inf"), float("-inf")]  # xmin,xmax,ymin,ymax
room2_started = 0.0
center_target = None

# для панорам/редкого анализа (необязательно)
spin_anchor = None

try:
    while True:
        tel = recv_tel()
        if not tel: continue
        (x,y,th),(vx,vy,vth),(wx,wy,wz),ranges = tel
        sm = smooth(ranges, SMOOTH_WIN)
        n=len(sm); front=sm[n//2]; left=sm[int(n*0.25)]; right=sm[int(n*0.75)]
        now = time.time()

        # moved?
        if last_pos is not None:
            if dist2((x,y), last_pos) > 0.02:  # >2см
                last_move = now
        last_pos = (x,y)

        # аварии отключаем во время CORNER_ESCAPE
        emergency_ok = (mode != Mode.CORNER_ESCAPE)

        # --- глобальные аварии ---
        if emergency_ok and min(front,left,right) < CRASH_DIST:
            if now - crash_t0 > CRASH_WIN: crash_streak = 0
            crash_streak += 1; crash_t0 = now
            if crash_streak >= 2:
                # старт ESC (в сторону свободы или всегда вправо)
                esc_dir = +1 if (TURN_POLICY=="right" or right>=left) else -1
                esc_phase = "back"; esc_until = now + ESC_BACK_TIME
                esc_target = ang_wrap(th + esc_dir*ESC_TURN_ANGLE)
                mode = Mode.CORNER_ESCAPE
                print(f"[ESC] start dir={'R' if esc_dir>0 else 'L'} target={math.degrees(esc_target):.1f}°")
                continue
            else:
                send_cmd(-0.22, math.copysign(0.6, (right-left)))
                print("[CRASH] short recover")
                continue

        # мягкое "stuck → recover" только если действительно едем вперёд и спереди тесно
        moving = (abs(vx)>0.03 or abs(vy)>0.03)
        if moving: last_move = now
        if (now-started_at)>3.0 and (now-last_move)>STUCK_TIME and last_cmd_v>0.2 and front<0.6 and emergency_ok:
            send_cmd(-0.22, math.copysign(0.6,(right-left)))
            print("[STUCK] recover")
            continue

        # --------------- режимы ---------------
        if mode == Mode.CORNER_ESCAPE:
            if esc_phase == "back":
                if now < esc_until:
                    send_cmd(-0.20, esc_dir*0.6)
                    print("[ESC] back…")
                    continue
                esc_phase = "turn"

            if esc_phase == "turn":
                err = ang_wrap(esc_target - th)
                if abs(err) > 0.10:
                    w = max(-ESC_W, min(ESC_W, 2.0*err))
                    send_cmd(0.0, w)
                    print(f"[ESC] turn… err={math.degrees(err):.1f}")
                    continue
                esc_phase = "forward"; esc_until = now + ESC_FWD_TIME
                print("[ESC] turn done → forward")

            if esc_phase == "forward":
                if now < esc_until:
                    send_cmd(0.25, 0.0)
                    print("[ESC] forward…")
                    continue
                crash_streak = 0
                esc_phase = None
                send_cmd(0.0,0.0)
                mode = Mode.FOLLOW_WALL
                print("[ESC] done → FOLLOW_WALL")
                continue

        elif mode == Mode.FOLLOW_WALL:
            # коридор? (узко по бокам и далеко вперёд) — запомним, что прошли коридор
            if right < 0.9 and left < 0.9 and front > 3.0:
                passed_corridor = True
            # открылась "комната"?
            if passed_corridor and looks_like_room(sm):
                mode = Mode.ROOM2_PERIM
                room2_started = now
                bbox = [x, x, y, y]
                print("[ROOM2] enter perimeter mode")
                # продолжаем wall-follow, но ещё пишем bbox

            # редкая панорама (необязательно; просто чтобы "подсветить" поведение)
            if spin_anchor is None: spin_anchor = (x,y)
            if dist2((x,y), spin_anchor) > SPIN_DISTANCE:
                spin_anchor = (x,y)  # просто метка на будущее

            v,w,sideR = wall_follow(sm, prefer_right)
            send_cmd(v,w)
            print(f"[WALL] v={v:.2f} w={w:.2f} F={front:.2f} L={left:.2f} R={right:.2f} side={'R' if sideR else 'L'}")

        elif mode == Mode.ROOM2_PERIM:
            # обновляем bbox посещённых поз
            bbox[0] = min(bbox[0], x); bbox[1] = max(bbox[1], x)
            bbox[2] = min(bbox[2], y); bbox[3] = max(bbox[3], y)

            # едем вдоль стены как обычно
            v,w,_ = wall_follow(sm, prefer_right)
            send_cmd(v,w)
            print(f"[PERIM] bbox=({bbox[0]:.2f},{bbox[1]:.2f},{bbox[2]:.2f},{bbox[3]:.2f})")

            # достаточно прошли — считаем центр
            if now - room2_started > PERIM_TIME_MIN:
                cx = 0.5*(bbox[0]+bbox[1]); cy = 0.5*(bbox[2]+bbox[3])
                center_target = (cx, cy)
                mode = Mode.GO_CENTER
                print(f"[ROOM2] center target = {center_target}")
                continue

        elif mode == Mode.GO_CENTER:
            if center_target is None:
                mode = Mode.FOLLOW_WALL
                continue
            d = dist2((x,y), center_target)
            if d < CENTER_RADIUS and looks_like_room(sm):
                mode = Mode.STOP
                send_cmd(0.0,0.0)
                print("[STOP] reached center")
                continue

            # наведение на точку + локальная объездка
            if front < SAFE_DIST:
                bias = math.copysign(W_MAX*0.9, (right-left))
                send_cmd(0.0, bias)
                print("[CENTER] avoid")
            else:
                dx,dy = center_target[0]-x, center_target[1]-y
                ang = ang_wrap(math.atan2(dy,dx) - th)
                w = max(-W_MAX, min(W_MAX, 1.6*ang))
                v = max(V_MIN, min(V_MAX, 0.6*math.hypot(dx,dy)))
                v *= max(0.25, 1.0 - abs(w)/W_MAX)
                send_cmd(v,w)
                print(f"[CENTER] d={d:.2f} v={v:.2f} w={w:.2f}")

        elif mode == Mode.STOP:
            send_cmd(0.0,0.0)
            print("[STOP] hold (debug)")
            time.sleep(0.5)
            continue

except KeyboardInterrupt:
    send_cmd(0.0,0.0)
    print("[client] stop")
    sock_cmd.close()
    sock_tel.close()
    print("[client] bye")