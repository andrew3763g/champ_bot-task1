# demo_gap.py
import math, os, socket, struct, time
from enum import Enum

CMD_HOST  = os.getenv("CMD_HOST", "127.0.0.1")
CMD_PORT  = int(os.getenv("CMD_PORT", "5555"))
TEL_HOST  = os.getenv("TEL_HOST", "0.0.0.0")
TEL_PORT  = int(os.getenv("TEL_PORT", "5600"))
PROTO     = os.getenv("PROTO", "tcp")

# --- Тюнингуемые параметры ---
SAFE_DIST      = 0.70   # безопасная дистанция для просвета, м
CRASH_DIST     = 0.30   # «почти столкновение»
MAX_RANGE_CLIP = 5.00   # срез верхних выбросов лидара
SMOOTH_WIN     = 5      # сглаживание (число лучей в окне)
BUBBLE_FRAC    = 0.05   # ширина «бан-вокруг-ближайшего» как доля от n
V_MAX          = 0.80
V_MIN          = 0.10
K_V            = 0.40   # усиление по свободному фронту
K_STEER        = 2.2    # усиление руления (рад/с на рад)
W_MAX          = 1.4
STUCK_TIME     = 2.0    # если столько не двигались — «застряли»
RECOVER_TIME   = 1.0    # длительность отъезда назад при выходе
LONG_TURN_TIME = 0.9    # длительность разворота на месте в тупике

class Mode(Enum):
    EXPLORE_MOVE = 1
    SPIN_MAP = 2
    DOORWAY_LOCK = 3
    ROOM2_CENTER = 4
    STOP = 5

mode = Mode.EXPLORE_MOVE
last_spin_pose = None
spin_accum = []      # (ang_rel, r) при вращении
spin_started_th = None
target_center = None
door_lock_until = 0.0

def dist(a,b): return math.hypot(a[0]-b[0], a[1]-b[1])

# --- детекция "тоннеля" (дверь/коридор) ---
def looks_like_door(front, left, right):
    return front > 3.5 and left < 0.8 and right < 0.8

# --- простая проверка "широкая зона" (комната) ---
def looks_like_room(front, left, right):
    return front > 4.0 and left > 1.8 and right > 1.8

# --- панорама завершается, когда развернулись примерно на 360° ---
def spin_done(curr_th):
    d = (curr_th - spin_started_th + math.pi) % (2*math.pi) - math.pi
    return abs(d) > (2*math.pi - 0.2)

# --- из панорамы получить центр bbox ---
def center_from_spin(spin_points, pose):
    if not spin_points: return None
    xs=[]; ys=[]
    for ang, r in spin_points:
        r = max(0.0, min(r, MAX_RANGE_CLIP))
        if r < 0.25: continue
        x = r*math.cos(ang); y = r*math.sin(ang)
        xs.append(x); ys.append(y)
    if len(xs) < 50: return None
    # перцентильный bbox устойчивее к мусору
    xs.sort(); ys.sort()
    x_lo = xs[int(0.10*len(xs))]; x_hi = xs[int(0.90*len(xs))]
    y_lo = ys[int(0.10*len(ys))]; y_hi = ys[int(0.90*len(ys))]
    cx_local = 0.5*(x_lo + x_hi)
    cy_local = 0.5*(y_lo + y_hi)
    Xr, Yr, Th = pose
    Xc = Xr + cx_local*math.cos(Th) - cy_local*math.sin(Th)
    Yc = Yr + cx_local*math.sin(Th) + cy_local*math.cos(Th)
    return (Xc, Yc)

# --- наведение на точку ---
def steer_to_point(pose, goal):
    Xr, Yr, Th = pose
    dx = goal[0]-Xr; dy = goal[1]-Yr
    ang = math.atan2(dy, dx) - Th
    ang = (ang + math.pi)%(2*math.pi) - math.pi
    w = max(-W_MAX, min(W_MAX, 1.8*ang))
    v = max(V_MIN, min(V_MAX, 0.6*math.hypot(dx,dy)))
    v *= max(0.25, 1.0 - abs(w)/W_MAX)
    return v, w

# --- Сеть ---
sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
if PROTO == "udp":
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); sock_tel.bind((TEL_HOST, TEL_PORT))
else:
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_tel.bind((TEL_HOST, TEL_PORT)); sock_tel.listen(1)
    print(f"[client] waiting TCP {TEL_HOST}:{TEL_PORT} …"); conn, _ = sock_tel.accept(); sock_tel = conn
    print("[client] connected to udp_diff telemetry")

def send_cmd(v: float, w: float):
    sock_cmd.sendto(struct.pack("<2f", float(v), float(w)), (CMD_HOST, CMD_PORT))

def recv_all(s, n):
    buf=b""; 
    while len(buf)<n:
        chunk=s.recv(n-len(buf))
        if not chunk: return None
        buf+=chunk
    return buf

def recv_tel():
    if PROTO == "udp":
        data,_ = sock_tel.recvfrom(65535)
    else:
        size_b = sock_tel.recv(4)
        if not size_b: return None
        data = recv_all(sock_tel, struct.unpack("<I", size_b)[0])
    if not data or not data.startswith(b"WBTG"): return None
    hdr = 4 + 9*4
    x,y,th,vx,vy,vth,wx,wy,wz = struct.unpack("<9f", data[4:hdr])
    n = struct.unpack("<I", data[hdr:hdr+4])[0]
    rng = ()
    if n>0: rng = struct.unpack(f"<{n}f", data[hdr+4:hdr+4+4*n])
    return (x,y,th),(vx,vy,vth),(wx,wy,wz),list(rng)

# --- Вспомогательные ---
def smooth(arr, win=3):
    if win<=1: return arr[:]
    out=[]; m=len(arr)
    half=win//2
    for i in range(m):
        lo=max(0,i-half); hi=min(m,i+half+1)
        out.append(sum(arr[lo:hi])/(hi-lo))
    return out

def find_largest_gap(masked, thresh):
    # возвращает (start, end) индексы наибольшего отрезка > thresh
    best=(0,-1); cur=(-1,-1); n=len(masked)
    i=0
    while i<n:
        while i<n and masked[i]<=thresh: i+=1
        if i==n: break
        s=i
        while i<n and masked[i]>thresh: i+=1
        e=i-1
        if e-s > best[1]-best[0]: best=(s,e)
    return best

def idx_to_angle(idx, n):
    # предполагаем фронт по центру, сектор ~[-pi/2, +pi/2]
    mid = n//2
    return (idx - mid) * (math.pi/2) / max(1, mid)

# --- Основной цикл ---
last_pos=None; last_move=time.time(); recover_until=0; long_turn_until=0

try:
    while True:
        tel = recv_tel()
        if not tel: continue
        (x,y,th),(vx,vy,vth),(gx,gy,gz),ranges = tel
        if not ranges: continue
        now=time.time()

        # анти-застревание по одометрии
        if last_pos is not None:
            dx=x-last_pos[0]; dy=y-last_pos[1]
            if dx*dx+dy*dy > 0.02*0.02:  # сдвиг > 2 см
                last_move=now
        last_pos=(x,y)
        stuck = (now - last_move) > STUCK_TIME

        # предобработка лидара
        n=len(ranges)
        clipped=[min(MAX_RANGE_CLIP, max(0.0, r)) for r in ranges]
        sm = smooth(clipped, SMOOTH_WIN)

        # бан-вокруг ближайшего (safety bubble)
        closest_i = min(range(n), key=lambda i: sm[i])
        bubble = max(1, int(BUBBLE_FRAC*n))
        masked = sm[:]
        for i in range(max(0,closest_i-bubble), min(n,closest_i+bubble+1)):
            masked[i]=0.0

        # ищем самый большой просвет
        g0,g1 = find_largest_gap(masked, SAFE_DIST)
        front = sm[n//2]
        left  = sm[int(n*0.25)]
        right = sm[int(n*0.75)]

        if now < recover_until:
            # продолжаем откат
            v=-0.2; w=0.8
        elif now < long_turn_until:
            # разворот на месте в тупике
            v=0.0; w=math.copysign(W_MAX*0.9, (right-left))
        elif front < CRASH_DIST or left < CRASH_DIST or right < CRASH_DIST:
            # аварийный откат
            recover_until = now + RECOVER_TIME
            v=-0.25; w=math.copysign(1.0, (right-left))
        elif g1 <= g0:  # просвет не найден (очень узко) — долгий разворот
            long_turn_until = now + LONG_TURN_TIME
            v=0.0; w=math.copysign(W_MAX*0.9, (right-left))
        else:
            # руление в центр просвета
            center = (g0+g1)//2
            ang = idx_to_angle(center, n)  # желаемый угол
            w = max(-W_MAX, min(W_MAX, K_STEER*ang))
            # скорость — от свободного фронта и величины поворота
            v = V_MIN + K_V*min(front, 2.0)
            v *= max(0.25, 1.0 - abs(w)/W_MAX)

        send_cmd(v, w)
        print(f"[gap] pos=({x:.2f},{y:.2f}) θ={math.degrees(th):.1f}° "
              f"front={front:.2f} L={left:.2f} R={right:.2f} "
              f"v={v:.2f} w={w:.2f}  gap=({g0},{g1}) stuck={stuck}")

except KeyboardInterrupt:
    send_cmd(0.0,0.0)
    print("[gap] stop")
