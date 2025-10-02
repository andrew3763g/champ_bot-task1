# step.py - Пошаговое ручное управление роботом для построения графа узлов
import math, os, socket, struct, time, msvcrt, sys
from datetime import datetime

CMD_HOST  = os.getenv("CMD_HOST", "127.0.0.1")   # для Docker Desktop: host.docker.internal
CMD_PORT  = int(os.getenv("CMD_PORT", "5555"))
TEL_HOST  = os.getenv("TEL_HOST", "0.0.0.0")
TEL_PORT  = int(os.getenv("TEL_PORT", "5600"))
PROTO     = os.getenv("PROTO", "tcp")            # "tcp" (по умолчанию) или "udp"

MAX_RANGE_CLIP = 8.0   # многие говорят, что «нет отклика» = ~8 м → считаем это "дальше макс."

# Открываем лог-файл
log_file = open("step.log", "w", encoding="utf-8")

def log_print(msg):
    """Печатает в консоль и записывает в лог"""
    print(msg)
    log_file.write(msg + "\n")
    log_file.flush()

# --- сеть ---
sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
if PROTO == "udp":
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_tel.bind((TEL_HOST, TEL_PORT))
else:
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_tel.bind((TEL_HOST, TEL_PORT))
    sock_tel.listen(1)
    print(f"[step] waiting TCP {TEL_HOST}:{TEL_PORT} …")
    conn, _ = sock_tel.accept()
    sock_tel = conn
    print("[step] connected to telemetry")

def send_cmd(v, w):
    sock_cmd.sendto(struct.pack("<2f", float(v), float(w)), (CMD_HOST, CMD_PORT))

def _recv_all(s, n):
    buf=b""
    while len(buf) < n:
        chunk = s.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf

def recv_tel(timeout=0.5):
    """Получить телеметрию с таймаутом"""
    old_timeout = sock_tel.gettimeout()
    try:
        sock_tel.settimeout(timeout)
        if PROTO == "udp":
            data,_ = sock_tel.recvfrom(65535)
        else:
            sz = sock_tel.recv(4)
            if not sz: return None
            data = _recv_all(sock_tel, struct.unpack("<I", sz)[0])
        if not data or not data.startswith(b"WBTG"):
            return None
        hdr = 4 + 9*4
        x,y,th,vx,vy,vth,wx,wy,wz = struct.unpack("<9f", data[4:hdr])
        n = struct.unpack("<I", data[hdr:hdr+4])[0]
        rng=[]
        if n>0:
            rng = list(struct.unpack(f"<{n}f", data[hdr+4:hdr+4+4*n]))
        # 0.0 трактуем как «дальше максимума»
        rng = [MAX_RANGE_CLIP if r<=0.0 else min(MAX_RANGE_CLIP, r) for r in rng]
        return (x,y,th),(vx,vy,vth),(wx,wy,wz),rng
    except socket.timeout:
        return None
    finally:
        sock_tel.settimeout(old_timeout)

def angle_wrap(a):
    """Нормализует угол в диапазон [-π, π]"""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def clear_telemetry_buffer():
    """Очистить буфер телеметрии (прочитать все старые пакеты)"""
    sock_tel.setblocking(False)
    cleared = 0
    try:
        while True:
            try:
                if PROTO == "udp":
                    sock_tel.recvfrom(65535)
                else:
                    sock_tel.recv(65535)
                cleared += 1
            except:
                break
    finally:
        sock_tel.setblocking(True)
    return cleared

def move_distance(dist, v=0.28, tol=0.02, timeout=8.0):
    """Проехать dist (м) со скоростью v (м/с). Положительная вперёд, отрицательная назад."""
    # КРИТИЧНО: Очистить буфер перед началом движения
    cleared = clear_telemetry_buffer()
    if cleared > 0:
        print(f"[Очищено {cleared} старых пакетов перед движением]")

    tel = recv_tel(timeout=1.0)
    if not tel:
        print("⚠ Не удалось получить телеметрию")
        return
    pose0, _, _, rng0 = tel
    x0, y0, th0 = pose0

    # Проверка препятствий перед движением
    if rng0:
        n = len(rng0)
        front = rng0[n//2] if n > 0 else MAX_RANGE_CLIP
        if dist > 0 and front < 0.25:  # Снижен порог с 0.4 до 0.25м для прохода в туннеле
            log_print(f"[WARNING] Не могу ехать вперёд - препятствие на расстоянии {front:.2f}м")
            print(f"⚠ Препятствие впереди ({front:.2f}м) - движение отменено")
            return

    t0=time.time()
    while True:
        tel = recv_tel(timeout=0.5)
        if not tel:
            print("⚠ Потеряна телеметрия")
            break
        pose, vel, gyro, rng = tel
        x, y, th = pose
        d = math.hypot(x-x0, y-y0)
        if d >= abs(dist)-tol: break
        if time.time()-t0 > timeout:
            log_print(f"[WARNING] Таймаут движения - проехали только {d:.2f}м из {abs(dist):.2f}м")
            break
        # простой регулятор: плавно тормозим к цели
        remain = abs(dist)-d
        vcmd = max(0.12, min(v, 0.6*remain))
        vcmd = vcmd if dist>0 else -vcmd
        send_cmd(vcmd, 0.0)
        time.sleep(0.02)
    send_cmd(0.0,0.0)
    time.sleep(0.2)  # Дать время роботу остановиться и телеметрии обновиться

def turn_angle(delta, w=0.8, tol=0.02, timeout=6.0):
    """Повернуться на delta (рад), + вправо, - влево."""
    # КРИТИЧНО: Очистить буфер перед началом поворота
    cleared = clear_telemetry_buffer()
    if cleared > 0:
        print(f"[Очищено {cleared} старых пакетов перед поворотом]")

    tel = recv_tel(timeout=1.0)
    if not tel:
        print("⚠ Не удалось получить телеметрию")
        return
    pose0, _, _, rng0 = tel
    x0, y0, th0 = pose0

    # Проверка: может ли робот поворачиваться (не прижат ли к стене)
    if rng0:
        n = len(rng0)
        front = rng0[n//2] if n > 0 else MAX_RANGE_CLIP
        left = rng0[n//4] if n > 0 else MAX_RANGE_CLIP
        right = rng0[3*n//4] if n > 0 else MAX_RANGE_CLIP
        if min(front, left, right) < 0.25:
            log_print(f"[WARNING] Слишком близко к стене (F={front:.2f} L={left:.2f} R={right:.2f}) - сначала отъедьте назад")
            print(f"⚠ Прижат к стене - сначала нажмите ↓ для отъезда назад")
            return

    target = angle_wrap(th0 + delta)
    t0=time.time()
    th_start = th0
    while True:
        tel = recv_tel(timeout=0.5)
        if not tel:
            print("⚠ Потеряна телеметрия")
            break
        pose, vel, gyro, rng = tel
        x, y, th = pose
        th = angle_wrap(th)  # Нормализуем текущий угол!
        err = angle_wrap(target - th)
        if abs(err) < tol: break
        if time.time()-t0 > timeout:
            turned = abs(angle_wrap(th - th_start))
            log_print(f"[WARNING] Таймаут поворота - повернулись на {math.degrees(turned):.1f}° из {math.degrees(abs(delta)):.1f}°")
            break
        wcmd = max(0.25, min(w, 2.0*abs(err))) * (1 if err>0 else -1)
        send_cmd(0.0, wcmd)
        time.sleep(0.02)
    send_cmd(0.0,0.0)
    time.sleep(0.2)  # Дать время роботу остановиться и телеметрии обновиться

def print_scan(header=None):
    """Печатает текущую телеметрию и данные лидара"""
    # Очистить старую телеметрию перед чтением
    cleared = clear_telemetry_buffer()

    tel = recv_tel(timeout=1.0)
    if not tel:
        log_print("[WARNING] Не удалось получить телеметрию для скана")
        print("⚠ Телеметрия недоступна")
        return None

    pose, vel, _, rng = tel
    (x,y,th) = pose
    n=len(rng)
    mid=n//2
    left = rng[int(n*0.25)] if n>0 else float("nan")
    right= rng[int(n*0.75)] if n>0 else float("nan")
    front= rng[mid] if n>0 else float("nan")

    if header:
        log_print(header)
    if cleared > 0:
        log_print(f"[INFO] Пропущено {cleared} старых пакетов")

    log_print(f"[TELEMETRY] pos=({x:.2f},{y:.2f}) th={math.degrees(th):.1f}°  F={front:.2f} L={left:.2f} R={right:.2f}")

    # Все лучи (только в лог, чтобы не засорять консоль)
    log_file.write(f"[LIDAR] beams={n} ranges: " + ", ".join(f"{r:.2f}" for r in rng) + "\n")
    log_file.flush()

    return (x, y, th), rng

def read_key():
    """Читает клавишу с клавиатуры (неблокирующий режим)"""
    # стрелки в Windows: сначала 0xE0 или 0x00, затем код
    if not msvcrt.kbhit(): return None
    ch = msvcrt.getch()
    if ch in (b'\xe0', b'\x00'):
        code = msvcrt.getch()
        return {b'H':'UP', b'P':'DOWN', b'K':'LEFT', b'M':'RIGHT'}.get(code, None)
    ch = ch.decode(errors='ignore').lower()
    if ch == 'q': return 'QUIT'
    if ch == 's': return 'SCAN'
    if ch == 'w': return 'WRITE_NODE'
    if ch == 'c': return 'COMMENT'
    if ch in ('1', '2', '3', '4'): return ('STEP', int(ch))
    # Точные повороты
    if ch == 'a': return 'TURN_LEFT_SMALL'   # π/32 влево
    if ch == 'd': return 'TURN_RIGHT_SMALL'  # π/32 вправо
    if ch == 'z': return 'TURN_LEFT_TINY'    # π/64 влево
    if ch == 'x': return 'TURN_RIGHT_TINY'   # π/64 вправо
    return None

def write_node_comment():
    """Запрашивает комментарий для узла графа и записывает в лог"""
    # КРИТИЧНО: Остановить робота перед записью узла!
    send_cmd(0.0, 0.0)
    time.sleep(0.3)  # Дать роботу остановиться

    # Очистить буфер и получить свежую телеметрию
    clear_telemetry_buffer()
    tel = recv_tel(timeout=1.0)
    if not tel:
        print("✗ Не удалось получить телеметрию")
        return None
    pose, _, _, rng = tel
    x, y, th = pose

    print("\n" + "="*60)
    print("ЗАПИСЬ УЗЛА ГРАФА")
    print("="*60)
    print(f"Текущая позиция: ({x:.2f}, {y:.2f}), угол: {math.degrees(th):.1f}°")
    print("Введите название узла (например: 'START', 'NODE_1', 'CORNER_NW'):")
    print("> ", end='', flush=True)

    # Читаем название узла (блокирующий ввод)
    node_name = input().strip()

    if node_name:
        log_print("\n" + "="*60)
        log_print(f"[NODE] {node_name}")
        log_print(f"[NODE] pos=({x:.2f},{y:.2f}) th={math.degrees(th):.1f}°")
        log_print("="*60 + "\n")
        print(f"✓ Узел '{node_name}' записан в step.log")
    else:
        print("✗ Узел не записан (пустое название)")

    # КРИТИЧНО: Очистить буфер телеметрии перед продолжением!
    cleared = clear_telemetry_buffer()
    if cleared > 0:
        print(f"[Очищено {cleared} старых пакетов телеметрии]")

    # Очистка буфера клавиатуры
    while msvcrt.kbhit():
        msvcrt.getch()

    print("[Готово! Можно продолжать управление]\n")

    return node_name

def write_comment():
    """Добавляет текстовый комментарий в лог"""
    # Остановить робота
    send_cmd(0.0, 0.0)
    time.sleep(0.3)

    print("\n" + "-"*60)
    print("Введите комментарий (например: 'проехал мимо утки слева'):")
    print("> ", end='', flush=True)

    comment = input().strip()

    if comment:
        log_print(f"[COMMENT] {comment}")
        print(f"✓ Комментарий записан в step.log")
    else:
        print("✗ Комментарий не записан (пустой)")

    # Очистить буфер телеметрии
    cleared = clear_telemetry_buffer()
    if cleared > 0:
        print(f"[Очищено {cleared} старых пакетов телеметрии]")

    # Очистка буфера клавиатуры
    while msvcrt.kbhit():
        msvcrt.getch()

    print("[Готово! Можно продолжать управление]\n")

def main():
    log_print("="*60)
    log_print("STEP.PY - Ручное управление роботом для построения графа узлов")
    log_print(f"Запуск: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    log_print("="*60)

    print("\n" + "="*60)
    print("УПРАВЛЕНИЕ РОБОТОМ - Построение графа узлов")
    print("="*60)
    print("Клавиши:")
    print("  ↑ / ↓      - движение вперёд/назад на 0.5 м")
    print("  → / ←      - поворот вправо/влево на π/16 (11.25°)")
    print("  1/2/3/4    - малые шаги: 0.1/0.2/0.3/0.4 м вперёд")
    print("")
    print("  A / D      - точный поворот влево/вправо на π/32 (5.625°)")
    print("  Z / X      - очень точный поворот влево/вправо на π/64 (2.8°)")
    print("")
    print("  S          - скан (показать телеметрию и лидар)")
    print("  W          - ЗАПИСАТЬ УЗЕЛ ГРАФА (с координатами)")
    print("  C          - добавить комментарий в лог")
    print("  Q          - выход")
    print("="*60)
    print("Совет: A/D для коррекции курса, Z/X для финальной точности")
    print("См. STEP_KEYS.md для подробной инструкции")
    print("="*60 + "\n")

    print_scan("[STEP] Начальное состояние")

    last_clear_time = time.time()

    while True:
        # Периодически очищать буфер телеметрии (каждые 5 секунд простоя)
        now = time.time()
        if now - last_clear_time > 5.0:
            cleared = clear_telemetry_buffer()
            if cleared > 10:  # Только если накопилось много
                print(f"[AUTO] Очищено {cleared} старых пакетов")
            last_clear_time = now

        k = read_key()
        if not k:
            time.sleep(0.01)
            continue

        if k == 'QUIT':
            send_cmd(0.0, 0.0)
            log_print("[STEP] Завершение программы")
            print("[STEP] Выход...")
            break
        elif k == 'UP':
            last_clear_time = now  # Сбросить таймер
            log_print("[STEP] Команда: вперёд 0.50 м")
            move_distance(+0.50)
            print_scan("[STEP] После движения вперёд")
        elif k == 'DOWN':
            last_clear_time = now
            log_print("[STEP] Команда: назад 0.50 м")
            move_distance(-0.50)
            print_scan("[STEP] После движения назад")
        elif k == 'RIGHT':
            last_clear_time = now
            log_print("[STEP] Команда: поворот вправо π/16 (11.25°)")
            turn_angle(-math.pi/16)
            print_scan("[STEP] После поворота вправо")
        elif k == 'LEFT':
            last_clear_time = now
            log_print("[STEP] Команда: поворот влево π/16 (11.25°)")
            turn_angle(+math.pi/16)
            print_scan("[STEP] После поворота влево")
        elif k == 'TURN_RIGHT_SMALL':
            last_clear_time = now
            log_print("[STEP] Команда: точный поворот вправо π/32 (5.625°)")
            turn_angle(-math.pi/32)
            print_scan("[STEP] После точного поворота вправо")
        elif k == 'TURN_LEFT_SMALL':
            last_clear_time = now
            log_print("[STEP] Команда: точный поворот влево π/32 (5.625°)")
            turn_angle(+math.pi/32)
            print_scan("[STEP] После точного поворота влево")
        elif k == 'TURN_RIGHT_TINY':
            last_clear_time = now
            log_print("[STEP] Команда: очень точный поворот вправо π/64 (2.8125°)")
            turn_angle(-math.pi/64)
            print_scan("[STEP] После очень точного поворота вправо")
        elif k == 'TURN_LEFT_TINY':
            last_clear_time = now
            log_print("[STEP] Команда: очень точный поворот влево π/64 (2.8125°)")
            turn_angle(+math.pi/64)
            print_scan("[STEP] После очень точного поворота влево")
        elif isinstance(k, tuple) and k[0] == 'STEP':
            last_clear_time = now
            step_size = k[1] * 0.1  # 1->0.1, 2->0.2, 3->0.3, 4->0.4
            log_print(f"[STEP] Команда: малый шаг вперёд {step_size:.1f} м")
            move_distance(step_size)
            print_scan(f"[STEP] После малого шага {step_size:.1f} м")
        elif k == 'SCAN':
            last_clear_time = now
            print_scan("[STEP] Скан по запросу")
        elif k == 'WRITE_NODE':
            last_clear_time = now
            write_node_comment()
        elif k == 'COMMENT':
            last_clear_time = now
            write_comment()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        send_cmd(0.0, 0.0)
        log_print("\n[STEP] Прервано пользователем (Ctrl+C)")
        print("\n[STEP] Прервано пользователем")
    finally:
        send_cmd(0.0, 0.0)
        log_file.close()
        print("[STEP] Лог сохранён в step.log")
