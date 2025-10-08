#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ВАРИАНТ A: Быстрый навигатор с минимальными улучшениями

Ключевые идеи:
1. flush_telemetry() - сбрасывать старые пакеты перед важными замерами
2. Движение без задержек - не ждать между шагами на открытых участках
3. LIDAR-навигация - для туннеля и узких мест
4. Детальное логирование - всё записывать для анализа

Координатная система (РЕАЛЬНОСТЬ):
- X+ = ВОСТОК (влево на карте)
- Y+ = СЕВЕР (вверх)
- th=0° = СЕВЕР
- th=90° = ВОСТОК
- LIDAR: L = СПРАВА, R = СЛЕВА (названия перепутаны!)
"""
import math
import os
import socket
import struct
import time
import sys

# === Настройки ===
CMD_HOST = os.getenv("CMD_HOST", "127.0.0.1")
CMD_PORT = int(os.getenv("CMD_PORT", "5555"))
TEL_HOST = os.getenv("TEL_HOST", "0.0.0.0")
TEL_PORT = int(os.getenv("TEL_PORT", "5600"))
PROTO = os.getenv("PROTO", "tcp")

MAX_RANGE_CLIP = 8.0
SLIP_FACTOR = 0.89  # Компенсация пробуксовки
TOTAL_TIME_LIMIT = 999.0  # ОТЛАДКА: без лимита, смотрим как далеко заедем
ROBOT_WIDTH = 0.3  # Примерная ширина робота

# === Шаги и повороты ===
STEP_LARGE = 1.0    # Большой шаг для открытых пространств
STEP_MEDIUM = 0.5   # Средний шаг (основной)
STEP_SMALL = 0.2    # Малый шаг для точности
STEP_TINY = 0.1     # Очень малый шаг

TURN_LARGE = math.pi / 8    # 22.5° (π/8)
TURN_MEDIUM = math.pi / 16  # 11.25° (π/16)
TURN_SMALL = math.pi / 32   # 5.625° (π/32)
TURN_TINY = math.pi / 64    # 2.8125° (π/64)

# === ПРАВИЛЬНЫЙ МАРШРУТ из step.log ===
# ВАЖНО: На рампе туннеля LIDAR светит в воздух (F=8.00)!
# Поэтому проезжаем рампу "вслепую" по расстоянию, без LIDAR-навигации
ROUTE = [
    # (x, y, th_deg, description, mode)

    # 0. START
    (0.00,   0.00,   0.0,   "START", "odometry"),

    # 1. Выезд задом
    (-1.95,  0.00,   0.0,   "Выезд из кармана", "odometry"),

    # 2. Поворот на восток
    (-1.94,  0.00,   85.3,  "Поворот на восток", "odometry"),

    # 3. Мимо первой утки
    (-1.83,  1.43,   85.3,  "Мимо первой утки", "odometry"),

    # 4. Диагональ (экономим расстояние!)
    (1.24,   5.72,   54.5,  "Диагональ к воротам", "odometry"),

    # 5. Перед воротами туннеля
    (1.22,   6.67,   91.0,  "Перед воротами (F=2.57 L=2.78 R=2.79)", "odometry"),

    # 6. Корректировка
    (0.06,   8.18,   127.6, "Корректировка (F=1.93 L=1.17 R=3.09)", "odometry"),

    # 7. ВЪЕЗД В ТУННЕЛЬ
    (-1.61,  8.99,   87.0,  "ВЪЕЗД В ТУННЕЛЬ (F=0.40 узко!)", "odometry"),

    # 8. НА РАМПЕ - LIDAR СЛЕПНЕТ! Едем "вслепую" по расстоянию
    # Из step.log: проехали ~5 шагов по 0.45м = 2.25м
    # Комментарий: "when we drive up the ramp there is no point in navigating by radar"
    # ЭТОТ УЗЕЛ ПРОПУСКАЕМ! Координаты врут, LIDAR не работает.

    # 9. ПОСЛЕ РАМПЫ - LIDAR ВОССТАНОВИЛСЯ!
    # Комментарий: "radar is useless again" → "lidar is hitting the room floor"
    # Переключаемся на LIDAR-навигацию только ЗДЕСЬ!
    (-0.95,  16.05,  83.9,  "Коридор комнаты 2 (LIDAR восстановился: F=1.21 L=1.19 R=1.46)", "lidar"),

    # 10. Угол
    (-5.22,  16.37,  175.7, "Угол (F=0.62)", "lidar"),

    # 11. К финишу
    (-4.82,  20.16,  83.9,  "К финишу (F=5.13 свободно)", "lidar"),

    # 12. ФИНИШ
    (-1.91,  19.78,  -7.3,  "ФИНИШ (F=6.24 L=3.34 R=6.77)", "lidar"),
]

NODE_REVERSE = 1  # Узел, где едем задом
NODE_TUNNEL_START = 9  # Начало LIDAR-навигации (ПОСЛЕ рампы!)

# === Глобальные переменные ===
start_time = None
log_file = None

# === Сеть ===
sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

if PROTO == "udp":
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_tel.bind((TEL_HOST, TEL_PORT))
else:
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_tel.bind((TEL_HOST, TEL_PORT))
    sock_tel.listen(1)
    print(f"[NAV] Ожидание TCP {TEL_HOST}:{TEL_PORT}...")
    conn, _ = sock_tel.accept()
    sock_tel = conn
    print("[NAV] Подключено")

# === Логирование ===

def log_print(msg):
    """Вывод в консоль и файл"""
    global log_file
    print(msg)
    if log_file:
        log_file.write(msg + "\n")
        log_file.flush()

def log_telemetry(tel, prefix=""):
    """Логирование телеметрии"""
    if not tel:
        log_print(f"{prefix}[TEL] TIMEOUT")
        return

    pose, vel, gyro, lidar = tel
    x, y, th = pose
    th_deg = math.degrees(th)

    # LIDAR: фронт, слева (R), справа (L)
    F = lidar[len(lidar)//2] if lidar else 8.0
    L = lidar[len(lidar)//4] if lidar else 8.0
    R = lidar[3*len(lidar)//4] if lidar else 8.0

    log_print(f"{prefix}[TEL] pos=({x:.2f},{y:.2f}) th={th_deg:.1f}°  F={F:.2f} L={L:.2f} R={R:.2f}")

# === Идея №1: Сброс старых пакетов телеметрии ===

def flush_telemetry():
    """
    Сбрасывает старые пакеты из буфера
    КРИТИЧНО: вызывать перед каждым важным замером!
    """
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

    if cleared > 0:
        log_print(f"[FLUSH] Сброшено {cleared} старых пакетов")
    return cleared

def get_fresh_telemetry(timeout=1.0):
    """
    Получить СВЕЖИЙ пакет телеметрии
    1. Сбросить старые пакеты
    2. Подождать немного
    3. Прочитать новый пакет
    """
    flush_telemetry()
    time.sleep(0.05)  # Дать время серверу отправить новый пакет
    return recv_tel(timeout=timeout)

# === Базовые функции ===

def send_cmd(v, w):
    """Отправить команду роботу"""
    sock_cmd.sendto(struct.pack("<2f", float(v), float(w)), (CMD_HOST, CMD_PORT))

def _recv_all(s, n):
    """Прочитать ровно n байт"""
    buf = b""
    while len(buf) < n:
        chunk = s.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf

def recv_tel(timeout=0.5):
    """Получить телеметрию"""
    old_timeout = sock_tel.gettimeout()
    try:
        sock_tel.settimeout(timeout)
        if PROTO == "udp":
            data, _ = sock_tel.recvfrom(65535)
        else:
            sz = sock_tel.recv(4)
            if not sz:
                return None
            data = _recv_all(sock_tel, struct.unpack("<I", sz)[0])

        if not data or not data.startswith(b"WBTG"):
            return None

        hdr = 4 + 9 * 4
        x, y, th, vx, vy, vth, wx, wy, wz = struct.unpack("<9f", data[4:hdr])
        n = struct.unpack("<I", data[hdr:hdr+4])[0]
        rng = []
        if n > 0:
            rng = list(struct.unpack(f"<{n}f", data[hdr+4:hdr+4+4*n]))
        rng = [MAX_RANGE_CLIP if r <= 0.0 else min(MAX_RANGE_CLIP, r) for r in rng]
        return (x, y, th), (vx, vy, vth), (wx, wy, wz), rng
    except socket.timeout:
        return None
    finally:
        sock_tel.settimeout(old_timeout)

def angle_wrap(a):
    """Нормализация угла в диапазон [-π, π]"""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def extract_lidar_frl(lidar):
    """
    Извлечь Front, Right (на самом деле ЛЕВЫЙ!), Left (на самом деле ПРАВЫЙ!)
    ВНИМАНИЕ: L = СПРАВА, R = СЛЕВА (названия перепутаны!)
    """
    if not lidar or len(lidar) == 0:
        return 8.0, 8.0, 8.0

    F = lidar[len(lidar)//2]       # 180° - впереди
    R = lidar[3*len(lidar)//4]     # 270° - слева (!) по курсу
    L = lidar[len(lidar)//4]       # 90° - справа (!) по курсу

    return F, R, L  # Возвращаем F, R=СЛЕВА, L=СПРАВА

def elapsed_time():
    """Время с начала"""
    global start_time
    if start_time is None:
        return 0.0
    return time.time() - start_time

# === Идея №2: Быстрое движение без задержек ===

def move_forward_fast(distance, v=0.4):
    """
    Движение вперёд БЕЗ ОЖИДАНИЯ И БЕЗ ПРОВЕРОК
    Используется на открытых участках для скорости
    """
    adjusted = distance / SLIP_FACTOR
    duration = adjusted / v

    log_print(f"[MOVE_FAST] Вперёд {distance:.2f}м (v={v:.2f}, время≈{duration:.1f}с)")

    send_cmd(v, 0.0)
    time.sleep(duration)
    send_cmd(0.0, 0.0)
    time.sleep(0.1)  # Минимальная пауза для остановки

def move_forward_precise(distance, v=0.25, timeout=5.0):
    """
    Точное движение вперёд с контролем расстояния
    Используется для финального подхода к узлу
    """
    flush_telemetry()
    tel = get_fresh_telemetry()

    if not tel:
        log_print(f"[MOVE_PRECISE] ОШИБКА: нет телеметрии")
        return False

    pose0, _, _, _ = tel
    x0, y0, th0 = pose0

    log_print(f"[MOVE_PRECISE] Вперёд {distance:.2f}м от ({x0:.2f},{y0:.2f})")

    adjusted = distance / SLIP_FACTOR
    t0 = time.time()

    while time.time() - t0 < timeout:
        tel = recv_tel(timeout=0.3)
        if not tel:
            break

        pose, _, _, lidar = tel
        x, y, th = pose
        d = math.hypot(x - x0, y - y0)

        if d >= distance * 0.95:
            log_print(f"[MOVE_PRECISE] Достигнуто {d:.2f}м")
            break

        # Проверка препятствий
        F, R, L = extract_lidar_frl(lidar)
        if F < 0.4:
            log_print(f"[MOVE_PRECISE] ПРЕПЯТСТВИЕ! F={F:.2f}м")
            break

        remain = adjusted - d
        vcmd = min(v, 0.8 * remain)
        vcmd = max(0.1, vcmd)
        send_cmd(vcmd, 0.0)
        time.sleep(0.05)

    send_cmd(0.0, 0.0)
    time.sleep(0.15)

    # Финальная проверка
    tel = get_fresh_telemetry()
    if tel:
        pose, _, _, _ = tel
        x, y, th = pose
        actual = math.hypot(x - x0, y - y0)
        log_print(f"[MOVE_PRECISE] Фактически проехали {actual:.2f}м")
        return True

    return False

def move_backward(distance, v=0.25):
    """Движение назад (для выезда из кармана)"""
    flush_telemetry()
    tel = get_fresh_telemetry()

    if not tel:
        log_print(f"[MOVE_BACK] ОШИБКА: нет телеметрии")
        return False

    pose0, _, _, _ = tel
    x0, y0, th0 = pose0

    log_print(f"[MOVE_BACK] Назад {distance:.2f}м от ({x0:.2f},{y0:.2f})")

    adjusted = distance / SLIP_FACTOR
    duration = adjusted / v

    send_cmd(-v, 0.0)
    time.sleep(duration)
    send_cmd(0.0, 0.0)
    time.sleep(0.2)

    tel = get_fresh_telemetry()
    if tel:
        pose, _, _, _ = tel
        x, y, th = pose
        actual = math.hypot(x - x0, y - y0)
        log_print(f"[MOVE_BACK] Фактически проехали {actual:.2f}м")
        log_telemetry(tel, "  ")
        return True

    return False

def turn_angle(angle_rad, w=0.5):
    """
    Поворот на заданный угол
    angle_rad > 0 : влево (CCW)
    angle_rad < 0 : вправо (CW)
    """
    flush_telemetry()
    tel = get_fresh_telemetry()

    if not tel:
        log_print(f"[TURN] ОШИБКА: нет телеметрии")
        return False

    pose0, _, _, _ = tel
    th0 = pose0[2]

    angle_deg = math.degrees(angle_rad)
    direction = "влево" if angle_rad > 0 else "вправо"
    log_print(f"[TURN] {direction} {abs(angle_deg):.1f}° от {math.degrees(th0):.1f}°")

    duration = abs(angle_rad) / w

    send_cmd(0.0, w if angle_rad > 0 else -w)
    time.sleep(duration)
    send_cmd(0.0, 0.0)
    time.sleep(0.15)

    tel = get_fresh_telemetry()
    if tel:
        pose, _, _, _ = tel
        th1 = pose[2]
        actual = angle_wrap(th1 - th0)
        log_print(f"[TURN] Фактически повернули {math.degrees(actual):.1f}°, теперь th={math.degrees(th1):.1f}°")
        return True

    return False

# === Идея №3: LIDAR-навигация для туннеля и узких мест ===

def navigate_by_lidar(target_x, target_y, target_th_deg, max_time=30.0):
    """
    Навигация по LIDAR (игнорируем координаты!)
    Используется в туннеле и узких проходах

    Стратегия:
    1. Держаться центра коридора (L ≈ R)
    2. Двигаться вперёд малыми шагами
    3. Остановиться когда F < порог или время истекло
    """
    log_print(f"[LIDAR_NAV] РЕЖИМ LIDAR - игнорируем координаты!")
    log_print(f"[LIDAR_NAV] Цель: ({target_x:.2f},{target_y:.2f}) th={target_th_deg:.1f}°")
    log_print(f"[LIDAR_NAV] L=СПРАВА, R=СЛЕВА (названия перепутаны!)")

    t_start = time.time()
    step_count = 0

    while time.time() - t_start < max_time:
        tel = get_fresh_telemetry()
        if not tel:
            log_print(f"[LIDAR_NAV] TIMEOUT телеметрии")
            break

        pose, vel, gyro, lidar = tel
        x, y, th = pose
        F, R, L = extract_lidar_frl(lidar)

        log_print(f"[LIDAR_NAV] шаг {step_count}: pos=({x:.2f},{y:.2f}) F={F:.2f} R={R:.2f} L={L:.2f}")

        # Проверка выхода: широкое пространство или препятствие
        # ИСПРАВЛЕНИЕ: порог уменьшен 0.8 → 0.35м, чтобы не останавливаться слишком рано
        if F < 0.35:
            log_print(f"[LIDAR_NAV] Препятствие впереди F={F:.2f}м, СТОП")
            break

        if F > 5.0 and (L > 3.0 or R > 3.0):
            log_print(f"[LIDAR_NAV] Широкое пространство - вышли из узкого места!")
            break

        # Центрирование: L=СПРАВА, R=СЛЕВА
        # ИСПРАВЛЕНИЕ: центрирование делаем ПЕРЕД движением, но НЕ блокируем движение!
        if L < 7.5 and R < 7.5:  # Обе стены видны
            center_error = L - R  # + → ближе к правой стене

            if abs(center_error) > 0.3:  # Порог увеличен: только большие отклонения
                if center_error > 0:
                    # L > R → правая стена ближе → повернуть влево
                    log_print(f"[LIDAR_NAV] Коррекция: L={L:.2f} > R={R:.2f}, поворот влево")
                    turn_angle(TURN_TINY, w=0.3)
                else:
                    # R > L → левая стена ближе → повернуть вправо
                    log_print(f"[LIDAR_NAV] Коррекция: R={R:.2f} > L={L:.2f}, поворот вправо")
                    turn_angle(-TURN_TINY, w=0.3)
                # НЕ ДЕЛАЕМ continue! Продолжаем движение вперёд!

        # Движение вперёд
        if F > 2.0:
            move_distance = 0.3
        elif F > 1.5:
            move_distance = 0.2
        else:
            move_distance = 0.1

        log_print(f"[LIDAR_NAV] Движение вперёд {move_distance:.2f}м")
        move_forward_precise(move_distance, v=0.2)

        step_count += 1

        if step_count > 50:
            log_print(f"[LIDAR_NAV] Достигнут лимит шагов (50)")
            break

    log_print(f"[LIDAR_NAV] Завершено за {time.time()-t_start:.1f}с, шагов: {step_count}")
    return True

# === Навигация по координатам (одометрия) ===

def navigate_by_odometry(target_x, target_y, target_th_deg, max_steps=40):
    """
    Навигация по координатам (одометрия)

    Стратегия (ИДЕЯ №2: быстрое движение без задержек):
    1. Быстрое движение к цели (0.5м шаги без ожидания)
    2. Финальный подход с проверкой (0.1-0.2м шаги)
    """
    target_th = math.radians(target_th_deg)

    log_print(f"[ODOM_NAV] Цель: ({target_x:.2f},{target_y:.2f}) th={target_th_deg:.1f}°")

    stuck_counter = 0  # Счётчик застревания
    last_pos = None

    for step in range(max_steps):
        tel = get_fresh_telemetry()
        if not tel:
            log_print(f"[ODOM_NAV] TIMEOUT телеметрии")
            return False

        pose, vel, gyro, lidar = tel
        x, y, th = pose
        F, R, L = extract_lidar_frl(lidar)

        dx = target_x - x
        dy = target_y - y
        dist = math.hypot(dx, dy)

        log_print(f"[ODOM_NAV] шаг {step}: pos=({x:.2f},{y:.2f}) th={math.degrees(th):.1f}° → цель dist={dist:.2f}м")
        log_telemetry(tel, "  ")

        # ЗАЩИТА ОТ ЗАСТРЕВАНИЯ: если робот не движется и впереди препятствие
        if last_pos:
            moved = math.hypot(x - last_pos[0], y - last_pos[1])
            if moved < 0.05 and F < 0.4:
                stuck_counter += 1
                log_print(f"[ODOM_NAV] ЗАСТРЕВАНИЕ! Счётчик: {stuck_counter}/5, F={F:.2f}м, двигались {moved:.3f}м")
                if stuck_counter >= 5:
                    log_print(f"[ODOM_NAV] КРИТИЧНО: Застряли! Препятствие F={F:.2f}м, не можем продолжить")
                    log_print(f"[ODOM_NAV] Координаты врут или стена впереди. Пропускаем узел.")
                    return False
            else:
                stuck_counter = 0  # Сброс счётчика если двигаемся

        last_pos = (x, y)

        # Достигли цели
        if dist < 0.12:
            log_print(f"[ODOM_NAV] Достигнут узел! dist={dist:.2f}м")
            # Финальный доворот
            angle_error = angle_wrap(target_th - th)
            if abs(angle_error) > 0.1:
                log_print(f"[ODOM_NAV] Финальный доворот {math.degrees(angle_error):.1f}°")
                turn_angle(angle_error, w=0.4)
            return True

        # Поворот к цели
        target_angle = math.atan2(dy, dx)
        angle_error = angle_wrap(target_angle - th)

        if abs(angle_error) > 0.2:  # > 11.4°
            # Крупный поворот
            if abs(angle_error) > 0.5:
                turn_angle(TURN_LARGE if angle_error > 0 else -TURN_LARGE, w=0.5)
            elif abs(angle_error) > 0.3:
                turn_angle(TURN_MEDIUM if angle_error > 0 else -TURN_MEDIUM, w=0.5)
            else:
                turn_angle(TURN_SMALL if angle_error > 0 else -TURN_SMALL, w=0.4)
            continue

        # Движение вперёд (ИДЕЯ №2: адаптивная скорость)
        # ИСПРАВЛЕНИЕ: убрана ветка else - не делаем лишний шаг!
        if dist > 2.0:
            # БЫСТРО без проверок
            log_print(f"[ODOM_NAV] БЫСТРОЕ движение (dist={dist:.2f}м > 2.0м)")
            move_forward_fast(1.0, v=0.5)
        elif dist > 1.0:
            # Средний темп
            log_print(f"[ODOM_NAV] Среднее движение (dist={dist:.2f}м > 1.0м)")
            move_forward_fast(0.5, v=0.4)
        elif dist > 0.15:
            # Медленно с контролем (порог уменьшен: 0.15м вместо 0.3м)
            log_print(f"[ODOM_NAV] Медленное движение (dist={dist:.2f}м > 0.15м)")
            move_forward_precise(min(dist, 0.2), v=0.25)
        # Если dist < 0.15м - ничего не делаем, на следующей итерации сработает проверка dist < 0.12

    log_print(f"[ODOM_NAV] Достигнут лимит шагов ({max_steps})")
    return False

# === Главная функция навигации ===

def navigate_route():
    """Пройти весь маршрут"""
    global start_time, log_file

    start_time = time.time()
    log_file = open("fast_navigator.log", "w", encoding="utf-8")

    log_print("=" * 80)
    log_print("FAST NAVIGATOR - Вариант A")
    log_print("=" * 80)
    log_print(f"Маршрут: {len(ROUTE)} узлов")
    log_print(f"Лимит: {TOTAL_TIME_LIMIT} секунд")
    log_print("=" * 80)
    log_print("")

    for node_idx in range(len(ROUTE)):
        if elapsed_time() > TOTAL_TIME_LIMIT:
            log_print(f"[MAIN] TIMEOUT! Превышен лимит {TOTAL_TIME_LIMIT}с")
            break

        x, y, th_deg, desc, mode = ROUTE[node_idx]

        log_print("")
        log_print("=" * 80)
        log_print(f"УЗЕЛ [{node_idx}]: {desc}")
        log_print(f"Цель: ({x:.2f}, {y:.2f}, {th_deg:.1f}°)")
        log_print(f"Режим: {mode.upper()}")
        log_print(f"Время: {elapsed_time():.1f}с / {TOTAL_TIME_LIMIT}с")
        log_print("=" * 80)

        # Выбор режима навигации
        if node_idx == 0:
            # Стартовая позиция - ничего не делаем
            log_print("[MAIN] Стартовый узел, пропускаем")
            continue

        elif node_idx == NODE_REVERSE:
            # Выезд из кармана задом
            # Как в ручном проходе: 1.95м
            # 3 шага по 0.5м + 1 шаг 0.45м = 1.95м
            log_print("[MAIN] Выезд задним ходом (1.95м как в ручном проходе)")
            for i in range(3):
                log_print(f"[REVERSE] Шаг {i+1}/4 назад 0.5м")
                move_backward(0.5, v=0.25)
            log_print(f"[REVERSE] Шаг 4/4 назад 0.45м")
            move_backward(0.45, v=0.25)

        elif mode == "lidar":
            # LIDAR-навигация
            navigate_by_lidar(x, y, th_deg, max_time=40.0)

        else:
            # Одометрия
            navigate_by_odometry(x, y, th_deg, max_steps=40)

        # Проверка финальной позиции
        tel = get_fresh_telemetry()
        if tel:
            pose, _, _, _ = tel
            actual_x, actual_y, actual_th = pose
            error_dist = math.hypot(x - actual_x, y - actual_y)
            error_th = abs(angle_wrap(math.radians(th_deg) - actual_th))

            log_print(f"[MAIN] Узел [{node_idx}] завершён")
            log_print(f"[MAIN] Цель:   ({x:.2f}, {y:.2f}) th={th_deg:.1f}°")
            log_print(f"[MAIN] Факт:   ({actual_x:.2f}, {actual_y:.2f}) th={math.degrees(actual_th):.1f}°")
            log_print(f"[MAIN] Ошибка: dist={error_dist:.2f}м, th={math.degrees(error_th):.1f}°")

    log_print("")
    log_print("=" * 80)
    log_print(f"ЗАВЕРШЕНО! Время: {elapsed_time():.1f}с")
    log_print("=" * 80)

    send_cmd(0.0, 0.0)
    log_file.close()

# === Запуск ===

if __name__ == "__main__":
    try:
        navigate_route()
    except KeyboardInterrupt:
        log_print("\n[MAIN] Прервано пользователем")
        send_cmd(0.0, 0.0)
    except Exception as e:
        log_print(f"\n[MAIN] ОШИБКА: {e}")
        import traceback
        traceback.print_exc()
        send_cmd(0.0, 0.0)
    finally:
        if log_file:
            log_file.close()
