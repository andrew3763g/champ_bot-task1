#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Навигация робота по графу узлов (на основе step.log)

Особенности:
- Учитывает пробуксовку (робот едет ~89% от команды)
- Обязательный заезд в коридор под углом 45°
- Остановка при потере телеметрии на финише
- Подруливание для обхода препятствий
"""
import math
import os
import socket
import struct
import time
import sys

# === Настройки сети ===
CMD_HOST = os.getenv("CMD_HOST", "127.0.0.1")
CMD_PORT = int(os.getenv("CMD_PORT", "5555"))
TEL_HOST = os.getenv("TEL_HOST", "0.0.0.0")
TEL_PORT = int(os.getenv("TEL_PORT", "5600"))
PROTO = os.getenv("PROTO", "tcp")

# === Параметры робота ===
MAX_RANGE_CLIP = 8.0
SLIP_FACTOR = 0.89  # Робот едет 44-45см вместо 50см
SAFE_DISTANCE = 0.35  # Минимальное безопасное расстояние до препятствий
TOTAL_TIME_LIMIT = 200.0  # Общий лимит времени на прохождение маршрута (сек)

# === Маршрут: массив координат (x, y, th_градусы) ===
# Формат: (x, y, угол_в_градусах, описание)
# th: 0°=СЕВЕР, 90°=ВОСТОК, 180°=ЮГ, -90°=ЗАПАД
ROUTE = [
    # (x,     y,      th,    описание)
    (0.00,   0.00,   0.0,   "START - в кармане"),
    (-3.35,  0.00,   0.0,   "Выехали из кармана задним ходом"),  # REVERSE!
    (1.05,   5.00,   48.7,  "Перед утками, поворот на восток"),
    (0.80,   6.95,   97.4,  "После уток, целимся в коридор"),
    (-1.64,  9.20,   146.1, "Заезд в коридор под углом 45°"),    # КРИТИЧНО!
    (-2.43,  9.73,   48.7,  "Выровнялись в коридоре"),
    (-0.66,  11.75,  48.7,  "Центр коридора (ближе к южной стене)"),
    (1.23,   13.89,  48.7,  "Вход во вторую комнату"),
    (1.24,   14.00,  146.9, "Начало подруливания к проходу"),
    (-0.77,  15.31,  146.9, "Подруливание вдоль оси Y"),
    (-2.66,  15.60,  171.3, "Перед проходом, поворот на восток"),
    (-2.46,  18.45,  86.1,  "Прошли проход, едем к финишу"),
    (-2.46,  18.45,  86.1,  "ФИНИШ - едем до потери телеметрии"), # FINISH!
]

# Специальные узлы
NODE_REVERSE = 1  # Узел 0→1: выезд задним ходом
NODE_FINISH = len(ROUTE) - 1  # Последний узел: финиш

def get_node_info(i):
    """Получить информацию об узле из массива"""
    if i < 0 or i >= len(ROUTE):
        return None

    x, y, th_deg, description = ROUTE[i]
    th_rad = math.radians(th_deg)

    return {
        'id': i,
        'x': x,
        'y': y,
        'th': th_rad,
        'th_deg': th_deg,
        'description': description,
        'is_reverse': (i == NODE_REVERSE),
        'is_finish': (i == NODE_FINISH)
    }

# === Сетевые функции ===
sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

if PROTO == "udp":
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_tel.bind((TEL_HOST, TEL_PORT))
else:
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_tel.bind((TEL_HOST, TEL_PORT))
    sock_tel.listen(1)
    print(f"[navigator] Ожидание TCP подключения на {TEL_HOST}:{TEL_PORT}...")
    conn, _ = sock_tel.accept()
    sock_tel = conn
    print("[navigator] Подключено к телеметрии")

def send_cmd(v, w):
    """Отправить команду управления"""
    sock_cmd.sendto(struct.pack("<2f", float(v), float(w)), (CMD_HOST, CMD_PORT))

def _recv_all(s, n):
    """Получить n байт из TCP сокета"""
    buf = b""
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

        # Обработка дальности: 0.0 → MAX_RANGE_CLIP
        rng = [MAX_RANGE_CLIP if r <= 0.0 else min(MAX_RANGE_CLIP, r) for r in rng]

        return (x, y, th), (vx, vy, vth), (wx, wy, wz), rng

    except socket.timeout:
        return None
    finally:
        sock_tel.settimeout(old_timeout)

def clear_telemetry_buffer():
    """Очистить буфер телеметрии"""
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

def angle_wrap(a):
    """Нормализовать угол в диапазон [-π, π]"""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

# === Навигационные функции ===

def move_to_position(target_x, target_y, target_th, v=0.3, angle_tol=0.1, dist_tol=0.15, timeout=20.0, max_attempts=3):
    """
    Движение к целевой позиции с проверкой реального достижения координат

    Args:
        target_x, target_y: целевые координаты
        target_th: целевой угол (радианы)
        v: скорость движения (м/с)
        angle_tol: допуск по углу (радианы)
        dist_tol: допуск по расстоянию (м)
        timeout: таймаут одной попытки (сек)
        max_attempts: максимальное количество попыток доехать

    Returns:
        True если достигли, False если не удалось
    """

    for attempt in range(max_attempts):
        if attempt > 0:
            print(f"[RETRY] Попытка {attempt + 1}/{max_attempts}")

        clear_telemetry_buffer()

        tel = recv_tel(timeout=1.0)
        if not tel:
            print("[ERROR] Не удалось получить начальную телеметрию")
            continue

        pose, _, _, _ = tel
        x0, y0, th0 = pose

        # Проверяем, может уже на месте?
        dx = target_x - x0
        dy = target_y - y0
        distance = math.hypot(dx, dy)
        angle_error = angle_wrap(target_th - th0)

        print(f"[CHECK] Текущая: ({x0:.2f}, {y0:.2f}, {math.degrees(th0):.1f}°)")
        print(f"[CHECK] Целевая:  ({target_x:.2f}, {target_y:.2f}, {math.degrees(target_th):.1f}°)")
        print(f"[CHECK] Ошибка: расстояние={distance:.2f}м, угол={math.degrees(angle_error):+.1f}°")

        # Если уже достигли - выходим
        if distance < dist_tol and abs(angle_error) < angle_tol:
            print(f"[OK] Узел уже достигнут!")
            return True

        target_angle = math.atan2(dy, dx)

        # Шаг 1: Поворот к цели
        angle_to_turn = angle_wrap(target_angle - th0)
        if abs(angle_to_turn) > angle_tol:
            print(f"[TURN] Поворот на {math.degrees(angle_to_turn):+.1f}° (направление к цели)")
            turn_to_angle(th0 + angle_to_turn, timeout=timeout/4)

        # Шаг 2: Движение к цели с контролем координат
        if distance > dist_tol:
            # Компенсация пробуксовки
            adjusted_distance = distance / SLIP_FACTOR
            print(f"[MOVE] Едем {distance:.2f}м (команда {adjusted_distance:.2f}м с учётом пробуксовки)")

            # Движемся с постоянной проверкой координат
            move_distance_to_target(target_x, target_y, v=v, timeout=timeout/2)

        # Шаг 3: Финальный доворот на целевой угол
        tel = recv_tel(timeout=1.0)
        if tel:
            pose, _, _, _ = tel
            x_curr, y_curr, th_curr = pose

            final_angle_error = angle_wrap(target_th - th_curr)
            final_distance = math.hypot(target_x - x_curr, target_y - y_curr)

            print(f"[CHECK] После движения: ({x_curr:.2f}, {y_curr:.2f})")
            print(f"[CHECK] Отклонение: {final_distance:.2f}м")

            if abs(final_angle_error) > angle_tol:
                print(f"[TURN] Финальный доворот на {math.degrees(final_angle_error):+.1f}°")
                turn_to_angle(target_th, timeout=timeout/4)

            # Проверка финального достижения
            if final_distance < dist_tol:
                print(f"[OK] Узел достигнут с точностью {final_distance:.2f}м")
                return True
            elif final_distance < dist_tol * 2:
                print(f"[WARN] Не совсем точно ({final_distance:.2f}м), но приемлемо")
                return True
            else:
                print(f"[WARN] Не доехали {final_distance:.2f}м, повторяем")

        time.sleep(0.3)

    print(f"[ERROR] Не удалось достичь узла за {max_attempts} попыток")
    return False

def move_reverse_to_target(target_x, target_y, target_th, v=0.25, timeout=15.0):
    """
    Движение ЗАДНИМ ХОДОМ к целевым координатам импульсами
    Используется для выезда из кармана (узел 0→1)

    Args:
        target_x, target_y: целевые координаты
        target_th: целевой угол (радианы, не меняем при движении назад)
        v: скорость (м/с)
        timeout: таймаут (сек)

    Returns:
        True если достигли цели
    """
    clear_telemetry_buffer()

    print(f"[REVERSE] Цель: ({target_x:.2f}, {target_y:.2f})")

    t0 = time.time()
    pulse_duration = 0.3  # Длительность одного импульса (сек)

    while time.time() - t0 < timeout:
        tel = recv_tel(timeout=1.0)
        if not tel:
            print("[WARN] Потеряна телеметрия")
            break

        pose, _, _, _ = tel
        x, y, th = pose

        # Проверяем расстояние до цели
        dx = target_x - x
        dy = target_y - y
        dist_to_target = math.hypot(dx, dy)

        print(f"[REVERSE] Текущая: ({x:.2f}, {y:.2f}), до цели: {dist_to_target:.2f}м")

        if dist_to_target < 0.15:  # Достигли цели
            print(f"[OK] Достигли целевых координат!")
            send_cmd(0.0, 0.0)
            return True

        # Импульс назад
        print(f"[REVERSE] Импульс назад {pulse_duration}с")
        send_cmd(-v, 0.0)  # Отрицательная скорость = назад
        time.sleep(pulse_duration)
        send_cmd(0.0, 0.0)
        time.sleep(0.2)  # Пауза между импульсами

    send_cmd(0.0, 0.0)
    print(f"[WARN] Таймаут выезда из кармана")
    return False

def move_distance_to_target(target_x, target_y, v=0.3, timeout=10.0):
    """
    Движение к целевым координатам ИМПУЛЬСАМИ с проверкой позиции
    Останавливается когда достигли цели или таймаут

    Стратегия:
    1. Проверяем текущие координаты
    2. Вычисляем расстояние до цели
    3. Даём импульс движения 0.3-0.5 секунды
    4. Останавливаемся и снова проверяем координаты
    5. Повторяем пока не достигнем цели
    """
    t0 = time.time()
    pulse_duration = 0.4  # Длительность импульса (сек)
    last_dist = None

    while time.time() - t0 < timeout:
        # Получаем текущую позицию
        tel = recv_tel(timeout=1.0)
        if not tel:
            print("[WARN] Потеряна телеметрия")
            break

        pose, _, _, rng = tel
        x, y, th = pose

        # Проверяем расстояние до цели
        dx = target_x - x
        dy = target_y - y
        dist_to_target = math.hypot(dx, dy)

        # Показываем прогресс
        if last_dist is None or abs(dist_to_target - last_dist) > 0.1:
            print(f"[MOVE] pos=({x:.2f},{y:.2f}) → цель=({target_x:.2f},{target_y:.2f}), осталось {dist_to_target:.2f}м")
            last_dist = dist_to_target

        if dist_to_target < 0.1:  # Достигли цели
            print(f"[OK] Достигли целевых координат")
            break

        # Вычисляем направление к цели
        target_angle = math.atan2(dy, dx)
        angle_error = angle_wrap(target_angle - th)

        # Проверка препятствий
        if rng:
            front = rng[len(rng)//2]
            if front < SAFE_DISTANCE:
                print(f"[WARN] Препятствие на {front:.2f}м - остановка")
                break

        # Если сильно отклонились от курса (>20°) - сначала доворачиваемся
        if abs(angle_error) > 0.35:  # >20°
            print(f"[CORRECT] Большое отклонение {math.degrees(angle_error):+.1f}° - доворот")
            wcmd = 0.6 if angle_error > 0 else -0.6
            send_cmd(0.0, wcmd)
            time.sleep(0.3)
            send_cmd(0.0, 0.0)
            time.sleep(0.2)
            continue

        # Даём импульс движения с лёгким подруливанием
        wcmd = 0.3 * angle_error  # Подруливание

        # Скорость зависит от расстояния
        if dist_to_target > 1.0:
            vcmd = v  # Полная скорость
            pulse = pulse_duration
        elif dist_to_target > 0.5:
            vcmd = v * 0.7  # 70% скорости
            pulse = pulse_duration * 0.8
        else:
            vcmd = v * 0.5  # 50% скорости для точности
            pulse = pulse_duration * 0.5

        vcmd = max(0.15, vcmd)  # Минимум 0.15 м/с

        print(f"[PULSE] Импульс {vcmd:.2f}м/с на {pulse:.2f}с")
        send_cmd(vcmd, wcmd)
        time.sleep(pulse)
        send_cmd(0.0, 0.0)
        time.sleep(0.15)  # Короткая пауза между импульсами

    send_cmd(0.0, 0.0)
    time.sleep(0.2)

def turn_to_angle(target_th, w=0.8, tol=0.05, timeout=6.0):
    """Повернуться к целевому углу"""
    clear_telemetry_buffer()

    tel = recv_tel(timeout=1.0)
    if not tel:
        return False

    pose, _, _, _ = tel
    _, _, th0 = pose

    t0 = time.time()
    while True:
        tel = recv_tel(timeout=0.5)
        if not tel:
            print("[WARN] Потеряна телеметрия при повороте")
            break

        pose, _, _, _ = tel
        _, _, th = pose
        th = angle_wrap(th)

        err = angle_wrap(target_th - th)
        if abs(err) < tol:
            break

        if time.time() - t0 > timeout:
            print(f"[WARN] Таймаут поворота, ошибка {math.degrees(err):.1f}°")
            break

        wcmd = max(0.3, min(w, 2.0 * abs(err))) * (1 if err > 0 else -1)
        send_cmd(0.0, wcmd)
        time.sleep(0.02)

    send_cmd(0.0, 0.0)
    time.sleep(0.2)
    return True

def move_distance(dist, v=0.3, tol=0.05, timeout=10.0):
    """Проехать заданное расстояние"""
    clear_telemetry_buffer()

    tel = recv_tel(timeout=1.0)
    if not tel:
        return False

    pose, _, _, _ = tel
    x0, y0, _ = pose

    t0 = time.time()
    while True:
        tel = recv_tel(timeout=0.5)
        if not tel:
            print("[WARN] Потеряна телеметрия при движении")
            break

        pose, _, _, rng = tel
        x, y, _ = pose

        d = math.hypot(x - x0, y - y0)
        if d >= abs(dist) - tol:
            break

        if time.time() - t0 > timeout:
            print(f"[WARN] Таймаут движения, проехали {d:.2f}м из {abs(dist):.2f}м")
            break

        # Проверка препятствий
        if rng:
            front = rng[len(rng)//2]
            if dist > 0 and front < SAFE_DISTANCE:
                print(f"[WARN] Препятствие на {front:.2f}м - остановка")
                break

        remain = abs(dist) - d
        vcmd = max(0.12, min(v, 0.6 * remain))
        vcmd = vcmd if dist > 0 else -vcmd
        send_cmd(vcmd, 0.0)
        time.sleep(0.02)

    send_cmd(0.0, 0.0)
    time.sleep(0.2)
    return True

def move_to_finish(start_time):
    """
    Движение к финишу (узел 11 → 12)
    Едем на север до потери телеметрии

    Args:
        start_time: время старта навигации (для финального отчёта)
    """
    print("[FINISH] Движение к финишу до потери телеметрии...")

    tel = recv_tel(timeout=1.0)
    if not tel:
        print("[ERROR] Нет телеметрии перед финишем")
        return False

    pose, _, _, _ = tel
    x0, y0, th0 = pose
    print(f"[FINISH] Стартуем с ({x0:.2f}, {y0:.2f}, {math.degrees(th0):.1f}°)")

    # Едем на север (th ≈ 86°)
    telemetry_lost = False
    consecutive_failures = 0
    t0 = time.time()

    while time.time() - t0 < 10.0:  # Максимум 10 секунд
        # Проверка глобального таймаута
        elapsed = time.time() - start_time
        if elapsed >= TOTAL_TIME_LIMIT:
            print(f"[TIMEOUT] Превышен лимит времени на финише!")
            return False

        tel = recv_tel(timeout=0.5)

        if tel is None:
            consecutive_failures += 1
            if consecutive_failures >= 3:
                telemetry_lost = True
                break
        else:
            consecutive_failures = 0
            pose, _, _, _ = tel
            x, y, th = pose
            print(f"[FINISH] pos=({x:.2f}, {y:.2f}), проехали {math.hypot(x-x0, y-y0):.2f}м")

        send_cmd(0.25, 0.0)  # Медленно вперёд
        time.sleep(0.05)

    send_cmd(0.0, 0.0)

    if telemetry_lost:
        elapsed_total = time.time() - start_time
        print(f"[SUCCESS] Телеметрия потеряна - мы на финише!")
        print(f"[TIME] ⏱️  Общее время прохождения: {elapsed_total:.1f} секунд (лимит: {TOTAL_TIME_LIMIT:.0f}с)")
        if elapsed_total < TOTAL_TIME_LIMIT:
            print(f"[TIME] ✅ Уложились в лимит! Запас: {TOTAL_TIME_LIMIT - elapsed_total:.1f}с")
        return True
    else:
        print("[WARN] Таймаут без потери телеметрии")
        return False

# === Основная логика навигации ===

def navigate_graph():
    """Навигация по массиву узлов"""
    print("=" * 80)
    print("GRAPH NAVIGATOR - Навигация по маршруту")
    print("=" * 80)
    print(f"Всего узлов: {len(ROUTE)}")
    print("Маршрут: южный путь (широкие проходы)")
    print(f"Лимит времени: {TOTAL_TIME_LIMIT:.0f} секунд")
    print("=" * 80)

    # Запускаем глобальный таймер
    start_time = time.time()

    # Проходим по всем узлам маршрута
    for i in range(len(ROUTE)):
        # Проверка глобального таймаута
        elapsed = time.time() - start_time
        remaining = TOTAL_TIME_LIMIT - elapsed

        if elapsed >= TOTAL_TIME_LIMIT:
            print(f"\n{'='*80}")
            print(f"⏱️  ТАЙМАУТ! Превышен лимит {TOTAL_TIME_LIMIT:.0f} секунд")
            print(f"{'='*80}")
            return False

        if remaining < 30:
            print(f"\n⚠️  ВНИМАНИЕ: Осталось {remaining:.0f} секунд!")

        print(f"\n[TIME] Прошло {elapsed:.1f}с / {TOTAL_TIME_LIMIT:.0f}с (осталось {remaining:.0f}с)")

        # Получаем информацию о текущем и следующем узле
        current = get_node_info(i)
        next_node = get_node_info(i + 1) if i + 1 < len(ROUTE) else None

        print(f"\n{'='*80}")
        print(f"УЗЕЛ [{i}]: {current['description']}")
        print(f"Позиция: ({current['x']:.2f}, {current['y']:.2f}, {current['th_deg']:.1f}°)")
        print(f"{'='*80}")

        # Если это последний узел - едем до финиша
        if current['is_finish']:
            print("[FINISH] Последний узел - движение до потери телеметрии")
            success = move_to_finish(start_time)
            if success:
                print("\n" + "="*80)
                print("🎉 ФИНИШ ДОСТИГНУТ!")
                print("="*80)
                return True
            else:
                print("\n[ERROR] Не удалось достичь финиша")
                return False

        # Если нет следующего узла - выходим
        if next_node is None:
            print("[INFO] Конечный узел маршрута")
            break

        print(f"→ Следующий: [{next_node['id']}] {next_node['description']}")
        print(f"  Цель: ({next_node['x']:.2f}, {next_node['y']:.2f}, {next_node['th_deg']:.1f}°)")

        # Специальная обработка для движения задним ходом (узел 0→1)
        if i == 0 and next_node['is_reverse']:
            print("[REVERSE] Выезд из кармана ЗАДНИМ ХОДОМ импульсами")
            success = move_reverse_to_target(
                next_node['x'],
                next_node['y'],
                next_node['th'],
                v=0.25,
                timeout=15.0
            )

            if not success:
                print(f"[ERROR] Не удалось выехать из кармана")
                return False

            print(f"[OK] Узел {next_node['id']} достигнут")
            continue

        # Обычное движение вперёд к следующему узлу
        success = move_to_position(
            next_node['x'],
            next_node['y'],
            next_node['th'],
            v=0.3
        )

        if not success:
            print(f"[ERROR] Не удалось достичь узла {next_node['id']}")
            return False

        print(f"[OK] Узел {next_node['id']} достигнут")
        time.sleep(0.3)  # Короткая пауза между узлами

    return True

# === Точка входа ===

if __name__ == "__main__":
    try:
        success = navigate_graph()
        send_cmd(0.0, 0.0)

        if success:
            print("\n✅ Навигация завершена успешно!")
            sys.exit(0)
        else:
            print("\n❌ Навигация не завершена")
            sys.exit(1)

    except KeyboardInterrupt:
        print("\n\n[INFO] Прервано пользователем (Ctrl+C)")
        send_cmd(0.0, 0.0)
        sys.exit(0)

    except Exception as e:
        print(f"\n[ERROR] Исключение: {e}")
        import traceback
        traceback.print_exc()
        send_cmd(0.0, 0.0)
        sys.exit(1)
