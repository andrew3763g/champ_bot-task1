#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Навигация роботом ПО ШАГАМ с проверкой координат после каждого шага

Логика как в step.py:
- Фиксированные шаги: 1.0м, 0.5м, 0.2м, 0.1м (с компенсацией пробуксовки)
- Фиксированные повороты: π/8, π/16, π/32
- После КАЖДОГО шага проверяем координаты
- Движемся к узлу пошагово, пока не достигнем
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
TOTAL_TIME_LIMIT = 200.0

# === Шаги и повороты ===
STEP_LARGE = 1.0    # Большой шаг для открытых пространств
STEP_MEDIUM = 0.5   # Средний шаг (основной)
STEP_SMALL = 0.2    # Малый шаг для точности
STEP_TINY = 0.1     # Очень малый шаг

TURN_LARGE = math.pi / 8    # 22.5° (π/8)
TURN_MEDIUM = math.pi / 16  # 11.25° (π/16)
TURN_SMALL = math.pi / 32   # 5.625° (π/32)

# === Маршрут ===
ROUTE = [
    (0.00,   0.00,   0.0,   "START - в кармане"),
    (-3.35,  0.00,   0.0,   "Выехали из кармана задним ходом"),
    (1.05,   5.00,   48.7,  "Перед утками, поворот на восток"),
    (0.80,   6.95,   97.4,  "После уток, целимся в коридор"),
    (-1.64,  9.20,   146.1, "Заезд в коридор под углом 45°"),
    (-2.43,  9.73,   48.7,  "Выровнялись в коридоре"),
    (-0.66,  11.75,  48.7,  "Центр коридора"),
    (1.23,   13.89,  48.7,  "Вход во вторую комнату"),
    (1.24,   14.00,  146.9, "Начало подруливания"),
    (-0.77,  15.31,  146.9, "Подруливание вдоль Y"),
    (-2.66,  15.60,  171.3, "Перед проходом"),
    (-2.46,  18.45,  86.1,  "К финишу"),
    (-2.46,  18.45,  86.1,  "ФИНИШ"),
]

NODE_REVERSE = 1
NODE_FINISH = len(ROUTE) - 1

# === Сеть ===
sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

if PROTO == "udp":
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_tel.bind((TEL_HOST, TEL_PORT))
else:
    sock_tel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_tel.bind((TEL_HOST, TEL_PORT))
    sock_tel.listen(1)
    print(f"[nav] Ожидание TCP {TEL_HOST}:{TEL_PORT}...")
    conn, _ = sock_tel.accept()
    sock_tel = conn
    print("[nav] Подключено")

def send_cmd(v, w):
    sock_cmd.sendto(struct.pack("<2f", float(v), float(w)), (CMD_HOST, CMD_PORT))

def _recv_all(s, n):
    buf = b""
    while len(buf) < n:
        chunk = s.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf

def recv_tel(timeout=0.5):
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

def clear_buffer():
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
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

# === Базовые примитивы ===

def step_forward(distance, v=0.3, timeout=5.0):
    """
    Один шаг вперёд на заданное расстояние с компенсацией пробуксовки
    Возвращает (success, actual_distance)
    """
    clear_buffer()

    tel = recv_tel(timeout=1.0)
    if not tel:
        return False, 0.0

    pose0, _, _, _ = tel
    x0, y0, th0 = pose0

    # Компенсируем пробуксовку
    adjusted = distance / SLIP_FACTOR

    t0 = time.time()
    while time.time() - t0 < timeout:
        tel = recv_tel(timeout=0.5)
        if not tel:
            break

        pose, _, _, rng = tel
        x, y, th = pose
        d = math.hypot(x - x0, y - y0)

        if d >= distance * 0.95:  # 95% от цели - достаточно
            break

        # Проверка препятствий
        if rng and rng[len(rng)//2] < 0.3:
            print(f"[WARN] Препятствие!")
            break

        remain = adjusted - d
        vcmd = min(v, 0.8 * remain)
        vcmd = max(0.15, vcmd)
        send_cmd(vcmd, 0.0)
        time.sleep(0.03)

    send_cmd(0.0, 0.0)
    time.sleep(0.15)

    # Финальная проверка
    tel = recv_tel(timeout=1.0)
    if tel:
        pose, _, _, _ = tel
        x, y, th = pose
        actual = math.hypot(x - x0, y - y0)
        return True, actual

    return False, 0.0

def step_backward(distance, v=0.25, timeout=5.0):
    """Один шаг назад"""
    clear_buffer()

    tel = recv_tel(timeout=1.0)
    if not tel:
        return False, 0.0

    pose0, _, _, _ = tel
    x0, y0, th0 = pose0

    adjusted = distance / SLIP_FACTOR

    t0 = time.time()
    while time.time() - t0 < timeout:
        tel = recv_tel(timeout=0.5)
        if not tel:
            break

        pose, _, _, _ = tel
        x, y, th = pose
        d = math.hypot(x - x0, y - y0)

        if d >= distance * 0.95:
            break

        remain = adjusted - d
        vcmd = min(v, 0.8 * remain)
        vcmd = max(0.12, vcmd)
        send_cmd(-vcmd, 0.0)
        time.sleep(0.03)

    send_cmd(0.0, 0.0)
    time.sleep(0.15)

    tel = recv_tel(timeout=1.0)
    if tel:
        pose, _, _, _ = tel
        x, y, th = pose
        actual = math.hypot(x - x0, y - y0)
        return True, actual

    return False, 0.0

def turn_angle(delta, w=0.7, timeout=4.0):
    """
    Повернуться на угол delta (в радианах)
    + влево (против часовой), - вправо (по часовой)
    Возвращает (success, actual_turn)
    """
    clear_buffer()

    tel = recv_tel(timeout=1.0)
    if not tel:
        return False, 0.0

    pose0, _, _, _ = tel
    _, _, th0 = pose0

    target = angle_wrap(th0 + delta)
    tol = 0.03  # ~1.7° допуск

    t0 = time.time()
    while time.time() - t0 < timeout:
        tel = recv_tel(timeout=0.5)
        if not tel:
            break

        pose, _, _, _ = tel
        _, _, th = pose
        th = angle_wrap(th)

        err = angle_wrap(target - th)
        if abs(err) < tol:
            break

        wcmd = max(0.3, min(w, 2.5 * abs(err))) * (1 if err > 0 else -1)
        send_cmd(0.0, wcmd)
        time.sleep(0.02)

    send_cmd(0.0, 0.0)
    time.sleep(0.15)

    # Финальная проверка
    tel = recv_tel(timeout=1.0)
    if tel:
        pose, _, _, _ = tel
        _, _, th = pose
        actual_turn = angle_wrap(th - th0)
        return True, actual_turn

    return False, 0.0

# === Навигация к узлу ===

def navigate_to_node(target_x, target_y, target_th, max_steps=50):
    """
    Навигация к узлу ПОШАГОВО

    Алгоритм:
    1. Проверяем текущие координаты
    2. Вычисляем расстояние и угол до цели
    3. Если нужен поворот - делаем поворот на π/16
    4. Делаем шаг вперёд (1м, 0.5м, 0.2м или 0.1м в зависимости от расстояния)
    5. Повторяем пока не достигнем цели
    """

    for step_num in range(max_steps):
        clear_buffer()
        tel = recv_tel(timeout=1.0)
        if not tel:
            print(f"[ERROR] Потеря телеметрии на шаге {step_num}")
            return False

        pose, _, _, _ = tel
        x, y, th = pose
        th = angle_wrap(th)

        # Расстояние и направление до цели
        dx = target_x - x
        dy = target_y - y
        dist = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = angle_wrap(target_angle - th)

        print(f"[STEP {step_num}] pos=({x:.2f},{y:.2f},{math.degrees(th):.1f}°) → цель ({target_x:.2f},{target_y:.2f}) dist={dist:.2f}м")

        # Достигли цели по координатам?
        if dist < 0.12:
            print(f"[OK] Достигли узла! Финальный доворот...")
            # Финальный доворот на целевой угол
            final_error = angle_wrap(target_th - th)
            if abs(final_error) > 0.1:  # >5.7°
                print(f"[TURN] Финальный доворот {math.degrees(final_error):+.1f}°")
                if abs(final_error) > TURN_MEDIUM:
                    turn_angle(TURN_MEDIUM if final_error > 0 else -TURN_MEDIUM)
                else:
                    turn_angle(TURN_SMALL if final_error > 0 else -TURN_SMALL)
            return True

        # Нужен поворот?
        if abs(angle_error) > 0.2:  # >11.5° - нужно повернуться
            # Выбираем величину поворота
            if abs(angle_error) > TURN_LARGE * 1.5:
                turn_step = TURN_LARGE
            elif abs(angle_error) > TURN_MEDIUM * 1.5:
                turn_step = TURN_MEDIUM
            else:
                turn_step = TURN_SMALL

            turn_dir = turn_step if angle_error > 0 else -turn_step
            print(f"[TURN] Ошибка {math.degrees(angle_error):+.1f}° → поворот {math.degrees(turn_dir):+.1f}°")

            success, actual = turn_angle(turn_dir)
            if not success:
                print(f"[WARN] Поворот не удался")
            continue

        # Выбираем размер шага в зависимости от расстояния
        if dist > 2.0:
            move_step = STEP_LARGE    # 1.0м
        elif dist > 1.0:
            move_step = STEP_MEDIUM   # 0.5м
        elif dist > 0.3:
            move_step = STEP_SMALL    # 0.2м
        else:
            move_step = STEP_TINY     # 0.1м

        # Не переезжаем цель
        move_step = min(move_step, dist + 0.05)

        print(f"[MOVE] Шаг вперёд {move_step:.2f}м")
        success, actual = step_forward(move_step, v=0.3)

        if not success:
            print(f"[WARN] Шаг не удался")
        else:
            print(f"[MOVE] Проехали {actual:.2f}м")

    print(f"[ERROR] Не удалось достичь узла за {max_steps} шагов")
    return False

# === Главная функция ===

def main():
    print("="*80)
    print("STEP NAVIGATOR - Пошаговая навигация")
    print("="*80)
    print(f"Маршрут: {len(ROUTE)} узлов")
    print(f"Лимит: {TOTAL_TIME_LIMIT:.0f} секунд")
    print("="*80)

    start_time = time.time()

    for i in range(len(ROUTE)):
        elapsed = time.time() - start_time
        if elapsed >= TOTAL_TIME_LIMIT:
            print(f"\n⏱️ ТАЙМАУТ {TOTAL_TIME_LIMIT:.0f}с!")
            return False

        remaining = TOTAL_TIME_LIMIT - elapsed
        print(f"\n[TIME] {elapsed:.1f}с / {TOTAL_TIME_LIMIT:.0f}с (осталось {remaining:.0f}с)")

        x, y, th_deg, desc = ROUTE[i]
        th_rad = math.radians(th_deg)

        print(f"\n{'='*80}")
        print(f"УЗЕЛ [{i}]: {desc}")
        print(f"Цель: ({x:.2f}, {y:.2f}, {th_deg:.1f}°)")
        print(f"{'='*80}")

        # Финиш
        if i == NODE_FINISH:
            print("[FINISH] Едем до потери телеметрии...")
            # Простое движение вперёд до потери связи
            consecutive_failures = 0
            while consecutive_failures < 3:
                tel = recv_tel(timeout=0.5)
                if tel is None:
                    consecutive_failures += 1
                else:
                    consecutive_failures = 0
                send_cmd(0.25, 0.0)
                time.sleep(0.05)
            send_cmd(0.0, 0.0)

            final_time = time.time() - start_time
            print(f"\n🎉 ФИНИШ! Время: {final_time:.1f}с")
            return True

        # Следующий узел
        if i + 1 >= len(ROUTE):
            break

        next_x, next_y, next_th_deg, next_desc = ROUTE[i + 1]
        next_th_rad = math.radians(next_th_deg)

        print(f"→ Следующий [{i+1}]: {next_desc}")

        # Движение задом (узел 0→1)
        if i == 0 and i + 1 == NODE_REVERSE:
            print("[REVERSE] Выезд задним ходом по шагам")
            # Едем назад 7 шагов по 0.5м
            for step in range(7):
                print(f"[BACK {step+1}/7] Шаг назад 0.5м")
                success, actual = step_backward(STEP_MEDIUM)
                if not success:
                    print("[ERROR] Не удалось выехать")
                    return False

                # Проверяем достигли ли цели
                tel = recv_tel()
                if tel:
                    pose, _, _, _ = tel
                    x_curr, y_curr, _ = pose
                    dist_to_target = math.hypot(next_x - x_curr, next_y - y_curr)
                    if dist_to_target < 0.15:
                        print(f"[OK] Выехали из кармана!")
                        break
            continue

        # Обычное движение
        success = navigate_to_node(next_x, next_y, next_th_rad)
        if not success:
            print(f"[ERROR] Не удалось достичь узла {i+1}")
            return False

        print(f"[OK] Узел {i+1} достигнут")
        time.sleep(0.2)

    return True

if __name__ == "__main__":
    try:
        success = main()
        send_cmd(0.0, 0.0)
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n[Ctrl+C]")
        send_cmd(0.0, 0.0)
        sys.exit(0)
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        send_cmd(0.0, 0.0)
        sys.exit(1)
