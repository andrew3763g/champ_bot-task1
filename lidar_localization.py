#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LIDAR-based Localization
========================

Решение обратной задачи: по показаниям LIDAR и примерным координатам
определить фактические координаты и угол робота.

Использует опорные точки (landmarks) из LIDAR_ANALYSIS.md:
- Узел 5: Перед воротами туннеля (L≈R≈2.8, F≈2.6)
- Узел 7: Вход в туннель (F<0.5, L≈R<0.5)
- Узел 2: Выезд из кармана (L≈1.1, R≈5.2)
"""

import math
import sys
import io

# Fix Windows console encoding
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

# === ОПОРНЫЕ ТОЧКИ (landmarks) из ручного прохода ===
LANDMARKS = {
    "START": {
        "coords": (0.00, 0.00, 0.0),  # x, y, th (градусы)
        "lidar": (3.63, 0.79, 0.80),  # F, L, R
        "confidence": 0.95,
        "description": "Карман (стартовая позиция)"
    },
    "OUT_OF_POCKET": {
        "coords": (-1.94, 0.00, 85.3),
        "lidar": (5.96, 1.09, 5.25),
        "confidence": 0.85,
        "description": "Выезд из кармана (th≈85°)"
    },
    "TUNNEL_GATE": {
        "coords": (1.22, 6.67, 91.0),
        "lidar": (2.57, 2.78, 2.79),
        "confidence": 0.95,
        "description": "Перед воротами туннеля (симметрия!)"
    },
    "TUNNEL_ENTRANCE": {
        "coords": (-1.61, 8.99, 87.0),
        "lidar": (0.40, 0.45, 0.43),
        "confidence": 0.98,
        "description": "Вход в туннель (самая узкая точка!)"
    },
    "ROOM2_CORRIDOR": {
        "coords": (-0.95, 16.05, 83.9),
        "lidar": (1.19, 1.29, 1.29),
        "confidence": 0.90,
        "description": "Коридор комнаты 2 (после туннеля)"
    },
    "ROOM2_CORNER": {
        "coords": (-5.22, 16.37, 175.7),
        "lidar": (0.63, 0.67, 0.68),
        "confidence": 0.90,
        "description": "Угол комнаты 2"
    }
}


def normalize_angle(angle_deg):
    """Нормализация угла в диапазон [-180, 180]"""
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg < -180:
        angle_deg += 360
    return angle_deg


def detect_landmark(F, L, R, verbose=False):
    """
    Детекция опорной точки по показаниям LIDAR.

    Возвращает: (landmark_name, confidence) или (None, 0)
    """

    # 1. Вход в туннель: F<0.5 и L≈R<0.5 (самая узкая точка!)
    if F < 0.5 and abs(L - R) < 0.1 and L < 0.5 and R < 0.5:
        if verbose:
            print(f"[LANDMARK] Вход в туннель: F={F:.2f} L={L:.2f} R={R:.2f}")
        return "TUNNEL_ENTRANCE", 0.98

    # 2. Перед воротами туннеля: L≈R≈2.8, F≈2.6 (симметрия!)
    if 2.4 < F < 2.8 and abs(L - R) < 0.2 and 2.5 < L < 3.0 and 2.5 < R < 3.0:
        if verbose:
            print(f"[LANDMARK] Перед воротами: F={F:.2f} L={L:.2f} R={R:.2f}")
        return "TUNNEL_GATE", 0.95

    # 3. Выезд из кармана: L≈1.1, R≈5.2, F>5.5 (уникальная асимметрия!)
    if 1.0 < L < 1.3 and 4.8 < R < 5.8 and F > 5.0:
        if verbose:
            print(f"[LANDMARK] Выезд из кармана: F={F:.2f} L={L:.2f} R={R:.2f}")
        return "OUT_OF_POCKET", 0.85

    # 4. Старт (карман): F≈3.6, L≈R≈0.8
    if 3.4 < F < 4.0 and abs(L - R) < 0.15 and 0.7 < L < 0.9 and 0.7 < R < 0.9:
        if verbose:
            print(f"[LANDMARK] Карман (START): F={F:.2f} L={L:.2f} R={R:.2f}")
        return "START", 0.95

    # 5. Коридор комнаты 2: F≈1.2, L≈R≈1.3
    if 1.0 < F < 1.4 and abs(L - R) < 0.2 and 1.1 < L < 1.5 and 1.1 < R < 1.5:
        if verbose:
            print(f"[LANDMARK] Коридор комнаты 2: F={F:.2f} L={L:.2f} R={R:.2f}")
        return "ROOM2_CORRIDOR", 0.80  # Снижен confidence (может путаться с другими коридорами)

    # 6. ЮГО-ЗАПАДНЫЙ УГОЛ комнаты 2: F≈0.63, L≈R≈0.67 (КРИТИЧЕСКАЯ ОПОРНАЯ ТОЧКА!)
    # Последняя надёжная точка перед финалом (дальше кресло с L=8.00)
    if 0.55 < F < 0.75 and abs(L - R) < 0.1 and 0.6 < L < 0.75 and 0.6 < R < 0.75:
        # Но нужно исключить вход в туннель (F=0.4, L=R=0.43)
        if F > 0.58:  # Вход в туннель с F<0.5
            if verbose:
                print(f"[LANDMARK] ЮГО-ЗАПАДНЫЙ УГОЛ комнаты 2: F={F:.2f} L={L:.2f} R={R:.2f}")
            return "ROOM2_CORNER", 0.95  # Повышена confidence!

    # Опорная точка не найдена
    if verbose:
        print(f"[LANDMARK] Не найдено: F={F:.2f} L={L:.2f} R={R:.2f}")
    return None, 0.0


def compute_correction(landmark_name, F, L, R, x_odom, y_odom, th_odom):
    """
    Вычисление поправок к координатам на основе опорной точки.

    Аргументы:
        landmark_name: название опорной точки
        F, L, R: показания LIDAR
        x_odom, y_odom, th_odom: показания одометрии (примерные)

    Возвращает:
        (dx, dy, dth, confidence) — поправки к координатам и уверенность
    """

    if landmark_name not in LANDMARKS:
        return 0.0, 0.0, 0.0, 0.0

    landmark = LANDMARKS[landmark_name]
    x_true, y_true, th_true = landmark["coords"]
    F_ref, L_ref, R_ref = landmark["lidar"]
    confidence = landmark["confidence"]

    # Базовые поправки: разница между истинными и измеренными координатами
    dx = x_true - x_odom
    dy = y_true - y_odom
    dth = normalize_angle(th_true - th_odom)

    # Проверка согласованности LIDAR-данных
    # Если текущие показания LIDAR сильно отличаются от референсных,
    # снижаем уверенность
    F_error = abs(F - F_ref)
    L_error = abs(L - L_ref)
    R_error = abs(R - R_ref)

    max_error = max(F_error, L_error, R_error)

    if max_error > 0.5:
        # Показания LIDAR сильно отличаются → снижаем confidence
        confidence *= 0.5
    elif max_error > 0.3:
        confidence *= 0.7
    elif max_error > 0.15:
        confidence *= 0.85

    return dx, dy, dth, confidence


def apply_correction(x_odom, y_odom, th_odom, dx, dy, dth, confidence):
    """
    Применение поправок к координатам с учётом уверенности.

    При confidence=1.0 — полное доверие опорной точке.
    При confidence=0.0 — опорная точка игнорируется.
    """

    x_corrected = x_odom + dx * confidence
    y_corrected = y_odom + dy * confidence
    th_corrected = normalize_angle(th_odom + dth * confidence)

    return x_corrected, y_corrected, th_corrected


# === КЛАСС ДЛЯ ОТСЛЕЖИВАНИЯ КОРРЕКЦИЙ ===

class LidarLocalizer:
    """
    Класс для отслеживания и применения коррекций координат на основе LIDAR.
    """

    def __init__(self):
        self.last_correction = None  # (dx, dy, dth, step_count)
        self.correction_decay = 0.95  # Коэффициент затухания коррекции
        self.step_count = 0
        self.last_landmark = None
        self.correction_active = False

        # История коррекций
        self.corrections_history = []

    def update(self, x_odom, y_odom, th_odom, F, L, R, verbose=False):
        """
        Обновление позиции с учётом LIDAR-локализации.

        Возвращает:
            (x_corrected, y_corrected, th_corrected, landmark_name, confidence)
        """

        self.step_count += 1

        # Детекция опорной точки
        landmark_name, confidence = detect_landmark(F, L, R, verbose=verbose)

        if landmark_name:
            # Найдена опорная точка!
            dx, dy, dth, conf = compute_correction(
                landmark_name, F, L, R, x_odom, y_odom, th_odom
            )

            # Сохраняем новую коррекцию
            self.last_correction = (dx, dy, dth, self.step_count)
            self.last_landmark = landmark_name
            self.correction_active = True

            # Логируем
            self.corrections_history.append({
                "step": self.step_count,
                "landmark": landmark_name,
                "odom": (x_odom, y_odom, th_odom),
                "correction": (dx, dy, dth),
                "confidence": conf
            })

            if verbose:
                print(f"[LOCALIZATION] Найдена опорная точка: {landmark_name}")
                print(f"[LOCALIZATION] Одометрия: ({x_odom:.2f}, {y_odom:.2f}, {th_odom:.1f}°)")
                print(f"[LOCALIZATION] Коррекция: dx={dx:.2f} dy={dy:.2f} dth={dth:.1f}°")
                print(f"[LOCALIZATION] Confidence: {conf:.2f}")

            # Применяем коррекцию с учётом confidence
            x_corr, y_corr, th_corr = apply_correction(
                x_odom, y_odom, th_odom, dx, dy, dth, conf
            )

            return x_corr, y_corr, th_corr, landmark_name, conf

        # Опорная точка не найдена
        # Используем последнюю коррекцию с затуханием
        if self.correction_active and self.last_correction:
            dx, dy, dth, corr_step = self.last_correction

            # Количество шагов с последней коррекции
            steps_since = self.step_count - corr_step

            # Затухание коррекции
            decay_factor = self.correction_decay ** steps_since

            if decay_factor < 0.1:
                # Коррекция слишком старая → отключаем
                self.correction_active = False
                return x_odom, y_odom, th_odom, None, 0.0

            # Применяем коррекцию с затуханием
            x_corr, y_corr, th_corr = apply_correction(
                x_odom, y_odom, th_odom, dx, dy, dth, decay_factor
            )

            if verbose and steps_since % 10 == 0:
                print(f"[LOCALIZATION] Используется старая коррекция ({self.last_landmark}), "
                      f"затухание: {decay_factor:.2f}")

            return x_corr, y_corr, th_corr, None, decay_factor

        # Коррекции нет → возвращаем одометрию как есть
        return x_odom, y_odom, th_odom, None, 0.0

    def get_correction_info(self):
        """Информация о текущей коррекции для отладки"""
        if not self.correction_active:
            return "Коррекция неактивна"

        if not self.last_correction:
            return "Коррекция не применялась"

        dx, dy, dth, corr_step = self.last_correction
        steps_since = self.step_count - corr_step
        decay = self.correction_decay ** steps_since

        return (f"Последняя коррекция: {self.last_landmark}, "
                f"шагов назад: {steps_since}, "
                f"затухание: {decay:.2f}, "
                f"поправки: dx={dx:.2f} dy={dy:.2f} dth={dth:.1f}°")


# === ТЕСТИРОВАНИЕ ===

if __name__ == "__main__":
    print("=== Тестирование LIDAR-локализации ===\n")

    # Тест 1: Опорная точка "Вход в туннель"
    print("Тест 1: Робот у входа в туннель")
    print("-" * 60)

    # Симулируем: одометрия врёт, показывает (0.31, 8.03)
    # А на самом деле робот у входа в туннель: (-1.61, 8.99)
    x_odom = 0.31
    y_odom = 8.03
    th_odom = 137.4

    F, L, R = 0.40, 0.45, 0.43  # LIDAR показывает узкий проход

    localizer = LidarLocalizer()
    x_corr, y_corr, th_corr, landmark, conf = localizer.update(
        x_odom, y_odom, th_odom, F, L, R, verbose=True
    )

    print(f"\nРезультат:")
    print(f"  Одометрия:    ({x_odom:.2f}, {y_odom:.2f}, {th_odom:.1f}°)")
    print(f"  Исправлено:   ({x_corr:.2f}, {y_corr:.2f}, {th_corr:.1f}°)")
    print(f"  Ожидается:    (-1.61, 8.99, 87.0°)")
    print(f"  Ошибка после: dx={abs(x_corr - (-1.61)):.2f} dy={abs(y_corr - 8.99):.2f} "
          f"dth={abs(normalize_angle(th_corr - 87.0)):.1f}°")

    print("\n" + "=" * 60 + "\n")

    # Тест 2: Опорная точка "Перед воротами"
    print("Тест 2: Робот перед воротами туннеля")
    print("-" * 60)

    x_odom = 0.10
    y_odom = 4.82
    th_odom = 51.4  # Одометрия показывает неправильный угол

    F, L, R = 2.57, 2.78, 2.79  # Симметрия!

    localizer2 = LidarLocalizer()
    x_corr, y_corr, th_corr, landmark, conf = localizer2.update(
        x_odom, y_odom, th_odom, F, L, R, verbose=True
    )

    print(f"\nРезультат:")
    print(f"  Одометрия:    ({x_odom:.2f}, {y_odom:.2f}, {th_odom:.1f}°)")
    print(f"  Исправлено:   ({x_corr:.2f}, {y_corr:.2f}, {th_corr:.1f}°)")
    print(f"  Ожидается:    (1.22, 6.67, 91.0°)")
    print(f"  Ошибка после: dx={abs(x_corr - 1.22):.2f} dy={abs(y_corr - 6.67):.2f} "
          f"dth={abs(normalize_angle(th_corr - 91.0)):.1f}°")

    print("\n" + "=" * 60 + "\n")

    # Тест 3: Нет опорной точки → используется одометрия
    print("Тест 3: Нет опорной точки")
    print("-" * 60)

    x_odom = 2.5
    y_odom = 10.0
    th_odom = 45.0

    F, L, R = 3.5, 2.1, 2.8  # Непонятная геометрия

    localizer3 = LidarLocalizer()
    x_corr, y_corr, th_corr, landmark, conf = localizer3.update(
        x_odom, y_odom, th_odom, F, L, R, verbose=True
    )

    print(f"\nРезультат:")
    print(f"  Одометрия:    ({x_odom:.2f}, {y_odom:.2f}, {th_odom:.1f}°)")
    print(f"  Исправлено:   ({x_corr:.2f}, {y_corr:.2f}, {th_corr:.1f}°)")
    print(f"  (Должно быть идентично одометрии)")

    print("\n" + "=" * 60)
    print("\nТестирование завершено!")
