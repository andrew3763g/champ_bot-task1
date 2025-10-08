#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Тестирование LIDAR-локализации на реальных данных из fast_navigator.log
========================================================================

Цель: Проверить, как система локализации исправила бы ошибки координат
на реальном проходе робота.
"""

import re
import math
from lidar_localization import LidarLocalizer, normalize_angle


def parse_log_line(line):
    """
    Парсинг строки лога с телеметрией.

    Формат: [TEL] pos=(x,y) th=angle°  F=f L=l R=r

    Возвращает: (x, y, th, F, L, R) или None
    """
    # Пример: [TEL] pos=(-1.87,1.46) th=75.4°  F=4.76 L=8.00 R=3.94
    match = re.search(r'pos=\(([-\d.]+),([-\d.]+)\)\s+th=([-\d.]+)°\s+F=([\d.]+)\s+L=([\d.]+)\s+R=([\d.]+)', line)

    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        th = float(match.group(3))
        F = float(match.group(4))
        L = float(match.group(5))
        R = float(match.group(6))
        return x, y, th, F, L, R

    return None


def parse_node_line(line):
    """
    Парсинг строки узла.

    Формат: УЗЕЛ [N]: описание

    Возвращает: (node_idx, description) или None
    """
    match = re.search(r'УЗЕЛ \[(\d+)\]:\s*(.+)', line)
    if match:
        return int(match.group(1)), match.group(2).strip()
    return None


def test_localization_on_log(log_file_path):
    """
    Тестирование локализации на данных из лога.
    """

    print("=" * 80)
    print("ТЕСТИРОВАНИЕ LIDAR-ЛОКАЛИЗАЦИИ НА РЕАЛЬНЫХ ДАННЫХ")
    print("=" * 80)
    print()

    localizer = LidarLocalizer()

    corrections_found = []
    current_node = None
    step_in_node = 0

    with open(log_file_path, 'r', encoding='utf-8') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()

            # Парсинг узла
            node_info = parse_node_line(line)
            if node_info:
                current_node, description = node_info
                step_in_node = 0
                print(f"\n{'='*80}")
                print(f"УЗЕЛ [{current_node}]: {description}")
                print(f"{'='*80}")
                continue

            # Парсинг телеметрии
            tel_data = parse_log_line(line)
            if not tel_data:
                continue

            x_odom, y_odom, th_odom, F, L, R = tel_data
            step_in_node += 1

            # Применяем локализацию
            x_corr, y_corr, th_corr, landmark, confidence = localizer.update(
                x_odom, y_odom, th_odom, F, L, R, verbose=False
            )

            # ОТЛАДКА: Показать потенциально близкие к опорным точкам данные
            if step_in_node == 1 and current_node is not None:
                print(f"   Начальная телеметрия узла: F={F:.2f} L={L:.2f} R={R:.2f} "
                      f"pos=({x_odom:.2f},{y_odom:.2f}) th={th_odom:.1f}°")

            # Вычисляем поправки
            dx = x_corr - x_odom
            dy = y_corr - y_odom
            dth = normalize_angle(th_corr - th_odom)

            # Если найдена опорная точка или большая коррекция
            if landmark or abs(dx) > 0.1 or abs(dy) > 0.1 or abs(dth) > 1.0:

                if landmark:
                    # ОПОРНАЯ ТОЧКА НАЙДЕНА!
                    print(f"\n🎯 ОПОРНАЯ ТОЧКА ОБНАРУЖЕНА: {landmark} (confidence={confidence:.2f})")
                    print(f"   Строка лога: {line_num}")
                    print(f"   Узел: [{current_node}], шаг: {step_in_node}")
                    print(f"   LIDAR: F={F:.2f} L={L:.2f} R={R:.2f}")
                    print(f"   Одометрия:  ({x_odom:6.2f}, {y_odom:6.2f}, {th_odom:6.1f}°)")
                    print(f"   Исправлено: ({x_corr:6.2f}, {y_corr:6.2f}, {th_corr:6.1f}°)")
                    print(f"   Поправки:   dx={dx:+6.2f}м  dy={dy:+6.2f}м  dth={dth:+6.1f}°")

                    corrections_found.append({
                        'node': current_node,
                        'step': step_in_node,
                        'line': line_num,
                        'landmark': landmark,
                        'confidence': confidence,
                        'odom': (x_odom, y_odom, th_odom),
                        'corrected': (x_corr, y_corr, th_corr),
                        'delta': (dx, dy, dth),
                        'lidar': (F, L, R)
                    })

                elif step_in_node % 10 == 0:  # Показываем каждый 10-й шаг с коррекцией
                    print(f"   Шаг {step_in_node}: pos=({x_odom:.2f},{y_odom:.2f}) → ({x_corr:.2f},{y_corr:.2f}) "
                          f"[Δ={math.hypot(dx, dy):.2f}м]")

    print("\n" + "=" * 80)
    print("ИТОГОВАЯ СТАТИСТИКА")
    print("=" * 80)
    print()
    print(f"Всего найдено опорных точек: {len(corrections_found)}")
    print()

    if corrections_found:
        print("Детали коррекций:")
        print("-" * 80)
        print(f"{'Узел':<6} {'Landmark':<25} {'Conf':<6} {'Δpos (м)':<10} {'Δth (°)':<10}")
        print("-" * 80)

        for corr in corrections_found:
            dx, dy, dth = corr['delta']
            dist_error = math.hypot(dx, dy)
            print(f"{corr['node']:<6} {corr['landmark']:<25} {corr['confidence']:<6.2f} "
                  f"{dist_error:<10.2f} {abs(dth):<10.1f}")

        print("-" * 80)

        # Статистика по типам опорных точек
        landmark_counts = {}
        for corr in corrections_found:
            lm = corr['landmark']
            if lm not in landmark_counts:
                landmark_counts[lm] = 0
            landmark_counts[lm] += 1

        print("\nОпорные точки по типам:")
        for lm, count in landmark_counts.items():
            print(f"  - {lm}: {count} раз")

    else:
        print("⚠️ НИ ОДНОЙ ОПОРНОЙ ТОЧКИ НЕ НАЙДЕНО!")
        print()
        print("Возможные причины:")
        print("  1. LIDAR-данные не соответствуют паттернам опорных точек")
        print("  2. Робот не достиг ни одной опорной точки (остановился раньше)")
        print("  3. Пороги детекции слишком жёсткие")

    print("\n" + "=" * 80)

    return corrections_found


if __name__ == "__main__":
    log_file = "fast_navigator.log"

    print(f"Анализ лога: {log_file}\n")

    corrections = test_localization_on_log(log_file)

    if corrections:
        print("\n✅ Тестирование завершено успешно!")
        print(f"\nНайдено {len(corrections)} коррекций координат.")
        print("\nСледующий шаг: Интегрировать локализацию в fast_navigator.py")
    else:
        print("\n⚠️ Опорные точки не обнаружены.")
        print("\nВозможно, робот остановился до достижения первой опорной точки,")
        print("или пороги детекции нужно ослабить.")
