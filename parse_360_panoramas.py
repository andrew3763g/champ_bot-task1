#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Парсинг 360° LIDAR-панорам из step.log
======================================

Извлекает данные 4-х направлений (0°, 90°, 180°, 270°) для каждого узла
и создаёт "отпечатки пальцев" для локализации.
"""

import re
import math
import json


def parse_node_name(line):
    """Parsing node name: [NODE] description or [NODE] N - description"""
    match = re.search(r'^\[NODE\]\s+(.+)', line)
    if match:
        text = match.group(1).strip()
        # Skip coordinate lines
        if 'pos=' not in text:
            return text
    return None


def parse_node_coords(line):
    """Парсинг координат узла: [NODE] pos=(x,y) th=angle°"""
    match = re.search(r'pos=\(([-\d.]+),([-\d.]+)\)\s+th=([-\d.]+)', line)
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        th = float(match.group(3))
        return x, y, th
    return None


def parse_telemetry(line):
    """Парсинг телеметрии: [TELEMETRY] pos=(x,y) th=angle°  F=f L=l R=r"""
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


def normalize_angle(angle):
    """Normalize angle to range [0, 360)"""
    while angle < 0:
        angle += 360
    while angle >= 360:
        angle -= 360
    return angle


def classify_direction(angle):
    """
    Classify angle to directions:
    0 deg = NORTH, 90 deg = EAST, 180 deg = SOUTH, 270 deg = WEST

    Tolerance: +/-20 deg for each direction
    """
    angle = normalize_angle(angle)

    if angle < 20 or angle > 340:
        return "NORTH", 0
    elif 70 < angle < 110:
        return "EAST", 90
    elif 160 < angle < 200:
        return "SOUTH", 180
    elif 250 < angle < 290:
        return "WEST", 270
    else:
        return None, angle  # Intermediate angle


def parse_360_panoramas(log_file):
    """
    Парсинг step.log и извлечение 360° панорам для узлов.

    Ожидаемая структура:
    [NODE] название
    [NODE] pos=(x,y) th=angle°
    [TELEMETRY] pos=(x,y) th=0°  F=... L=... R=...    (СЕВЕР)
    ... (повороты)
    [TELEMETRY] pos=(x,y) th=90°  F=... L=... R=...   (ВОСТОК)
    ... (повороты)
    [TELEMETRY] pos=(x,y) th=180°  F=... L=... R=...  (ЮГ)
    ... (повороты)
    [TELEMETRY] pos=(x,y) th=270°  F=... L=... R=...  (ЗАПАД)

    Возвращает: список узлов с 360° данными
    """

    nodes = []
    current_node = None
    current_node_name = None
    current_node_coords = None
    panorama_readings = {}  # {direction: (F, L, R)}

    with open(log_file, 'r', encoding='utf-8') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()

            # Парсинг координат узла (проверяем первым!)
            node_coords = parse_node_coords(line)
            if node_coords:
                if current_node_name:
                    current_node_coords = node_coords
                continue

            # Парсинг названия узла
            node_name = parse_node_name(line)
            if node_name:
                # Если был предыдущий узел, сохраняем его
                if current_node_name and panorama_readings:
                    nodes.append({
                        'name': current_node_name,
                        'coords': current_node_coords,
                        'panorama': panorama_readings.copy(),
                        'complete': len(panorama_readings) == 4
                    })

                # Начинаем новый узел
                current_node_name = node_name
                panorama_readings = {}
                current_node_coords = None
                continue

            # Парсинг телеметрии
            tel = parse_telemetry(line)
            if tel and current_node_name:
                x, y, th, F, L, R = tel

                # Классифицируем направление
                direction, canonical_angle = classify_direction(th)

                if direction and direction not in panorama_readings:
                    # Сохраняем первое чтение для этого направления
                    panorama_readings[direction] = {
                        'angle': th,
                        'F': F,
                        'L': L,
                        'R': R,
                        'pos': (x, y)
                    }

        # Сохраняем последний узел
        if current_node_name and panorama_readings:
            nodes.append({
                'name': current_node_name,
                'coords': current_node_coords,
                'panorama': panorama_readings.copy(),
                'complete': len(panorama_readings) == 4
            })

    return nodes


def create_lidar_fingerprint(panorama):
    """
    Создание "отпечатка пальца" из 360° панорамы.

    Отпечаток — это набор инвариантных признаков:
    - Симметрия (L≈R) в каждом направлении
    - Характерные расстояния (F, L, R)
    - Паттерны (узкий проход, угол, открытое пространство)
    """

    fingerprint = {
        'directions': {},
        'patterns': []
    }

    for direction in ['NORTH', 'EAST', 'SOUTH', 'WEST']:
        if direction in panorama:
            data = panorama[direction]
            F, L, R = data['F'], data['L'], data['R']

            fingerprint['directions'][direction] = {
                'F': F,
                'L': L,
                'R': R,
                'symmetry': abs(L - R),  # 0 = идеальная симметрия
                'narrow': F < 1.0 and L < 1.0 and R < 1.0,
                'wide': F > 5.0 or L > 5.0 or R > 5.0,
                'wall_ahead': F < 1.5,
                'wall_left': L < 1.5,
                'wall_right': R < 1.5
            }

    # Детекция паттернов
    if len(fingerprint['directions']) == 4:
        # Угол: две стены под 90°
        walls = []
        for direction, data in fingerprint['directions'].items():
            if data['wall_ahead']:
                walls.append(direction)

        if len(walls) == 2:
            fingerprint['patterns'].append('CORNER')

        # Узкий проход: стены с двух сторон, свободно спереди/сзади
        narrow_sides = sum(1 for d in ['EAST', 'WEST']
                          if d in fingerprint['directions'] and
                          fingerprint['directions'][d]['wall_left'] or
                          fingerprint['directions'][d]['wall_right'])

        if narrow_sides >= 2:
            fingerprint['patterns'].append('CORRIDOR')

        # Симметричная точка: L≈R во всех направлениях
        symmetries = [data['symmetry'] for data in fingerprint['directions'].values()]
        avg_symmetry = sum(symmetries) / len(symmetries)

        if avg_symmetry < 0.2:
            fingerprint['patterns'].append('SYMMETRIC')

    return fingerprint


def print_node_report(node):
    """Красивый вывод информации о узле с 360° панорамой"""

    print(f"\n{'='*80}")
    print(f"УЗЕЛ: {node['name']}")
    print(f"{'='*80}")

    if node['coords']:
        x, y, th = node['coords']
        print(f"Coords: ({x:.2f}, {y:.2f}) th={th:.1f} deg")

    print(f"360 deg panorama: {'COMPLETE' if node['complete'] else 'INCOMPLETE'}")
    print(f"Directions: {len(node['panorama'])}/4")
    print()

    # Вывод данных по направлениям
    for direction in ['NORTH', 'EAST', 'SOUTH', 'WEST']:
        if direction in node['panorama']:
            data = node['panorama'][direction]
            print(f"  {direction:6s} (th={data['angle']:6.1f}°): "
                  f"F={data['F']:5.2f}  L={data['L']:5.2f}  R={data['R']:5.2f}")
        else:
            print(f"  {direction:6s}: [NO DATA]")

    # Fingerprint
    fingerprint = create_lidar_fingerprint(node['panorama'])

    if fingerprint['patterns']:
        print(f"\n  Patterns: {', '.join(fingerprint['patterns'])}")

    print()


if __name__ == "__main__":
    import sys

    log_file = "step.log"

    if len(sys.argv) > 1:
        log_file = sys.argv[1]

    print(f"Parsing 360 degree panoramas from: {log_file}")
    print()

    nodes = parse_360_panoramas(log_file)

    print(f"Found nodes with panoramas: {len(nodes)}")

    # Вывод отчёта по каждому узлу
    for node in nodes:
        print_node_report(node)

    # Статистика
    complete_nodes = [n for n in nodes if n['complete']]

    print("=" * 80)
    print("STATISTICS")
    print("=" * 80)
    print(f"Total nodes: {len(nodes)}")
    print(f"Complete 360 deg panoramas: {len(complete_nodes)}")
    print(f"Incomplete panoramas: {len(nodes) - len(complete_nodes)}")
    print()

    if complete_nodes:
        print("Nodes with complete panoramas:")
        for node in complete_nodes:
            patterns = create_lidar_fingerprint(node['panorama'])['patterns']
            patterns_str = f" [{', '.join(patterns)}]" if patterns else ""
            print(f"  - {node['name']}{patterns_str}")

    # Save to JSON
    output_file = "panoramas_360.json"

    output_data = []
    for node in nodes:
        fingerprint = create_lidar_fingerprint(node['panorama'])
        output_data.append({
            'name': node['name'],
            'coords': node['coords'],
            'panorama': node['panorama'],
            'fingerprint': fingerprint,
            'complete': node['complete']
        })

    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)

    print(f"\nData saved to: {output_file}")
