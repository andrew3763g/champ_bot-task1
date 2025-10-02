#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Анализ маршрутов из step.log и step1.log
Извлекает узлы графа, вычисляет расстояния и углы поворотов
"""
import math
import re
import sys
import io

# Fix Windows encoding
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

def parse_log(filename):
    """Извлечь узлы из лог-файла"""
    nodes = []
    with open(filename, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    i = 0
    while i < len(lines):
        if '[NODE]' in lines[i] and 'pos=' not in lines[i]:
            # Строка с названием узла
            name_match = re.search(r'\[NODE\] (.+)', lines[i])
            if name_match and i + 1 < len(lines):
                name = name_match.group(1).strip()
                # Следующая строка с координатами
                pos_line = lines[i + 1]
                pos_match = re.search(r'pos=\(([-\d.]+),([-\d.]+)\)\s+th=([-\d.]+)', pos_line)
                if pos_match:
                    x = float(pos_match.group(1))
                    y = float(pos_match.group(2))
                    th = float(pos_match.group(3))
                    nodes.append({
                        'name': name,
                        'x': x,
                        'y': y,
                        'th': th,
                        'th_rad': math.radians(th)
                    })
                i += 2
                continue
        i += 1

    return nodes

def distance(n1, n2):
    """Евклидово расстояние между узлами"""
    return math.hypot(n2['x'] - n1['x'], n2['y'] - n1['y'])

def angle_to_target(n1, n2):
    """Угол от n1 к n2 относительно оси X (восток = 0°)"""
    dx = n2['x'] - n1['x']
    dy = n2['y'] - n1['y']
    return math.degrees(math.atan2(dy, dx))

def angle_diff(a1, a2):
    """Разница углов с нормализацией"""
    diff = a2 - a1
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff

def analyze_route(nodes, route_name):
    """Анализ маршрута: расстояния, углы, повороты"""
    print("=" * 80)
    print(f"МАРШРУТ: {route_name}")
    print("=" * 80)
    print(f"Всего узлов: {len(nodes)}\n")

    total_distance = 0
    total_turns = 0

    for i, node in enumerate(nodes):
        print(f"[{i}] {node['name']}")
        print(f"    pos=({node['x']:.2f}, {node['y']:.2f})  th={node['th']:.1f}°")

        if i > 0:
            prev = nodes[i-1]
            dist = distance(prev, node)
            total_distance += dist

            # Угол к следующему узлу
            target_angle = angle_to_target(prev, node)

            # Требуемый поворот (от текущего th к target_angle)
            turn_needed = angle_diff(prev['th'], target_angle)

            # Фактический поворот робота между узлами
            actual_turn = angle_diff(prev['th'], node['th'])
            total_turns += abs(actual_turn)

            print(f"    ├─ расстояние от узла {i-1}: {dist:.2f}м")
            print(f"    ├─ направление к узлу: {target_angle:.1f}°")
            print(f"    ├─ требуется поворот: {turn_needed:+.1f}°")
            print(f"    └─ фактический поворот: {actual_turn:+.1f}°")

        print()

    print("-" * 80)
    print(f"ИТОГО:")
    print(f"  Пройденное расстояние: {total_distance:.2f}м")
    print(f"  Суммарный поворот: {total_turns:.1f}°")
    print(f"  Финишная позиция: ({nodes[-1]['x']:.2f}, {nodes[-1]['y']:.2f})")
    print("=" * 80)
    print()

def compare_routes(route1, route2):
    """Сравнение двух маршрутов"""
    print("\n" + "=" * 80)
    print("СРАВНЕНИЕ МАРШРУТОВ")
    print("=" * 80)

    dist1 = sum(distance(route1[i], route1[i+1]) for i in range(len(route1)-1))
    dist2 = sum(distance(route2[i], route2[i+1]) for i in range(len(route2)-1))

    turns1 = sum(abs(angle_diff(route1[i]['th'], route1[i+1]['th'])) for i in range(len(route1)-1))
    turns2 = sum(abs(angle_diff(route2[i]['th'], route2[i+1]['th'])) for i in range(len(route2)-1))

    print(f"{'Параметр':<30} {'step.log':>15} {'step1.log':>15} {'Разница':>15}")
    print("-" * 80)
    print(f"{'Количество узлов':<30} {len(route1):>15} {len(route2):>15} {len(route1)-len(route2):>+15}")
    print(f"{'Пройденное расстояние (м)':<30} {dist1:>15.2f} {dist2:>15.2f} {dist1-dist2:>+15.2f}")
    print(f"{'Суммарный поворот (°)':<30} {turns1:>15.1f} {turns2:>15.1f} {turns1-turns2:>+15.1f}")
    print(f"{'Финиш X':<30} {route1[-1]['x']:>15.2f} {route2[-1]['x']:>15.2f} {route1[-1]['x']-route2[-1]['x']:>+15.2f}")
    print(f"{'Финиш Y':<30} {route1[-1]['y']:>15.2f} {route2[-1]['y']:>15.2f} {route1[-1]['y']-route2[-1]['y']:>+15.2f}")

    print("\nВЫВОД:")
    if dist1 < dist2:
        print(f"  ✓ step.log короче на {dist2-dist1:.2f}м ({(dist2-dist1)/dist2*100:.1f}%)")
    else:
        print(f"  ✓ step1.log короче на {dist1-dist2:.2f}м ({(dist1-dist2)/dist1*100:.1f}%)")

    if turns1 < turns2:
        print(f"  ✓ step.log - меньше поворотов на {turns2-turns1:.1f}°")
    else:
        print(f"  ✓ step1.log - меньше поворотов на {turns1-turns2:.1f}°")

    print("=" * 80)

if __name__ == "__main__":
    print("\nАНАЛИЗ МАРШРУТОВ РОБОТА\n")

    # Парсим оба маршрута
    route_step = parse_log("step.log")
    route_step1 = parse_log("step1.log")

    # Анализируем каждый
    analyze_route(route_step, "step.log (южный путь - широкие проходы)")
    analyze_route(route_step1, "step1.log (северный путь - узкий проход)")

    # Сравниваем
    compare_routes(route_step, route_step1)

    print("\n\nГРАФ ДЛЯ PYTHON (step.log - рекомендуемый маршрут)")
    print("=" * 80)
    print("NODES = {")
    for i, node in enumerate(route_step):
        next_nodes = [i+1] if i < len(route_step)-1 else []
        print(f"    {i}: {{")
        print(f"        'name': '{node['name']}',")
        print(f"        'pos': ({node['x']:.2f}, {node['y']:.2f}),")
        print(f"        'th': {node['th_rad']:.4f},  # {node['th']:.1f}°")
        print(f"        'next': {next_nodes}")
        print(f"    }},")
    print("}")
