#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Extract 360 degree panoramas from step.log
Each node has 4 key directions: NORTH (0°), EAST (90°), SOUTH (180°), WEST (270°)
"""

import re
import json
import math

def normalize_angle(angle):
    """Normalize angle to [0, 360)"""
    while angle < 0:
        angle += 360
    while angle >= 360:
        angle -= 360
    return angle

def classify_direction(angle):
    """
    Classify angle to cardinal direction with ±20° tolerance
    Returns: (direction_name, canonical_angle) or (None, angle)
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
        return None, angle

def parse_telemetry(line):
    """Parse: [TELEMETRY] pos=(x,y) th=angle°  F=f L=l R=r"""
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

def extract_360_panoramas(log_file):
    """
    Extract 360° panoramas from step.log

    Returns: list of nodes with 360° LIDAR data
    """
    nodes = []
    current_node = None
    current_node_name = None
    current_node_coords = None
    panorama = {}

    with open(log_file, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()

            # Detect node marker
            if line.startswith('[NODE] ') and 'pos=' not in line:
                # Save previous node
                if current_node_name and panorama:
                    nodes.append({
                        'name': current_node_name,
                        'coords': current_node_coords,
                        'panorama': panorama.copy(),
                        'complete': len(panorama) == 4
                    })

                # Start new node
                current_node_name = line.replace('[NODE] ', '').strip()
                panorama = {}
                current_node_coords = None
                continue

            # Parse node coordinates
            if line.startswith('[NODE] pos='):
                match = re.search(r'pos=\(([-\d.]+),([-\d.]+)\)\s+th=([-\d.]+)', line)
                if match:
                    x = float(match.group(1))
                    y = float(match.group(2))
                    th = float(match.group(3))
                    current_node_coords = (x, y, th)
                continue

            # Parse telemetry
            tel = parse_telemetry(line)
            if tel and current_node_name:
                x, y, th, F, L, R = tel

                # Classify direction
                direction, canonical = classify_direction(th)

                if direction and direction not in panorama:
                    # Save first reading for this direction
                    panorama[direction] = {
                        'angle': th,
                        'F': F,
                        'L': L,
                        'R': R,
                        'pos': (x, y)
                    }

        # Save last node
        if current_node_name and panorama:
            nodes.append({
                'name': current_node_name,
                'coords': current_node_coords,
                'panorama': panorama.copy(),
                'complete': len(panorama) == 4
            })

    return nodes


def print_node_report(node):
    """Print node 360° panorama"""
    print(f"\n{'='*80}")
    print(f"NODE: {node['name']}")
    print(f"{'='*80}")

    if node['coords']:
        x, y, th = node['coords']
        print(f"Coords: ({x:.2f}, {y:.2f}) th={th:.1f} deg")

    print(f"360 panorama: {'COMPLETE' if node['complete'] else 'INCOMPLETE'}")
    print(f"Directions: {len(node['panorama'])}/4")
    print()

    # Print directions
    for direction in ['NORTH', 'EAST', 'SOUTH', 'WEST']:
        if direction in node['panorama']:
            data = node['panorama'][direction]
            print(f"  {direction:6s} (th={data['angle']:6.1f}°): "
                  f"F={data['F']:5.2f}  L={data['L']:5.2f}  R={data['R']:5.2f}")
        else:
            print(f"  {direction:6s}: [NO DATA]")


if __name__ == "__main__":
    import sys

    log_file = "step.log"
    if len(sys.argv) > 1:
        log_file = sys.argv[1]

    print(f"Extracting 360 degree panoramas from: {log_file}")
    print()

    nodes = extract_360_panoramas(log_file)

    print(f"Found nodes with panoramas: {len(nodes)}")

    # Print report for each node
    for node in nodes:
        print_node_report(node)

    # Statistics
    complete_nodes = [n for n in nodes if n['complete']]

    print("\n" + "="*80)
    print("STATISTICS")
    print("="*80)
    print(f"Total nodes: {len(nodes)}")
    print(f"Complete 360 deg panoramas: {len(complete_nodes)}")
    print(f"Incomplete panoramas: {len(nodes) - len(complete_nodes)}")
    print()

    if complete_nodes:
        print("Nodes with complete 360 deg panoramas:")
        for node in complete_nodes:
            print(f"  - {node['name']}")

    # Save to JSON
    output_file = "panoramas_360.json"

    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(nodes, f, indent=2, ensure_ascii=False)

    print(f"\nData saved to: {output_file}")
