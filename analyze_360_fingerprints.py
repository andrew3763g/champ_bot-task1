#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analyze 360° panoramas and create LIDAR fingerprints for localization
"""

import json
import math

def create_360_fingerprint(node):
    """
    Create LIDAR fingerprint from 360° panorama data

    Features extracted:
    - Narrow passages (all readings < 1.5m)
    - Corners (2+ walls within 1.0m)
    - Corridors (symmetric L≈R in multiple directions)
    - Asymmetric patterns (large L vs R difference)
    - Blind spots (8.00m readings)
    """
    panorama = node['panorama']
    fingerprint = {
        'name': node['name'],
        'coords': node['coords'],
        'features': [],
        'readings': {}
    }

    # Extract readings for each direction
    for direction in ['NORTH', 'EAST', 'SOUTH', 'WEST']:
        if direction in panorama:
            data = panorama[direction]
            F, L, R = data['F'], data['L'], data['R']
            fingerprint['readings'][direction] = {'F': F, 'L': L, 'R': R}

    # Feature detection
    if len(panorama) == 4:
        # Count walls (readings < 1.0m)
        walls = []
        for direction, readings in fingerprint['readings'].items():
            if readings['F'] < 1.0:
                walls.append(f"{direction}_F")
            if readings['L'] < 1.0:
                walls.append(f"{direction}_L")
            if readings['R'] < 1.0:
                walls.append(f"{direction}_R")

        # CORNER: 2+ close walls
        if len(walls) >= 2:
            # Check for 90° corner (walls in perpendicular directions)
            corner_dirs = set()
            for w in walls:
                if '_F' in w:
                    corner_dirs.add(w.split('_')[0])

            if 'SOUTH' in corner_dirs and 'WEST' in corner_dirs:
                fingerprint['features'].append('SW_CORNER')
            elif len(walls) >= 3:
                fingerprint['features'].append('TIGHT_CORNER')

        # NARROW PASSAGE: F and (L or R) < 1.5m in any direction
        narrow_dirs = []
        for direction, readings in fingerprint['readings'].items():
            if readings['F'] < 1.5 and (readings['L'] < 1.5 or readings['R'] < 1.5):
                narrow_dirs.append(direction)

        if len(narrow_dirs) >= 2:
            fingerprint['features'].append('CORRIDOR')
        elif narrow_dirs:
            fingerprint['features'].append('NARROW_PASSAGE')

        # SYMMETRIC: L ≈ R in multiple directions (±0.3m tolerance)
        symmetric_dirs = []
        for direction, readings in fingerprint['readings'].items():
            if abs(readings['L'] - readings['R']) < 0.3:
                symmetric_dirs.append(direction)

        if len(symmetric_dirs) >= 3:
            fingerprint['features'].append('SYMMETRIC')

        # ASYMMETRIC: Large L vs R difference
        asymmetric_dirs = []
        for direction, readings in fingerprint['readings'].items():
            if abs(readings['L'] - readings['R']) > 2.0:
                asymmetric_dirs.append(direction)

        if asymmetric_dirs:
            fingerprint['features'].append(f'ASYMMETRIC_{asymmetric_dirs[0]}')

        # BLIND SPOTS: 8.00m readings (LIDAR max range)
        blind_dirs = []
        for direction, readings in fingerprint['readings'].items():
            if readings['F'] == 8.0 or readings['L'] == 8.0 or readings['R'] == 8.0:
                blind_dirs.append(direction)

        if blind_dirs:
            fingerprint['features'].append(f'BLIND_SPOT')

        # OPEN SPACE: All readings > 4.0m
        all_open = all(
            r['F'] > 4.0 and r['L'] > 4.0 and r['R'] > 4.0
            for r in fingerprint['readings'].values()
        )

        if all_open:
            fingerprint['features'].append('OPEN_SPACE')

    return fingerprint


def compare_fingerprints(fp1, fp2):
    """
    Compare two fingerprints and return similarity score (0.0 - 1.0)

    Higher score = more similar
    """
    if not fp1['readings'] or not fp2['readings']:
        return 0.0

    # Feature overlap
    features1 = set(fp1['features'])
    features2 = set(fp2['features'])

    if not features1 or not features2:
        feature_score = 0.0
    else:
        overlap = len(features1 & features2)
        total = len(features1 | features2)
        feature_score = overlap / total if total > 0 else 0.0

    # LIDAR readings similarity (for matching directions)
    reading_scores = []
    for direction in ['NORTH', 'EAST', 'SOUTH', 'WEST']:
        if direction in fp1['readings'] and direction in fp2['readings']:
            r1 = fp1['readings'][direction]
            r2 = fp2['readings'][direction]

            # Compute distance between readings
            diff_F = abs(r1['F'] - r2['F'])
            diff_L = abs(r1['L'] - r2['L'])
            diff_R = abs(r1['R'] - r2['R'])

            # Convert to similarity (exponential decay)
            sim_F = math.exp(-diff_F / 2.0)
            sim_L = math.exp(-diff_L / 2.0)
            sim_R = math.exp(-diff_R / 2.0)

            reading_scores.append((sim_F + sim_L + sim_R) / 3.0)

    reading_score = sum(reading_scores) / len(reading_scores) if reading_scores else 0.0

    # Combined score: 40% features + 60% readings
    return 0.4 * feature_score + 0.6 * reading_score


def main():
    # Load 360° panoramas
    with open('panoramas_360.json', 'r', encoding='utf-8') as f:
        nodes = json.load(f)

    print("=" * 80)
    print("360° LIDAR FINGERPRINT ANALYSIS")
    print("=" * 80)
    print()

    # Create fingerprints
    fingerprints = []
    for node in nodes:
        if node['complete']:
            fp = create_360_fingerprint(node)
            fingerprints.append(fp)

    print(f"Created fingerprints for {len(fingerprints)} complete nodes")
    print()

    # Print fingerprints
    for fp in fingerprints:
        print("=" * 80)
        print(f"NODE: {fp['name']}")
        print("=" * 80)

        if fp['coords']:
            x, y, th = fp['coords']
            print(f"Coords: ({x:.2f}, {y:.2f}) th={th:.1f}°")

        print(f"\nFeatures: {', '.join(fp['features']) if fp['features'] else 'None'}")
        print(f"\nLIDAR Readings:")

        for direction in ['NORTH', 'EAST', 'SOUTH', 'WEST']:
            if direction in fp['readings']:
                r = fp['readings'][direction]
                print(f"  {direction:6s}: F={r['F']:5.2f}  L={r['L']:5.2f}  R={r['R']:5.2f}")

        print()

    # Unique fingerprints analysis
    print("\n" + "=" * 80)
    print("UNIQUE FINGERPRINTS")
    print("=" * 80)
    print()

    # Find most distinctive nodes (low similarity to others)
    for i, fp1 in enumerate(fingerprints):
        similarities = []
        for j, fp2 in enumerate(fingerprints):
            if i != j:
                sim = compare_fingerprints(fp1, fp2)
                similarities.append(sim)

        avg_sim = sum(similarities) / len(similarities) if similarities else 0.0
        uniqueness = 1.0 - avg_sim

        print(f"{fp1['name'][:50]:50s} | Uniqueness: {uniqueness:.2f} | Features: {len(fp1['features'])}")

    # Save fingerprints
    output_file = "lidar_fingerprints_360.json"
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(fingerprints, f, indent=2, ensure_ascii=False)

    print(f"\nFingerprints saved to: {output_file}")


if __name__ == "__main__":
    main()
