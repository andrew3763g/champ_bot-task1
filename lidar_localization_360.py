#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LIDAR-based Localization with 360° Panoramas
==============================================

Uses 360° LIDAR fingerprints for robust localization.
Based on panoramas_360.json and lidar_fingerprints_360.json
"""

import math
import json
import sys
import io

# Fix Windows console encoding
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')


# === LANDMARKS from 360° panoramas ===
# Most unique nodes with complete 360° data
LANDMARKS_360 = {
    "NODE_0_START": {
        "coords": (0.00, -0.00, 0.0),
        "panorama": {
            "NORTH": (3.63, 0.79, 0.80),
            "EAST": (0.98, 1.03, 0.72),
            "SOUTH": (6.46, 1.23, 6.03),
            "WEST": (0.56, 0.64, 0.57)
        },
        "features": ["TIGHT_CORNER", "CORRIDOR", "ASYMMETRIC_SOUTH"],
        "uniqueness": 0.73,
        "description": "Start pocket - tight corner"
    },
    "NODE_4_PAST_DUCKS": {
        "coords": (0.99, 7.31, 454.6),
        "panorama": {
            "NORTH": (2.15, 2.15, 2.53),
            "EAST": (2.18, 2.63, 2.15),
            "SOUTH": (5.51, 3.33, 3.68),
            "WEST": (6.06, 1.07, 7.55)
        },
        "features": ["ASYMMETRIC_WEST"],
        "uniqueness": 0.80,  # HIGHEST!
        "description": "Past ducks - asymmetric west"
    },
    "NODE_5_BEFORE_GATE": {
        "coords": (-1.71, 8.19, 522.0),
        "panorama": {
            "NORTH": (3.85, 4.76, 1.68),
            "EAST": (1.31, 1.29, 1.58),
            "SOUTH": (2.89, 1.49, 4.94),
            "WEST": (1.63, 2.88, 1.53)
        },
        "features": ["NARROW_PASSAGE", "ASYMMETRIC_NORTH"],
        "uniqueness": 0.65,
        "description": "Before tunnel gate - narrow passage"
    },
    "NODE_6_ROOM2_CORRIDOR": {
        "coords": (-1.90, 16.13, 450.9),
        "panorama": {
            "NORTH": (4.60, 4.66, 5.35),
            "EAST": (1.26, 5.74, 1.23),
            "SOUTH": (3.44, 1.78, 5.20),
            "WEST": (1.17, 1.55, 1.08)
        },
        "features": ["CORRIDOR", "ASYMMETRIC_EAST"],
        "uniqueness": 0.68,
        "description": "Room 2 corridor after tunnel"
    },
    "NODE_7_CORNER": {
        "coords": (-6.58, 15.61, 546.3),
        "panorama": {
            "NORTH": (8.00, 5.89, 2.37),
            "EAST": (2.66, 8.00, 0.89),
            "SOUTH": (0.49, 0.59, 0.50),
            "WEST": (0.84, 0.56, 0.88)
        },
        "features": ["SW_CORNER", "CORRIDOR", "ASYMMETRIC_NORTH", "BLIND_SPOT"],
        "uniqueness": 0.66,
        "description": "SW corner of room 2 - distinctive corner"
    },
    "NODE_8_OPPOSITE_MAT": {
        "coords": (-7.23, 19.75, 458.9),
        "panorama": {
            "NORTH": (8.00, 6.22, 8.00),
            "EAST": (4.91, 5.87, 3.36),
            "SOUTH": (0.60, 0.79, 0.56),
            "WEST": (1.56, 0.82, 5.05)
        },
        "features": ["TIGHT_CORNER", "NARROW_PASSAGE", "ASYMMETRIC_EAST", "BLIND_SPOT"],
        "uniqueness": 0.72,
        "description": "Opposite to mat - tight corner with blind spots"
    },
    "NODE_9_FINISH": {
        "coords": (-4.42, 20.13, 368.1),
        "panorama": {
            "NORTH": (6.35, 3.50, 6.81),
            "EAST": (5.48, 7.59, 4.85),
            "SOUTH": (4.01, 5.86, 3.45),
            "WEST": (2.94, 4.24, 2.55)
        },
        "features": ["ASYMMETRIC_NORTH"],
        "uniqueness": 0.71,
        "description": "Finish area - last call"
    }
}


def normalize_angle(angle_deg):
    """Normalize angle to [-180, 180]"""
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg < -180:
        angle_deg += 360
    return angle_deg


def classify_direction_from_heading(th_deg):
    """
    Classify robot heading to cardinal direction

    0° = NORTH, 90° = EAST, 180° = SOUTH, 270° = WEST
    Returns closest cardinal direction
    """
    th_deg = th_deg % 360

    if th_deg < 45 or th_deg >= 315:
        return "NORTH"
    elif 45 <= th_deg < 135:
        return "EAST"
    elif 135 <= th_deg < 225:
        return "SOUTH"
    else:
        return "WEST"


def detect_landmark_360(F, L, R, x_approx, y_approx, th_approx, verbose=False):
    """
    Detect landmark using current LIDAR + approximate heading

    Since we only have F, L, R (not full 360° scan), we use heading
    to determine which direction robot is facing, then match against
    panorama data for that direction.

    Args:
        F, L, R: Current LIDAR readings
        x_approx, y_approx, th_approx: Approximate position from odometry
        verbose: Print debug info

    Returns:
        (landmark_name, confidence, corrected_x, corrected_y, corrected_th)
        or (None, 0, x_approx, y_approx, th_approx)
    """

    # Determine which direction robot is facing
    facing_dir = classify_direction_from_heading(th_approx)

    if verbose:
        print(f"[DETECT] F={F:.2f} L={L:.2f} R={R:.2f} heading={th_approx:.1f}° facing={facing_dir}")

    best_match = None
    best_score = 0.0

    # Try to match against all landmarks
    for landmark_name, landmark in LANDMARKS_360.items():
        if facing_dir not in landmark['panorama']:
            continue

        # Get expected LIDAR for this direction
        expected_F, expected_L, expected_R = landmark['panorama'][facing_dir]

        # Compute similarity score
        diff_F = abs(F - expected_F)
        diff_L = abs(L - expected_L)
        diff_R = abs(R - expected_R)

        # Weighted score (exponential decay)
        score = (
            math.exp(-diff_F / 1.0) * 0.5 +  # F is most important
            math.exp(-diff_L / 1.0) * 0.25 +
            math.exp(-diff_R / 1.0) * 0.25
        )

        # Boost score if position is close
        lm_x, lm_y, lm_th = landmark['coords']
        dist = math.hypot(x_approx - lm_x, y_approx - lm_y)

        if dist < 2.0:
            score *= 1.5  # 50% boost for nearby positions

        if verbose:
            print(f"  {landmark_name}: score={score:.3f} expected=({expected_F:.2f}, {expected_L:.2f}, {expected_R:.2f}) dist={dist:.2f}m")

        if score > best_score:
            best_score = score
            best_match = landmark_name

    # Threshold for accepting match
    if best_score > 0.60:  # 60% similarity threshold
        landmark = LANDMARKS_360[best_match]
        lm_x, lm_y, lm_th = landmark['coords']

        if verbose:
            print(f"[MATCH] {best_match} (score={best_score:.2f})")
            print(f"  Correcting: ({x_approx:.2f}, {y_approx:.2f}, {th_approx:.1f}°)")
            print(f"  → ({lm_x:.2f}, {lm_y:.2f}, {lm_th:.1f}°)")

        return best_match, best_score, lm_x, lm_y, lm_th

    return None, 0, x_approx, y_approx, th_approx


class LidarLocalizer360:
    """
    LIDAR-based localizer using 360° panorama fingerprints

    Maintains correction history and applies exponential decay
    """

    def __init__(self):
        self.last_correction = None
        self.correction_decay = 0.95  # 5% decay per step
        self.corrections_history = []

    def update(self, x_odom, y_odom, th_odom, F, L, R, verbose=False):
        """
        Update localization with new LIDAR reading

        Args:
            x_odom, y_odom, th_odom: Odometry position
            F, L, R: LIDAR readings
            verbose: Print debug info

        Returns:
            (x_corrected, y_corrected, th_corrected, landmark_name, confidence)
        """

        # Try to detect landmark
        landmark, confidence, x_corr, y_corr, th_corr = detect_landmark_360(
            F, L, R, x_odom, y_odom, th_odom, verbose=verbose
        )

        if landmark:
            # New correction found!
            dx = x_corr - x_odom
            dy = y_corr - y_odom
            dth = normalize_angle(th_corr - th_odom)

            self.last_correction = {
                'dx': dx,
                'dy': dy,
                'dth': dth,
                'weight': 1.0,
                'landmark': landmark,
                'confidence': confidence
            }

            self.corrections_history.append(self.last_correction.copy())

            return x_corr, y_corr, th_corr, landmark, confidence

        # No landmark detected - apply decaying correction if available
        if self.last_correction:
            # Decay weight
            self.last_correction['weight'] *= self.correction_decay

            # Apply weighted correction
            dx = self.last_correction['dx'] * self.last_correction['weight']
            dy = self.last_correction['dy'] * self.last_correction['weight']
            dth = self.last_correction['dth'] * self.last_correction['weight']

            x_corr = x_odom + dx
            y_corr = y_odom + dy
            th_corr = th_odom + dth

            if verbose and self.last_correction['weight'] > 0.1:
                print(f"[DECAY] Applying decayed correction from {self.last_correction['landmark']}: "
                      f"weight={self.last_correction['weight']:.2f}")

            return x_corr, y_corr, th_corr, None, 0

        # No correction available - return odometry as-is
        return x_odom, y_odom, th_odom, None, 0

    def get_stats(self):
        """Return statistics about corrections"""
        if not self.corrections_history:
            return None

        return {
            'total_corrections': len(self.corrections_history),
            'landmarks_detected': [c['landmark'] for c in self.corrections_history],
            'avg_confidence': sum(c['confidence'] for c in self.corrections_history) / len(self.corrections_history)
        }


def test_landmark_detection():
    """Test landmark detection with sample data"""
    print("=" * 80)
    print("TESTING 360° LANDMARK DETECTION")
    print("=" * 80)
    print()

    # Test 1: Start position
    print("Test 1: Start position (pocket)")
    print("-" * 80)
    F, L, R = 3.63, 0.79, 0.80
    x, y, th = 0.0, 0.0, 0.0
    landmark, conf, xc, yc, thc = detect_landmark_360(F, L, R, x, y, th, verbose=True)
    print()

    # Test 2: Corner (SW)
    print("Test 2: SW Corner")
    print("-" * 80)
    F, L, R = 0.49, 0.59, 0.50  # SOUTH direction at corner
    x, y, th = -6.5, 15.6, 180.0  # Facing south
    landmark, conf, xc, yc, thc = detect_landmark_360(F, L, R, x, y, th, verbose=True)
    print()

    # Test 3: Room 2 corridor
    print("Test 3: Room 2 corridor")
    print("-" * 80)
    F, L, R = 1.26, 5.74, 1.23  # EAST direction
    x, y, th = -1.9, 16.0, 90.0  # Facing east
    landmark, conf, xc, yc, thc = detect_landmark_360(F, L, R, x, y, th, verbose=True)
    print()

    print("=" * 80)


if __name__ == "__main__":
    test_landmark_detection()
