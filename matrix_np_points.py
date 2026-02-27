#!/usr/bin/env python3
"""Move camera detections into the world frame (NumPy only).

Demonstrates:
- apply_transform(...)
"""

import numpy as np

from kinematics import RobotKinematics


def main() -> None:
    print("1) Define camera pose (world -> camera)")
    T_world_camera = np.array(
        [
            [0.90, 0.40, 0.15, 1.10],
            [-0.35, 0.92, -0.15, 0.35],
            [-0.20, 0.05, 0.98, 0.65],
            [0.00, 0.00, 0.00, 1.00],
        ],
        dtype=float,
    )
    print(T_world_camera)

    print("\n2) Sample camera-frame detections")
    camera_points = np.array(
        [
            [0.30, 0.00, 0.00],
            [0.50, -0.10, 0.05],
            [0.62, 0.18, -0.02],
        ],
        dtype=float,
    )
    print(camera_points)

    print("\n3) apply_transform for each detection -> world coordinates")
    world_points = [RobotKinematics.apply_transform(T_world_camera, p) for p in camera_points]
    print(np.vstack(world_points))


if __name__ == "__main__":
    main()
