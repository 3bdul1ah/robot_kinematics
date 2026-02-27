#!/usr/bin/env python3
"""Broadcast static-only TF frames with RobotKinematics.

Demonstrates:
- broadcast_transform(..., is_static=True)
- broadcast_transforms(..., is_static=True)
"""

import time

import numpy as np

from kinematics import RobotKinematics


def main() -> None:
    Kinematics = RobotKinematics()
    node = Kinematics.node

    # Static chain with unique frame names for this example.
    T_world_static_base = np.array(
        [
            [1.0, 0.0, 0.0, 1.2],
            [0.0, 1.0, 0.0, -0.2],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    T_static_base_cam = np.array(
        [
            [1.0, 0.0, 0.0, 0.2],
            [0.0, 1.0, 0.0, 0.1],
            [0.0, 0.0, 1.0, 0.5],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    T_static_base_lidar = np.array(
        [
            [1.0, 0.0, 0.0, -0.1],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.3],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )

    Kinematics.broadcast_transform(
        "world",
        "static_base_link",
        T_world_static_base,
        is_static=True,
    )
    Kinematics.broadcast_transforms(
        "static_base_link",
        ["static_camera_link", "static_lidar_link"],
        [T_static_base_cam, T_static_base_lidar],
        is_static=True,
    )

    try:
        node.get_logger().info("Broadcasting static frames. Press Ctrl+C to stop.")
        while node.context.ok():
            time.sleep(0.2)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        Kinematics.shutdown()


if __name__ == "__main__":
    main()
