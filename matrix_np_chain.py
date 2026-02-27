#!/usr/bin/env python3
"""Chain, invert, and compare frames with RobotKinematics (NumPy only).

Demonstrates:
1. compose_transforms(...)
2. invert_transform(...)
3. relative_transform(...)
"""

import numpy as np

from kinematics import RobotKinematics


def main() -> None:
    print("1) compose_transforms(world -> base, base -> tool) -> world -> tool")
    T_world_base = np.array(
        [
            [0.96, -0.28, 0.00, 0.80],
            [0.28, 0.96, 0.00, 0.30],
            [0.00, 0.00, 1.00, 0.12],
            [0.00, 0.00, 0.00, 1.00],
        ],
        dtype=float,
    )
    T_base_tool = np.array(
        [
            [0.98, 0.00, 0.18, 0.45],
            [0.00, 1.00, 0.00, 0.04],
            [-0.18, 0.00, 0.98, 0.32],
            [0.00, 0.00, 0.00, 1.00],
        ],
        dtype=float,
    )
    T_world_tool = RobotKinematics.compose_transforms(T_world_base, T_base_tool)
    print(T_world_tool)

    print("\n2) invert_transform(base -> tool) -> tool -> base")
    T_tool_base = RobotKinematics.invert_transform(T_base_tool)
    print(T_tool_base)

    print("\n3) relative_transform(world -> camera, world -> tool) -> camera -> tool")
    T_world_camera = np.array(
        [
            [0.87, -0.50, 0.00, 1.20],
            [0.50, 0.87, 0.00, -0.10],
            [0.00, 0.00, 1.00, 0.60],
            [0.00, 0.00, 0.00, 1.00],
        ],
        dtype=float,
    )
    T_camera_tool = RobotKinematics.relative_transform(T_world_camera, T_world_tool)
    print(T_camera_tool)


if __name__ == "__main__":
    main()
