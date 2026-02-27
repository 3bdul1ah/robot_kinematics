#!/usr/bin/env python3
"""Round-trip matrices and ROS message types.

Demonstrates:
- matrix_to_transform_msg(...)
- transform_msg_to_matrix(...)
- matrix_to_pose_msg(...)
- pose_msg_to_matrix(...)
"""

import numpy as np

try:
    from geometry_msgs.msg import Pose, Transform
except ImportError:  # pragma: no cover
    Pose = None
    Transform = None

from kinematics import RobotKinematics


def main() -> None:
    if Pose is None or Transform is None:
        print("geometry_msgs not available on PYTHONPATH.")
        return

    T_world_goal = np.array(
        [
            [0.00, -1.00, 0.00, 0.40],
            [1.00, 0.00, 0.00, -0.25],
            [0.00, 0.00, 1.00, 0.70],
            [0.00, 0.00, 0.00, 1.00],
        ],
        dtype=float,
    )

    print("1) matrix_to_transform_msg(T_world_goal)")
    transform_msg = RobotKinematics.matrix_to_transform_msg(T_world_goal)
    print(transform_msg)

    print("\n2) transform_msg_to_matrix(transform_msg)")
    print(RobotKinematics.transform_msg_to_matrix(transform_msg))

    print("\n3) matrix_to_pose_msg(T_world_goal)")
    pose_msg = RobotKinematics.matrix_to_pose_msg(T_world_goal)
    print(pose_msg)

    print("\n4) pose_msg_to_matrix(pose_msg)")
    print(RobotKinematics.pose_msg_to_matrix(pose_msg))


if __name__ == "__main__":
    main()
