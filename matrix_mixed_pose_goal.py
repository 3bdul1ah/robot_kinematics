#!/usr/bin/env python3
"""Convert a Pose message to a 4x4 matrix and back.

Demonstrates:
- pose_msg_to_matrix(...)
- matrix_to_pose_msg(...)
"""

try:
    from geometry_msgs.msg import Pose
except ImportError:  # pragma: no cover
    Pose = None

from kinematics import RobotKinematics


def main() -> None:
    if Pose is None:
        print("geometry_msgs not available on PYTHONPATH.")
        return

    pose_msg = Pose()
    pose_msg.position.x = 0.55
    pose_msg.position.y = -0.35
    pose_msg.position.z = 0.80
    pose_msg.orientation.x = 0.0
    pose_msg.orientation.y = 0.0
    pose_msg.orientation.z = 0.7071
    pose_msg.orientation.w = 0.7071

    print("1) pose_msg_to_matrix(pose_msg)")
    matrix = RobotKinematics.pose_msg_to_matrix(pose_msg)
    print(matrix)

    print("\n2) matrix_to_pose_msg(matrix)")
    pose_copy = RobotKinematics.matrix_to_pose_msg(matrix)
    print(pose_copy)


if __name__ == "__main__":
    main()
