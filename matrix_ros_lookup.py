#!/usr/bin/env python3
"""Wait for TF frames and convert them back into ROS messages.

Demonstrates:
- wait_for_transform(...)
- frame_exists(...)
- lookup_transform(...)
- matrix_to_pose_msg(...)
- matrix_to_stamped_transform_msg(...)
- standalone mode: RobotKinematics()
"""

from kinematics import RobotKinematics


def main() -> None:
    Kinematics = RobotKinematics()
    node = Kinematics.node

    try:
        if not Kinematics.wait_for_transform("world", "tool0", timeout_seconds=5.0):
            node.get_logger().warning("tool0 transform never became available")
        elif not Kinematics.frame_exists("tool0"):
            node.get_logger().warning("tool0 frame is missing after wait")
        else:
            ts, matrix = Kinematics.lookup_transform("world", "tool0")
            if ts is not None and matrix is not None:
                pose = Kinematics.matrix_to_pose_msg(matrix)
                stamped = Kinematics.matrix_to_stamped_transform_msg("world", "tool0_goal", matrix)
                node.get_logger().info(
                    f"tool0 @ x={pose.position.x:.2f} y={pose.position.y:.2f} z={pose.position.z:.2f}"
                )
                node.get_logger().debug(f"Stamped goal ready: {stamped}")
    except KeyboardInterrupt:  # pragma: no cover
        node.get_logger().info("Matrix ROS lookup interrupted")
    finally:
        Kinematics.shutdown()


if __name__ == "__main__":
    main()
