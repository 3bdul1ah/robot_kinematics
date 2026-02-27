#!/usr/bin/env python3
"""Broadcast multiple static transforms and keep running until interrupted."""

import time

import numpy as np

from kinematics import RobotKinematics


def main() -> None:
    Kinematics = RobotKinematics()
    node = Kinematics.node

    try:
        # Create a unique parent frame for this example.
        T_world_multi_camera = np.eye(4)
        T_world_multi_camera[0, 3] = 0.8
        T_world_multi_camera[1, 3] = -0.4
        Kinematics.broadcast_transform(
            "world",
            "multi_camera_link",
            T_world_multi_camera,
            is_static=True,
        )

        # Create two object transforms relative to multi_camera_link.
        T_obj1 = np.eye(4)
        T_obj1[0, 3] = 0.3
        T_obj1[1, 3] = 0.1
        T_obj1[2, 3] = 0.5

        T_obj2 = np.eye(4)
        T_obj2[0, 3] = -0.2
        T_obj2[1, 3] = 0.0
        T_obj2[2, 3] = 0.4

        child_frames = ["object1", "object2"]
        matrices = [T_obj1, T_obj2]

        Kinematics.broadcast_transforms(
            "multi_camera_link",
            child_frames,
            matrices,
            is_static=True,
        )

        while node.context.ok():
            time.sleep(0.2)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        Kinematics.shutdown()


if __name__ == "__main__":
    main()
