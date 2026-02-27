#!/usr/bin/env python3
"""Broadcast mixed TF frames (static + dynamic) with RobotKinematics.

Demonstrates:
- broadcast_transform(..., is_static=True)
- broadcast_transform(..., is_static=False)
"""

import math
import time

import numpy as np

from kinematics import RobotKinematics


def main() -> None:
    Kinematics = RobotKinematics()
    node = Kinematics.node

    # Static base frame used by lookup example (world -> tool0 pipeline).
    T_world_base = np.array(
        [
            [1.0, 0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0, 0.2],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    Kinematics.broadcast_transform("world", "base_link", T_world_base, is_static=True)

    try:
        node.get_logger().info("Broadcasting mixed frames. Press Ctrl+C to stop.")
        while node.context.ok():
            phase = time.time()
            T_base_tool = np.array(
                [
                    [1.0, 0.0, 0.0, 0.6],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.4 + 0.05 * math.sin(phase)],
                    [0.0, 0.0, 0.0, 1.0],
                ],
                dtype=float,
            )
            Kinematics.broadcast_transform("base_link", "tool0", T_base_tool, is_static=False)
            time.sleep(0.05)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    except Exception:
        if node.context.ok():
            raise
    finally:
        Kinematics.shutdown()


if __name__ == "__main__":
    main()
