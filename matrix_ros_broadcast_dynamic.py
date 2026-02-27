#!/usr/bin/env python3
"""Broadcast dynamic-only TF frames with RobotKinematics.

Demonstrates:
- broadcast_transform(..., is_static=False)
"""

import math
import time

import numpy as np

from kinematics import RobotKinematics


def main() -> None:
    Kinematics = RobotKinematics()
    node = Kinematics.node

    try:
        node.get_logger().info("Broadcasting dynamic frames. Press Ctrl+C to stop.")
        while node.context.ok():
            phase = time.time()

            T_world_dynamic_base = np.array(
                [
                    [1.0, 0.0, 0.0, 0.9 + 0.1 * math.sin(phase)],
                    [0.0, 1.0, 0.0, 0.2 * math.cos(phase)],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
                dtype=float,
            )
            T_dynamic_base_tool = np.array(
                [
                    [1.0, 0.0, 0.0, 0.5],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.35 + 0.05 * math.sin(phase * 0.8)],
                    [0.0, 0.0, 0.0, 1.0],
                ],
                dtype=float,
            )

            Kinematics.broadcast_transform(
                "world",
                "dynamic_base_link",
                T_world_dynamic_base,
                is_static=False,
            )
            Kinematics.broadcast_transform(
                "dynamic_base_link",
                "dynamic_tool0",
                T_dynamic_base_tool,
                is_static=False,
            )
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
