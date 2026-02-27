#!/usr/bin/env python3
"""Validate FK/IK rotation outputs (NumPy only).

Demonstrates:
- is_valid_rotation(...)
"""

import numpy as np

from kinematics import RobotKinematics


def main() -> None:
    cases = {
        "identity": np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        ),
        "noisy_ok": np.array(
            [
                [0.00, -1.00, 0.0001],
                [1.00, 0.00, -0.0001],
                [0.00, 0.0001, 1.00],
            ],
            dtype=float,
        ),
        "singular": np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
            dtype=float,
        ),
    }

    for label, matrix in cases.items():
        result = RobotKinematics.is_valid_rotation(matrix)
        print(f"Checking {label:>10} -> {result}")


if __name__ == "__main__":
    main()
