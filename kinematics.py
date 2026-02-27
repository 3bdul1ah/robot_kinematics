from __future__ import annotations

"""
robot_kinematics.py
-------------------
ROS 2 / TF2 kinematics utilities for homogeneous transforms, TF broadcasting,
TF lookup, and matrix/message conversions.

Use README.md and matrix_* scripts for runnable end-to-end examples.
"""

import threading
from typing import Optional, Sequence, Tuple

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Pose, Transform, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.time import Time
from scipy.spatial.transform import Rotation


class RobotKinematics:
    """
    TF2 and homogeneous-transform toolkit for ROS 2 nodes.

    Supports both attached-node mode and standalone mode with an internal node.
    """

    def __init__(self, node: Node | None = None) -> None:
        """
        Initialize TF interfaces.

        If ``node`` is provided, attach to that node.
        If ``node`` is ``None``, create and spin an internal node in a background thread.
        """

        self._external_node = node is not None

        if node is None:
            if not rclpy.ok():
                rclpy.init()

            self.node = rclpy.create_node("robot_kinematics")
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self.node)
            self._spin_thread = threading.Thread(
                target=self._spin_executor,
                name="robot_kinematics_executor",
                daemon=True,
            )
            self._spin_thread.start()
        else:
            self.node = node
            self._executor = None
            self._spin_thread = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.dynamic_broadcaster = tf2_ros.TransformBroadcaster(self.node)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self.node)

    def _spin_executor(self) -> None:
        if self._executor is None:
            return
        try:
            self._executor.spin()
        except (ExternalShutdownException, RuntimeError):
            # Normal path during teardown when context/executor is shutting down.
            pass

    def shutdown(self) -> None:
        """
        Shut down internal ROS resources created in standalone mode.

        No-op when attached to an external node.
        """

        if self._external_node:
            return

        if self._executor is not None:
            try:
                self._executor.remove_node(self.node)
            except Exception:
                pass
            try:
                self._executor.shutdown()
            except Exception:
                pass
            self._executor = None

        if self._spin_thread is not None:
            self._spin_thread.join(timeout=0.2)
            self._spin_thread = None

        try:
            self.node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.try_shutdown()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # TF Broadcasting
    # ------------------------------------------------------------------

    def broadcast_transform(
        self,
        parent_frame: str,
        child_frame: str,
        homogeneous_matrix: np.ndarray,
        is_static: bool = False,
    ) -> None:
        """
        Publish one transform from a 4x4 homogeneous matrix.

        Set ``is_static=True`` for fixed frames, ``False`` for continuously updated frames.
        """
        homogeneous_matrix = np.asarray(homogeneous_matrix, dtype=float)

        ts = TransformStamped()
        ts.header.stamp    = self.node.get_clock().now().to_msg()
        ts.header.frame_id = parent_frame
        ts.child_frame_id  = child_frame
        ts.transform       = self.matrix_to_transform_msg(homogeneous_matrix)

        if is_static:
            self.static_broadcaster.sendTransform(ts)
        else:
            self.dynamic_broadcaster.sendTransform(ts)

    def broadcast_transforms(
        self,
        parent_frame: str,
        child_frames: Sequence[str],
        homogeneous_matrices: Sequence[np.ndarray],
        is_static: bool = False,
    ) -> None:
        """
        Publish multiple transforms that share one parent frame.

        ``child_frames`` and ``homogeneous_matrices`` must have the same length.
        """
        if len(child_frames) != len(homogeneous_matrices):
            raise ValueError("child_frames and homogeneous_matrices must be the same length.")

        now = self.node.get_clock().now().to_msg()
        transforms: list[TransformStamped] = []
        for frame, matrix in zip(child_frames, homogeneous_matrices):
            ts = TransformStamped()
            ts.header.stamp = now
            ts.header.frame_id = parent_frame
            ts.child_frame_id = frame
            ts.transform = self.matrix_to_transform_msg(np.asarray(matrix, dtype=float))
            transforms.append(ts)

        if is_static:
            self.static_broadcaster.sendTransform(transforms)
        else:
            self.dynamic_broadcaster.sendTransform(transforms)

    # ------------------------------------------------------------------
    # TF Lookup & Waiting
    # ------------------------------------------------------------------

    def lookup_transform(
        self,
        parent_frame: str,
        child_frame: str,
        timeout_seconds: float = 1.0,
    ) -> Tuple[Optional[TransformStamped], Optional[np.ndarray]]:
        """
        Lookup a transform and return both ROS message and 4x4 matrix forms.

        Returns ``(None, None)`` on failure and logs a warning.
        """
        try:
            ts = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                Time(),
                timeout=Duration(seconds=timeout_seconds),
            )
            return ts, self.transform_msg_to_matrix(ts.transform)
        except Exception as err:
            self.node.get_logger().warn(
                f"TF lookup failed ({parent_frame} -> {child_frame}): {err}"
            )
            return None, None

    def wait_for_transform(
        self,
        parent_frame: str,
        child_frame: str,
        timeout_seconds: float = 5.0,
    ) -> bool:
        """
        Wait until a transform is available in the TF buffer.

        Returns ``True`` when available, ``False`` on timeout or error.
        """
        try:
            return self.tf_buffer.can_transform(
                parent_frame,
                child_frame,
                Time(),
                timeout=Duration(seconds=timeout_seconds),
            )
        except Exception as err:
            self.node.get_logger().warn(
                f"wait_for_transform failed ({parent_frame} -> {child_frame}): {err}"
            )
            return False

    def frame_exists(self, frame_name: str) -> bool:
        """
        Check whether a frame can be resolved in the current TF graph.
        """
        return self.tf_buffer.can_transform(
            frame_name,
            frame_name,
            Time(),
        )

    # ------------------------------------------------------------------
    # Matrix -> Stamped Transform
    # ------------------------------------------------------------------

    def matrix_to_stamped_transform_msg(
        self,
        parent_frame: str,
        child_frame: str,
        homogeneous_matrix: np.ndarray,
    ) -> TransformStamped:
        """
        Convert a 4x4 matrix into a time-stamped ``TransformStamped`` message.
        """
        ts = TransformStamped()
        ts.header.stamp    = self.node.get_clock().now().to_msg()
        ts.header.frame_id = parent_frame
        ts.child_frame_id  = child_frame
        ts.transform       = self.matrix_to_transform_msg(
            np.asarray(homogeneous_matrix, dtype=float)
        )
        return ts

    # ------------------------------------------------------------------
    # Matrix <-> Transform
    # ------------------------------------------------------------------

    @staticmethod
    def matrix_to_transform_msg(homogeneous_matrix: np.ndarray) -> Transform:
        """
        Convert a 4x4 homogeneous matrix to ``geometry_msgs/Transform``.

        Raises ``ValueError`` if the rotation submatrix determinant is non-positive.
        """
        homogeneous_matrix = np.asarray(homogeneous_matrix, dtype=float)
        R   = homogeneous_matrix[:3, :3]
        det = float(np.linalg.det(R))
        if det <= 0.0:
            raise ValueError(f"Rotation matrix det={det:.6f} — must be +1 (SO(3)).")

        q   = Rotation.from_matrix(R).as_quat()   # [x, y, z, w]
        msg = Transform()
        msg.translation.x, msg.translation.y, msg.translation.z = (
            float(homogeneous_matrix[0, 3]),
            float(homogeneous_matrix[1, 3]),
            float(homogeneous_matrix[2, 3]),
        )
        msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w = (
            float(q[0]), float(q[1]), float(q[2]), float(q[3])
        )
        return msg

    @staticmethod
    def transform_msg_to_matrix(transform_msg: Transform) -> np.ndarray:
        """
        Convert ``geometry_msgs/Transform`` to a 4x4 homogeneous matrix.
        """
        T = np.eye(4, dtype=float)
        q = [
            float(transform_msg.rotation.x),
            float(transform_msg.rotation.y),
            float(transform_msg.rotation.z),
            float(transform_msg.rotation.w),
        ]
        T[:3, :3] = Rotation.from_quat(q).as_matrix()
        T[0, 3]   = float(transform_msg.translation.x)
        T[1, 3]   = float(transform_msg.translation.y)
        T[2, 3]   = float(transform_msg.translation.z)
        return T

    # ------------------------------------------------------------------
    # Matrix <-> Pose
    # ------------------------------------------------------------------

    @staticmethod
    def pose_msg_to_matrix(pose_msg: Pose) -> np.ndarray:
        """
        Convert ``geometry_msgs/Pose`` to a 4x4 homogeneous matrix.
        """
        T = np.eye(4, dtype=float)
        q = [
            float(pose_msg.orientation.x),
            float(pose_msg.orientation.y),
            float(pose_msg.orientation.z),
            float(pose_msg.orientation.w),
        ]
        T[:3, :3] = Rotation.from_quat(q).as_matrix()
        T[0, 3]   = float(pose_msg.position.x)
        T[1, 3]   = float(pose_msg.position.y)
        T[2, 3]   = float(pose_msg.position.z)
        return T

    @staticmethod
    def matrix_to_pose_msg(homogeneous_matrix: np.ndarray) -> Pose:
        """
        Convert a 4x4 homogeneous matrix to ``geometry_msgs/Pose``.
        """
        homogeneous_matrix = np.asarray(homogeneous_matrix, dtype=float)
        q   = Rotation.from_matrix(homogeneous_matrix[:3, :3]).as_quat()
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = (
            float(homogeneous_matrix[0, 3]),
            float(homogeneous_matrix[1, 3]),
            float(homogeneous_matrix[2, 3]),
        )
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = (
            float(q[0]), float(q[1]), float(q[2]), float(q[3])
        )
        return msg

    # ------------------------------------------------------------------
    # Transformation Operations
    # ------------------------------------------------------------------

    @staticmethod
    def compose_transforms(T_A_B: np.ndarray, T_B_C: np.ndarray) -> np.ndarray:
        """
        Compose two transforms: ``T_A_C = T_A_B @ T_B_C``.
        """
        return np.asarray(T_A_B, dtype=float) @ np.asarray(T_B_C, dtype=float)

    @staticmethod
    def invert_transform(T: np.ndarray) -> np.ndarray:
        """
        Analytically invert a rigid 4x4 transform.

        Uses ``R.T`` and ``-R.T @ t`` instead of a full matrix inverse.
        """
        T         = np.asarray(T, dtype=float)
        R         = T[:3, :3]
        t         = T[:3,  3]
        T_inv     = np.eye(4, dtype=float)
        T_inv[:3, :3] =  R.T
        T_inv[:3,  3] = -R.T @ t
        return T_inv

    @staticmethod
    def relative_transform(T_world_A: np.ndarray, T_world_B: np.ndarray) -> np.ndarray:
        """
        Compute frame B expressed in frame A: ``T_A_B = inv(T_world_A) @ T_world_B``.
        """
        return RobotKinematics.invert_transform(T_world_A) @ np.asarray(T_world_B, dtype=float)

    # ------------------------------------------------------------------
    # Point / Vector Transformation
    # ------------------------------------------------------------------

    @staticmethod
    def apply_transform(T: np.ndarray, point: np.ndarray) -> np.ndarray:
        """
        Apply a 4x4 transform to a 3D point and return ``[x, y, z]``.
        """
        p = np.asarray(point, dtype=float).ravel()
        if p.shape[0] != 3:
            raise ValueError(f"Expected a 3D point, got shape {p.shape}.")
        return (np.asarray(T, dtype=float) @ np.array([p[0], p[1], p[2], 1.0]))[:3]

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    @staticmethod
    def is_valid_rotation(R: np.ndarray, tol: float = 1e-6) -> bool:
        """
        Check whether ``R`` is a valid SO(3) matrix.

        Verifies orthogonality and determinant close to ``+1``.
        """
        R   = np.asarray(R, dtype=float)
        det = float(np.linalg.det(R))
        return np.allclose(R.T @ R, np.eye(3), atol=tol) and abs(det - 1.0) < tol
