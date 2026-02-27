# robot_kinematics

ROS 2 / TF2 utility for homogeneous transforms, TF broadcasting, TF lookup, and geometry message conversions.

## Modes

**Inside your node** pass `self`, shares the node's executor and clock:

```python
from robot_kinematics import RobotKinematics
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.kin = RobotKinematics(self)   # attached, no extra thread
```

**Standalone script** no node needed, spins its own thread:

```python
from robot_kinematics import RobotKinematics

kin = RobotKinematics()   # creates and spins internal node

kin.shutdown()   # always call at the end
```

## Broadcast

```python
kin.broadcast_transform("world", "base_link", T)                            # dynamic (default)
kin.broadcast_transform("world", "base_link", T, is_static=True)            # static, latched once
kin.broadcast_transforms("world", ["link1", "link2"], [T1, T2])             # batch dynamic
kin.broadcast_transforms("world", ["link1", "link2"], [T1, T2], is_static=True)  # batch static
```

## Lookup

```python
ts, T  = kin.lookup_transform("world", "tool0", timeout_seconds=1.0)   # returns (TransformStamped, 4x4)
ok     = kin.wait_for_transform("world", "tool0", timeout_seconds=5.0) # bool
exists = kin.frame_exists("base_link")                                  # bool
```

## Conversions

```python
T   = RobotKinematics.transform_msg_to_matrix(msg)              # Transform  -> 4x4
msg = RobotKinematics.matrix_to_transform_msg(T)                # 4x4 -> Transform
T   = RobotKinematics.pose_msg_to_matrix(msg)                   # Pose       -> 4x4
msg = RobotKinematics.matrix_to_pose_msg(T)                     # 4x4 -> Pose
ts  = kin.matrix_to_stamped_transform_msg("world", "tool0", T)  # 4x4 -> TransformStamped
```


## Matrix Ops

```python
T_A_C = RobotKinematics.compose_transforms(T_A_B, T_B_C)           # T_A_B @ T_B_C
T_inv  = RobotKinematics.invert_transform(T)                        # analytical Rᵀ, -Rᵀt
T_A_B  = RobotKinematics.relative_transform(T_world_A, T_world_B)  # inv(T_world_A) @ T_world_B
p_out  = RobotKinematics.apply_transform(T, [x, y, z])             # transform a 3D point
valid  = RobotKinematics.is_valid_rotation(R)                       # SO(3) check
```

## Examples

| | |
|---|---|
| [matrix_ros_broadcast_static.py](./matrix_ros_broadcast_static.py) | static frame |
| [matrix_ros_broadcast_dynamic.py](./matrix_ros_broadcast_dynamic.py) | dynamic frame |
| [matrix_ros_broadcast_mixed.py](./matrix_ros_broadcast_mixed.py) | mixed |
| [matrix_ros_broadcast_multiple.py](./matrix_ros_broadcast_multiple.py) | batch broadcast |
| [matrix_ros_lookup.py](./matrix_ros_lookup.py) | lookup |
| [matrix_np_chain.py](./matrix_np_chain.py) | kinematic chain |
| [matrix_np_points.py](./matrix_np_points.py) | transform points |
| [matrix_np_rotation_check.py](./matrix_np_rotation_check.py) | validate rotation |
| [matrix_mixed_message_conversions.py](./matrix_mixed_message_conversions.py) | matrix ↔ msg |
| [matrix_mixed_pose_goal.py](./matrix_mixed_pose_goal.py) | pose goal |