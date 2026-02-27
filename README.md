# robot_kinematics

> ROS 2 / TF2 utility for homogeneous transforms, TF broadcasting, TF lookup, and geometry message conversions.

**Inside your node** pass `self`, shares the node's executor and clock:

```python
from robot_kinematics import RobotKinematics
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.kin = RobotKinematics(self)   # attached, no extra thread
```

> OR

**Standalone script** no node needed, spins its own thread:

```python
from robot_kinematics import RobotKinematics

kin = RobotKinematics()   # creates and spins internal node

kin.shutdown()   # always call at the end
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
