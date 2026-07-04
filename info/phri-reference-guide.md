# PHRI Robotics Reference Guide

> **Safety first:** Before commanding motion, inspect the robot state, begin from the current pose, use small movements, and make sure the workspace is clear. Stop motion immediately if behavior is unexpected.

## Configuration placeholders

Replace these values for your environment before running a command:

| Placeholder | Meaning |
| --- | --- |
| `10.163.18.200` | Default robot network address used by the commands below |
| `<WORKSPACE>` | Path to your ROS 2 workspace |

## Start the system

### Kinova driver

Start without MoveIt:

```bash
ros2 launch kortex_bringup gen3.launch.py robot_ip:=10.163.18.200 dof:=6 launch_rviz:=true gripper:=robotiq_2f_140 gripper_joint_name:=finger_joint
```

Or start with MoveIt:

```bash
ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=10.163.18.200 use_fake_hardware:=false
```

### Cameras

Azure Kinect:

```bash
ros2 launch azure_kinect_ros2_driver k4a_device_launch.py
```


Kinova arm camera:

```bash
ros2 launch kinova_vision kinova_vision.launch.py launch_depth:=false device:=10.163.18.200
```

### Supporting services

ROS bridge:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages:=0.0
```

Carriage and lift (Elmo):

```bash
ros2 launch elmo elmo_launch.py
```

## Inspect system state

Robot joint state:

```bash
ros2 topic echo /joint_states
```

Camera image stream:

```bash
ros2 topic echo /camera/color/image_raw
```

All available topics:

```bash
ros2 topic list
```

Carriage and lift positions:

```bash
ros2 topic echo /elmo/id1/carriage/position/get
ros2 topic echo /elmo/id1/lift/position/get
```

## Move the robot

### Joint trajectory controller

Publish a trajectory to the joint controller. Substitute positions with the current joint state plus only small, deliberate changes when testing.

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
  points: [{
    positions: [<J1>, <J2>, <J3>, <J4>, <J5>, <J6>],
    time_from_start: {sec: 10}
  }]
}" -1
```

### Twist controller

First activate the twist controller and deactivate the joint-trajectory controller:

```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [twist_controller],
  deactivate_controllers: [joint_trajectory_controller],
  strictness: 1,
  activate_asap: true
}"
```

Send a small Cartesian velocity command:

```bash
ros2 topic pub --once /twist_controller/commands geometry_msgs/msg/Twist "{
  linear: {x: 0.02, y: 0.0, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: 0.0}
}"
```

**Stop the twist command explicitly:**

```bash
ros2 topic pub --once /twist_controller/commands geometry_msgs/msg/Twist "{}"
```

## Control the gripper

Use the gripper action. A position of `0.0` opens and `0.8` closes the gripper.

```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.0, max_effort: 100.0}}"
```

## Control the carriage and lift

Set a carriage position:

```bash
ros2 topic pub --once /elmo/id1/carriage/position/set std_msgs/msg/Float32 "{data: <CARRIAGE_POSITION>}"
```

Set a lift position:

```bash
ros2 topic pub --once /elmo/id1/lift/position/set std_msgs/msg/Float32 "{data: <LIFT_POSITION>}"
```

Stop either axis:

```bash
ros2 topic pub --once /elmo/id1/carriage/stop std_msgs/msg/Empty "{}"
ros2 topic pub --once /elmo/id1/lift/stop std_msgs/msg/Empty "{}"
```

