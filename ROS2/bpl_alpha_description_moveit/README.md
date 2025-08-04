# MoveIt resources for testing Reach Robotics Alpha 5 arm

This package depends on *bpl_alpha_description* package and implements Gazebo simulation with MoveIt and ros2_control.

## Running simulation:

```python
ros2 launch bpl_alpha_description_moveit demo_gz.launch.py
```

## Testing controllers 
```python
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller 
```

## Editing the package:
- Manually modify *alpha_5.srdf* file

Or

- Run MoveIt setup assistant wizard

```python
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

⚠️ MoveIt wizard does not update *joint_limits.yaml* and *alpha_5.urdf.xacro* files, which have to be fixed manually.

🔍 **joint_limits example yaml file**
```python
# joint_limits.yaml allows the dynamics properties specified in the URDF to be overwritten or augmented as needed

# For beginners, we downscale velocity and acceleration limits.
# You can always specify higher scaling factors (<= 1.0) in your motion requests.  # Increase the values below to 1.0 to always move at maximum speed.
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1

# Specific joint properties can be changed with the keys [max_position, min_position, max_velocity, max_acceleration]
# Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]
joint_limits:
  axis_a:
    has_velocity_limits: true
    max_velocity: 4.0
    has_acceleration_limits: true
    max_acceleration: 20.0
  axis_b:
    has_velocity_limits: true
    max_velocity: 4.0
    has_acceleration_limits: true
    max_acceleration: 20.0
  axis_c:
    has_velocity_limits: true
    max_velocity: 4.0
    has_acceleration_limits: true
    max_acceleration: 20.0
  axis_d:
    has_velocity_limits: true
    max_velocity: 4.0
    has_acceleration_limits: true
    max_acceleration: 20.0
  axis_e:
    has_velocity_limits: true
    max_velocity: 4.0
    has_acceleration_limits: true
    max_acceleration: 20.0
  rs1_130_joint:
    has_velocity_limits: true
    max_velocity: 10.0
    has_acceleration_limits: true
    max_acceleration: 50.0
  rs1_139_joint:
    has_velocity_limits: true
    max_velocity: 10.0
    has_acceleration_limits: true
    max_acceleration: 50.0

```

🔍 **alpha_5.urdf.xacro file**
```python
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="alpha_5">
    <!-- <xacro:arg name="initial_positions_file" default="initial_positions.yaml" /> -->

    <!-- Import alpha_5 urdf file -->
    <xacro:include filename="$(find bpl_alpha_description)/urdf/alpha_5.urdf.xacro" />

    <!-- Import control_xacro -->
    <!-- <xacro:include filename="alpha_5.ros2_control.xacro" /> -->


    <!-- <xacro:alpha_5_ros2_control name="FakeSystem" initial_positions_file="$(arg initial_positions_file)"/> -->

</robot>
```
- *initial_positions.yaml* file is not necessary since joint states are published by *joint_state_broadcaster*
- Custom *ros2_control.xacro* is loaded directly from */bpl_alpha_description/urdf/alpha_5.urdf.xacro*
- This package uses a real hardware interface plugin (*gz_ros2_control*) that connects the robot's joints to Gazebo’s physics engine. Including the FakeSystem would conflict with the Gazebo control plugin.