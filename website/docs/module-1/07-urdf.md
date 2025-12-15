---
sidebar_position: 7
---

# URDF: Modeling Humanoid Robots (Links, Joints, Transmission, Gazebo Plugins)

This chapter covers the Unified Robot Description Format (URDF) for modeling humanoid robots, including links, joints, transmission, and Gazebo plugins.

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML format used to describe robots in ROS. It defines the physical and visual properties of a robot, including its links, joints, and other components.

## Basic URDF Structure

A basic URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="joint_name" type="continuous">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>

  <link name="child_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Links

Links represent rigid bodies in the robot. Each link has:

- **Visual**: How the link appears in simulation and visualization
- **Collision**: How the link interacts with physics simulation
- **Inertial**: Physical properties for dynamics simulation

## Joints

Joints connect links and define how they can move relative to each other. Joint types include:

- `revolute`: Rotational joint with limits
- `continuous`: Rotational joint without limits
- `prismatic`: Linear sliding joint
- `fixed`: No movement (welded connection)
- `floating`: 6 DOF (not typically used in URDF)
- `planar`: Movement on a plane

## Creating a Simple Humanoid Robot

Let's create a simplified humanoid robot model with torso, head, arms, and legs:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.15"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0.1 0.1 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Right Arm (similar to left, mirrored) -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.07 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="15" velocity="1"/>
  </joint>

  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0" effort="15" velocity="1"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.35" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.35" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.008" ixy="0" ixz="0" iyy="0.008" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Leg -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.07 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="15" velocity="1"/>
  </joint>

  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0" effort="15" velocity="1"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.35" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.35" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.008" ixy="0" ixz="0" iyy="0.008" iyz="0" izz="0.001"/>
    </inertial>
  </link>
</robot>
```

## Adding Gazebo Plugins

To make the robot work in Gazebo, you need to add Gazebo-specific tags:

```xml
<gazebo>
  <plugin name="robot_state_publisher" filename="libgazebo_ros_robot_state_publisher.so">
    <robot_param>robot_description</robot_param>
    <update_rate>30</update_rate>
  </plugin>
</gazebo>

<!-- For each link that should be controlled -->
<gazebo reference="left_shoulder_joint">
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>
```

## Loading URDF in ROS 2

To load and visualize your URDF:

```bash
# Launch robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(xacro path/to/your/robot.urdf.xacro)'
```

## Xacro for Complex Models

For more complex robots, use Xacro (XML Macros) to simplify the URDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_humanoid">
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Define a macro for limbs -->
  <xacro:macro name="arm" params="side reflect">
    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="torso"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${reflect*0.2} 0 0.1" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    </joint>

    <!-- More arm components... -->
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:arm side="left" reflect="1" />
  <xacro:arm side="right" reflect="-1" />
</robot>
```

## Exercises

1. Create a URDF for a simple wheeled robot with two wheels
2. Add colors to different parts of the humanoid robot
3. Modify joint limits to make the robot's movement more realistic
4. Create a URDF that includes sensors (e.g., camera, IMU)

## Next Steps

With URDF knowledge, you can now create robot models for simulation. The next modules will show how to simulate these robots in Gazebo and control them using ROS 2.