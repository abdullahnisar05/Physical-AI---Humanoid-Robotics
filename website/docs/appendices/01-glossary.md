---
sidebar_position: 1
---

# Glossary of Terms

This appendix provides definitions for key terms used throughout the Physical AI & Humanoid Robotics curriculum.

## A

**Action (in ROS 2)**: A communication pattern in ROS 2 for long-running tasks that provides feedback during execution and returns a result upon completion. Actions are ideal for tasks like navigation or manipulation that take time to complete.

**Artificial Intelligence (AI)**: The simulation of human intelligence processes by machines, especially computer systems. In robotics, AI enables robots to perceive, reason, learn, and adapt to their environment.

**Autonomous**: Capable of acting independently without human intervention. An autonomous robot can perceive its environment and make decisions without continuous human guidance.

## B

**Balance Control**: The ability of a humanoid robot to maintain its center of mass within its support polygon to prevent falling. Critical for bipedal locomotion.

**Base of Support**: The area beneath an object or person that includes every point of contact with the supporting surface. For humanoid robots, this is the area between the feet.

**Behavior Tree**: A hierarchical model used in robotics and AI to structure the execution of tasks. It provides a way to organize complex behaviors in a modular and reusable manner.

## C

**Center of Mass (CoM)**: The point in a robot where all of its mass can be considered to be concentrated. Critical for balance and stability calculations in humanoid robots.

**Computer Vision**: A field of artificial intelligence that trains computers to interpret and understand the visual world. In robotics, it enables robots to identify objects, people, and navigate environments.

**Control Theory**: The interdisciplinary branch of engineering and mathematics that deals with the behavior of dynamical systems with inputs, and how their behavior is modified by feedback.

**Convolutional Neural Network (CNN)**: A class of deep neural networks commonly applied to analyzing visual imagery. Widely used in robotics for object detection and scene understanding.

## D

**Deep Learning**: A subset of machine learning based on artificial neural networks with representation learning. Used extensively in robotics for perception and control tasks.

**Degree of Freedom (DOF)**: The number of independent movements a mechanical system can make. For humanoid robots, each joint typically contributes one or more degrees of freedom.

**Digital Twin**: A virtual replica of a physical system that mirrors its characteristics, behaviors, and responses in real-time. Used for safe, cost-effective development and testing.

**Domain Randomization**: A technique in simulation where environment parameters (textures, lighting, colors) are randomized to improve the transfer of models from simulation to reality.

**Dynamic Movement Primitive (DMP)**: A method for learning and reproducing robot movements. Used in humanoid robotics for generating smooth, adaptive motions.

## E

**Embodied AI**: Artificial intelligence that is integrated with a physical body that interacts with the real world. The physical embodiment influences and is influenced by the AI's behavior.

**End Effector**: The device at the end of a robot arm that interacts with the environment. For humanoid robots, this is typically a hand designed for grasping and manipulation.

**Episodic Memory**: Memory of autobiographical episodes or specific events that occurred at particular times and places. Relevant in long-term human-robot interaction.

## F

**Forward Kinematics**: The process of determining the position and orientation of the end effector based on the joint angles of a robot. The opposite of inverse kinematics.

**Frame of Reference**: A coordinate system used to represent and measure the position and orientation of objects in space. Essential for robot navigation and manipulation.

## G

**Gazebo**: A 3D dynamic simulator with robust physics engine, high-quality graphics, and sensor simulation capabilities. Widely used in robotics for testing and development.

**General Artificial Intelligence (AGI)**: Hypothetical AI that matches or exceeds human intelligence across the full range of cognitive tasks.

**Generative Adversarial Network (GAN)**: A class of machine learning frameworks where two neural networks contest with each other. Used in robotics for synthetic data generation.

**Geometry**: In ROS, refers to the geometric shapes and transformations used in robot modeling, typically handled by the TF (Transform) library.

## H

**Hardware Acceleration**: The use of computer hardware specially made to execute specific tasks, used to speed up computations in robotics perception and control.

**Human-Robot Interaction (HRI)**: The study of interactions between humans and robots, focusing on design, development, and evaluation of robots for human use.

**Humanoid Robot**: A robot with a body structure that mimics the human form, typically featuring a head, torso, two arms, and two legs.

## I

**Inverse Kinematics**: The process of determining the joint angles required to place the end effector at a desired position and orientation. The reverse of forward kinematics.

**Isaac Sim**: NVIDIA's high-fidelity simulation environment built on the Omniverse platform, designed for robotics development and testing.

**Isaac ROS**: A collection of hardware-accelerated packages that bridge the gap between NVIDIA's GPU computing platform and ROS 2.

**Iteration**: A repetition of a process in robotics programming, often used in learning algorithms and control systems.

## J

**Joint**: A connection between two or more links in a robot that allows relative motion. The degrees of freedom of a robot are determined by its joints.

**Joint Space**: The space defined by the joint angles of a robot. Often contrasted with Cartesian space.

## K

**Kinematics**: The branch of mechanics concerned with the motion of objects without reference to the forces that cause the motion. In robotics, it deals with the geometric aspects of motion.

**Kinesthetic Teaching**: A method of teaching robot motions by physically guiding the robot through the desired motion path.

## L

**Large Language Model (LLM)**: Advanced AI models trained on vast amounts of text data, capable of understanding and generating human-like text. Used in robotics for natural language interaction.

**Learning Rate**: A hyperparameter that controls how much to change the model in response to the estimated error each time the model weights are updated.

**LiDAR**: Light Detection and Ranging. A remote sensing method that uses light in the form of a pulsed laser to measure distances. Commonly used in robotics for navigation and mapping.

**Localization**: The process of determining the robot's position and orientation within a known or unknown environment.

## M

**Machine Learning (ML)**: A subset of AI that provides systems the ability to automatically learn and improve from experience without being explicitly programmed.

**Manipulation**: The ability of a robot to physically interact with objects in its environment, typically through grasping, lifting, or moving objects.

**Middleware**: Software that provides common services and capabilities to applications beyond what's offered by the operating system. ROS 2 serves as middleware for robotics applications.

**Model Predictive Control (MPC)**: An advanced control method that uses a model of the system to predict future behavior and optimize control actions.

## N

**Navigation**: The ability of a robot to move from one location to another in an environment, often involving path planning and obstacle avoidance.

**Neural Network**: A computing system inspired by the biological neural networks that constitute animal brains. Used extensively in robotics for perception and control.

**Non-Holonomic Constraint**: A constraint on a robot's motion that cannot be integrated to give a constraint only on position. Common in wheeled robots.

## O

**Odometry**: The use of data from motion sensors to estimate change in robot position over time. Fundamental for robot navigation.

**Omniverse**: NVIDIA's platform for real-time collaboration and simulation, serving as the foundation for Isaac Sim.

**Operational Space**: The space in which the robot's end effector operates, typically Cartesian space with position and orientation.

## P

**Perception**: The ability of a robot to interpret and understand its environment through sensors and processing algorithms.

**Planning**: The process of determining a sequence of actions to achieve a goal. Includes path planning, motion planning, and task planning.

**Point Cloud**: A set of data points in space, representing the external surface of an object or environment. Commonly generated by LiDAR and stereo vision.

**Prompt Engineering**: The practice of crafting inputs to guide large language models to produce desired outputs, particularly important in robotics applications.

## R

**Real-time**: Systems that must respond to inputs within strict time constraints. Critical for robot control and safety systems.

**Reinforcement Learning**: A type of machine learning where an agent learns to make decisions by performing actions and receiving rewards or penalties.

**Robot Operating System (ROS)**: Flexible framework for writing robot software. It provides services designed for a heterogeneous computer cluster.

**Robotics Middleware**: Software layer that facilitates communication between different components of a robotic system, with ROS 2 being the current standard.

**ROS 2**: The second generation of the Robot Operating System, featuring improved security, real-time capabilities, and support for commercial development.

**Runtime**: The period during which a program is executing, as opposed to compile time. Important for understanding robot system performance.

## S

**Sensor Fusion**: The process of combining data from multiple sensors to achieve better accuracy and reliability than could be achieved by using a single sensor.

**Sim-to-Real Transfer**: The process of transferring knowledge, models, or behaviors learned in simulation to real-world robotic systems.

**Simulation**: The imitation of the operation of a real-world process or system over time, essential for robotics development and testing.

**SLAM (Simultaneous Localization and Mapping)**: The computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

**State Estimation**: The process of determining the internal state of a system from measurements, critical for robot control and navigation.

**Stereo Vision**: A technique that extracts 3D information from digital images taken by two or more cameras, used for depth perception in robotics.

**System Integration**: The process of bringing together the component subsystems into one system and ensuring that the subsystems function together as a unit.

## T

**Task Planning**: The process of decomposing high-level goals into sequences of primitive actions that a robot can execute.

**TF (Transform Library)**: A package in ROS that keeps track of multiple coordinate frames over time, essential for robot perception and navigation.

**Trajectory**: A path that includes timing information, describing not just where a robot should go but when it should be at each point.

## U

**Unified Robot Description Format (URDF)**: An XML format for representing a robot model, including its physical and visual properties, joints, and other components.

**Universal Scene Description (USD)**: NVIDIA's format for 3D computer graphics data interchange, used extensively in Isaac Sim.

**Uncanny Valley**: The hypothesis that human replicas that appear almost, but not exactly, like real human beings cause a feeling of eeriness and repulsion among human observers.

**Upper Body**: The portion of a humanoid robot consisting of the torso, arms, and head, responsible for manipulation and interaction tasks.

## V

**Variable Impedance Control**: A control method that allows the robot to vary its mechanical impedance (resistance to motion) to adapt to different tasks and environments.

**Vision-Language-Action (VLA)**: A paradigm combining computer vision, natural language processing, and robotic action planning in a unified framework.

**Virtual Reality (VR)**: A simulated experience that can be similar to or completely different from the real world, sometimes used in robotics development.

## W

**Waypoint**: A reference or marker in physical space used for navigation, typically represented as coordinates in a map.

**Whole-Body Control**: A control approach that considers the entire robot as a single system, optimizing for all tasks and constraints simultaneously.

**Workspace**: The volume in space that a robot can reach with its end effector.

## X

**Xacro**: An XML macro language for generating URDF files, allowing for more flexible and reusable robot descriptions.

## Y

**Yaw**: The rotation of a robot around its vertical axis, one of the three rotational degrees of freedom (roll, pitch, yaw).

## Z

**Zero Moment Point (ZMP)**: A concept used in robotics and biomechanics to maintain balance in walking robots by ensuring the net moment of the ground reaction forces is zero.

**Zone of proximal development**: A concept from educational psychology relevant to human-robot interaction, referring to the difference between what a learner can do without help and what they can achieve with guidance.