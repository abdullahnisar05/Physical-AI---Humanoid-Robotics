---
sidebar_position: 7
---

# Glossary of Terms

This glossary provides definitions for key terms used throughout the Physical AI & Humanoid Robotics curriculum.

## A

**Action (in ROS 2)**: A communication pattern in ROS 2 for long-running tasks that provides feedback during execution and returns a result upon completion. Actions are ideal for tasks like navigation or manipulation that take time to complete.

**Actuator**: A mechanical device that converts energy (typically electrical) into physical motion. In humanoid robots, actuators control joint movement and enable manipulation capabilities.

**Adaptive Control**: A control system that adjusts its parameters in real-time based on changes in the system or environment to maintain optimal performance.

**Affordance**: A property of an object that indicates how it can be used. In robotics, affordances help robots understand how to interact with objects in their environment.

**AI (Artificial Intelligence)**: The simulation of human intelligence processes by machines, especially computer systems. In robotics, AI enables robots to perceive, reason, learn, and adapt to their environment.

**Algorithm**: A step-by-step procedure for solving a problem or completing a task, expressed in a finite amount of space and time with a well-defined formal language.

**Angular Velocity**: The rate of change of angular displacement; a vector quantity that describes the rotation of an object around an axis.

**Anthropomorphic**: Having human-like characteristics, particularly in appearance or behavior, often used to describe humanoid robots.

**Articulated Robot**: A robot with rotary joints (e.g., an arm), which can have four or more degrees of freedom to perform complex tasks.

**Autonomous**: Capable of acting independently without human intervention. An autonomous robot can perceive its environment and make decisions without continuous human guidance.

## B

**Balance Control**: The ability of a humanoid robot to maintain its center of mass within its support polygon to prevent falling. Critical for bipedal locomotion.

**Base of Support**: The area beneath an object or person that includes every point of contact with the supporting surface. For humanoid robots, this is the area between the feet.

**Behavior Tree**: A hierarchical model used in robotics and AI to structure the execution of tasks. It provides a way to organize complex behaviors in a modular and reusable manner.

**Bipedal**: Having two feet or legs; walking on two legs. Bipedal locomotion is the act of walking on two legs, as humans do.

**Body Schema**: A representation of the body's configuration and position in space, used by robots to understand their own physical state.

**Bootstrapping**: A learning method where a system learns without explicit supervision by using its own experience and interactions with the environment.

## C

**Cartesian Coordinates**: A coordinate system that specifies each point uniquely in a plane by a pair of numerical coordinates, which are the signed distances to the point from two fixed perpendicular directed lines, measured in the same unit of length.

**Center of Mass (CoM)**: The point in a robot where all of its mass can be considered to be concentrated. Critical for balance and stability calculations in humanoid robots.

**Center of Pressure (CoP)**: The point where the total sum of a pressure field acts upon a body, causing a force to act through that point. Important for balance in humanoid robots.

**Cognitive Architecture**: An integrated system of computational structures that produce behavior in response to environmental stimuli, reflecting the underlying cognitive processes.

**Collaborative Robot (Cobot)**: A robot that can safely and effectively interact with human workers while performing simple, repetitive tasks.

**Collision Detection**: The computational problem of detecting the intersection of two or more objects, essential for safe robot operation.

**Computer Vision**: A field of artificial intelligence that trains computers to interpret and understand the visual world. In robotics, it enables robots to identify objects, people, and navigate environments.

**Configuration Space (C-Space)**: The space of all possible configurations of a robot, where each point represents a possible configuration of the robot's joints.

**Control Loop**: A process where a control variable is measured, compared to a desired value, and the difference used to adjust the control variable to reach the desired value.

**Convolutional Neural Network (CNN)**: A class of deep neural networks commonly applied to analyzing visual imagery. Widely used in robotics for object detection and scene understanding.

**Covariance**: A measure of how much two random variables change together. In robotics, often used to quantify uncertainty in sensor measurements.

## D

**Degrees of Freedom (DOF)**: The number of independent movements a mechanical system can make. For humanoid robots, each joint typically contributes one or more degrees of freedom.

**Deep Learning**: A subset of machine learning based on artificial neural networks with representation learning. Used extensively in robotics for perception and control tasks.

**Deep Neural Network (DNN)**: An artificial neural network with multiple layers between the input and output layers. Used in robotics for perception, control, and decision-making.

**Delta Robot**: A parallel robot with three arms connected to a common base, used for high-speed pick-and-place operations.

**Denavit-Hartenberg Parameters**: A method to define the coordinate frames on the links of a robot manipulator, used for kinematic analysis.

**Dexterous Manipulation**: The ability to perform complex manipulation tasks using fine motor control, similar to human hand manipulation.

**Digital Twin**: A virtual replica of a physical system that mirrors its characteristics, behaviors, and responses in real-time. Used for safe, cost-effective development and testing.

**Domain Randomization**: A technique in simulation where environment parameters (textures, lighting, colors) are randomized to improve the transfer of models from simulation to reality.

**Dynamics**: The branch of mechanics concerned with the study of forces and their effect on motion. In robotics, it deals with the relationship between forces acting on a robot and its motion.

## E

**Embodied AI**: Artificial intelligence that is integrated with a physical body that interacts with the real world. The physical embodiment influences and is influenced by the AI's behavior.

**End Effector**: The device at the end of a robot arm that interacts with the environment. For humanoid robots, this is typically a hand designed for grasping and manipulation.

**Episodic Memory**: Memory of autobiographical episodes or specific events that occurred at particular times and places. Relevant in long-term human-robot interaction.

**Error Ellipsoid**: A geometric representation of the uncertainty in a robot's position or orientation, often used in localization and mapping.

**Euclidean Distance**: The "ordinary" straight-line distance between two points in Euclidean space.

**Evolutionary Robotics**: A methodology that uses evolutionary computation to design robots, including their morphologies, controllers, and environments.

## F

**Feedback Control**: A control system that uses the difference between the desired and actual output to adjust the system's behavior.

**Field of Expertise**: The specific area or domain in which a robot is designed to operate effectively.

**Forward Kinematics**: The process of determining the position and orientation of the end effector based on the joint angles of a robot. The opposite of inverse kinematics.

**Frame of Reference**: A coordinate system used to represent and measure the position and orientation of objects in space. Essential for robot navigation and manipulation.

**Fuzzy Logic**: A form of many-valued logic that deals with reasoning that is approximate rather than fixed and exact, useful in robotics for handling uncertainty.

## G

**Gait**: The pattern of movement of the limbs of legged robots during locomotion. For humanoid robots, this includes walking, running, and other forms of bipedal movement.

**Gaussian Distribution**: A continuous probability distribution that is symmetric about the mean, showing that data near the mean are more frequent in occurrence than data far from the mean.

**Gazebo**: A 3D dynamic simulator with robust physics engine, high-quality graphics, and sensor simulation capabilities. Widely used in robotics for testing and development.

**General Artificial Intelligence (AGI)**: Hypothetical AI that matches or exceeds human intelligence across the full range of cognitive tasks.

**Generative Adversarial Network (GAN)**: A class of machine learning frameworks where two neural networks contest with each other. Used in robotics for synthetic data generation.

**Geometry**: In ROS, refers to the geometric shapes and transformations used in robot modeling, typically handled by the TF (Transform) library.

**Global Navigation Satellite System (GNSS)**: A satellite-based system that provides geospatial positioning with global coverage, used for outdoor robot navigation.

## H

**Hardware Acceleration**: The use of computer hardware specially made to execute specific tasks, used to speed up computations in robotics perception and control.

**Haptic Feedback**: The use of touch sensation to communicate with users, providing tactile information about robot interactions with the environment.

**Heuristic**: A technique designed for solving a problem more quickly when classic methods are too slow, or for finding an approximate solution when classic methods fail to find any exact solution.

**Homogeneous Transformation**: A 4x4 matrix that represents both rotation and translation in 3D space, used extensively in robotics for coordinate transformations.

**Human-Robot Interaction (HRI)**: The study of interactions between humans and robots, focusing on design, development, and evaluation of robots for human use.

**Humanoid Robot**: A robot with a body structure that mimics the human form, typically featuring a head, torso, two arms, and two legs.

## I

**IK (Inverse Kinematics)**: The process of determining the joint angles required to place the end effector at a desired position and orientation. The reverse of forward kinematics.

**Impedance Control**: A control strategy that regulates the interaction forces between a robot and its environment by controlling the robot's mechanical impedance.

**Inertial Measurement Unit (IMU)**: An electronic device that measures and reports a body's specific force, angular rate, and sometimes the magnetic field surrounding the body, used for navigation and balance.

**Inertial Navigation**: A navigation aid that uses a computer, motion sensors (accelerometers), and rotation sensors (gyroscopes) to continuously calculate by dead reckoning the position, orientation, and velocity of a moving object.

**Intelligent Agent**: A system that perceives its environment and takes actions that maximize its chance of achieving its goals.

**Isaac Sim**: NVIDIA's high-fidelity simulation environment built on the Omniverse platform, designed for robotics development and testing.

**Isaac ROS**: A collection of hardware-accelerated packages that bridge the gap between NVIDIA's GPU computing platform and ROS 2.

## J

**Jacobian Matrix**: A matrix of all first-order partial derivatives of a vector-valued function, used in robotics to relate joint velocities to end-effector velocities.

**Jerk**: The rate of change of acceleration; the third derivative of position with respect to time. Important in robotics for smooth motion planning.

**Joint**: A connection between two or more links in a robot that allows relative motion. The degrees of freedom of a robot are determined by its joints.

**Joint Space**: The space defined by the joint angles of a robot. Often contrasted with Cartesian space.

## K

**Kalman Filter**: An algorithm that uses a series of measurements observed over time, containing statistical noise and other inaccuracies, to produce estimates of unknown variables that tend to be more accurate than those based on a single measurement alone.

**Kinematics**: The branch of mechanics concerned with the motion of objects without reference to the forces that cause the motion. In robotics, it deals with the geometric aspects of motion.

**Kinesthetic Teaching**: A method of teaching robot motions by physically guiding the robot through the desired motion path.

## L

**Large Language Model (LLM)**: Advanced AI models trained on vast amounts of text data, capable of understanding and generating human-like text. Used in robotics for natural language interaction.

**Laser Scanner**: A device that measures distances using laser light, commonly used in robotics for navigation and mapping.

**Learning Rate**: A hyperparameter that controls how much to change the model in response to the estimated error each time the model weights are updated.

**Legged Locomotion**: The ability of robots with legs to move through their environment, including walking, running, and climbing.

**LiDAR**: Light Detection and Ranging. A remote sensing method that uses light in the form of a pulsed laser to measure distances. Commonly used in robotics for navigation and mapping.

**Linear Actuator**: An actuator that creates motion in a straight-line motion, as contrasted with the circular motion of a conventional electric motor.

**Localization**: The process of determining the robot's position and orientation within a known or unknown environment.

**Long Short-Term Memory (LSTM)**: A type of recurrent neural network architecture that can learn order dependence in sequence prediction problems, used in robotics for temporal understanding.

## M

**Machine Learning (ML)**: A subset of AI that provides systems the ability to automatically learn and improve from experience without being explicitly programmed.

**Manipulation**: The ability of a robot to physically interact with objects in its environment, typically through grasping, lifting, or moving objects.

**Mapping**: The process of creating a representation of an environment, typically including the locations of objects, obstacles, and landmarks.

**Middleware**: Software that provides common services and capabilities to applications beyond what's offered by the operating system. ROS 2 serves as middleware for robotics applications.

**Mobile Robot**: A robot that is capable of locomotion, as opposed to fixed robots like industrial arms.

**Model Predictive Control (MPC)**: An advanced control method that uses a model of the system to predict future behavior and optimize control actions.

**Monte Carlo Method**: A computational algorithm that relies on repeated random sampling to obtain numerical results, used in robotics for uncertainty quantification and path planning.

## N

**Navigation**: The ability of a robot to move from one location to another in an environment, often involving path planning and obstacle avoidance.

**Neural Network**: A computing system inspired by the biological neural networks that constitute animal brains. Used extensively in robotics for perception and control.

**Non-Holonomic Constraint**: A constraint on a robot's motion that cannot be integrated to give a constraint only on position. Common in wheeled robots.

## O

**Odometry**: The use of data from motion sensors to estimate change in robot position over time. Fundamental for robot navigation.

**Omnidirectional Drive**: A type of robot drive system that allows movement in any direction without changing the robot's orientation.

**Operational Space**: The space in which the robot's end effector operates, typically Cartesian space with position and orientation.

**Omniverse**: NVIDIA's platform for real-time collaboration and simulation, serving as the foundation for Isaac Sim.

## P

**Path Planning**: The computational process of determining a path from a start to a goal location, considering obstacles and other constraints.

**Perception**: The ability of a robot to interpret and understand its environment through sensors and processing algorithms.

**PID Controller**: A control loop feedback mechanism widely used in robotics and industrial control systems, using Proportional, Integral, and Derivative terms.

**Point Cloud**: A set of data points in space, representing the external surface of an object or environment. Commonly generated by LiDAR and stereo vision.

**Pose**: The position and orientation of a robot or object in space, typically described by 6 degrees of freedom (3 for position, 3 for orientation).

**Prompt Engineering**: The practice of crafting inputs to guide large language models to produce desired outputs, particularly important in robotics applications.

## R

**RANSAC**: Random Sample Consensus, an iterative method to estimate parameters of a mathematical model from a set of observed data that contains outliers, commonly used in computer vision.

**Reachability**: The ability of a robot to reach a particular point in space with its end effector.

**Real-time**: Systems that must respond to inputs within strict time constraints. Critical for robot control and safety systems.

**Reinforcement Learning**: A type of machine learning where an agent learns to make decisions by performing actions and receiving rewards or penalties.

**Robot Operating System (ROS)**: Flexible framework for writing robot software. It provides services designed for a heterogeneous computer cluster.

**Robotics Middleware**: Software layer that facilitates communication between different components of a robotic system, with ROS 2 being the current standard.

**ROS 2**: The second generation of the Robot Operating System, featuring improved security, real-time capabilities, and support for commercial development.

**Runtime**: The period during which a program is executing, as opposed to compile time. Important for understanding robot system performance.

## S

**Sampling-based Motion Planning**: Motion planning approaches that sample the configuration space to find collision-free paths, including algorithms like RRT and PRM.

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

**Torque**: The tendency of a force to cause or change the rotational motion of a body, important in robot joint control.

**Trajectory**: A path that includes timing information, describing not just where a robot should go but when it should be at each point.

**Trust Region**: A term used in optimization to denote the subset of the domain of the objective function that is trusted to be accurately modeled by a local approximation.

## U

**Uncertainty**: The presence of randomness or unknown information in robotics, which must be managed through probabilistic methods and robust control.

**Unified Robot Description Format (URDF)**: An XML format for representing a robot model, including its physical and visual properties, joints, and other components.

**Universal Scene Description (USD)**: NVIDIA's format for 3D computer graphics data interchange, used extensively in Isaac Sim.

**Upper Body**: The portion of a humanoid robot consisting of the torso, arms, and head, responsible for manipulation and interaction tasks.

## V

**Variable Impedance Control**: A control method that allows the robot to vary its mechanical impedance (resistance to motion) to adapt to different tasks and environments.

**Velocity**: The rate of change of displacement, a vector quantity that describes both the speed and direction of motion.

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

**Z-Motion**: Vertical movement along the Z-axis in a 3D coordinate system, important for humanoid robot balance and manipulation.