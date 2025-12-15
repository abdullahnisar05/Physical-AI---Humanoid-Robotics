---
sidebar_position: 3
---

# Practical Exercises

This section provides practical exercises from throughout the curriculum for hands-on practice and reinforcement of key concepts.

## Module 1: The Robotic Nervous System (ROS 2) Exercises

### Exercise 1.1: Basic Publisher-Subscriber System

**Objective**: Create a simple publisher-subscriber system to understand ROS 2 communication patterns.

**Instructions**:
1. Create a new ROS 2 package called `exercise_pkg`
2. Create a publisher node that publishes a counter value every second
3. Create a subscriber node that subscribes to the counter topic
4. The subscriber should log the received values to the console
5. Test the system by running both nodes simultaneously

**Expected Outcome**:
- Publisher node successfully publishes integer messages
- Subscriber node successfully receives and logs the messages
- System demonstrates basic ROS 2 communication

**Difficulty**: Beginner

### Exercise 1.2: Service-Based Calculator

**Objective**: Implement a service server and client to understand request-response communication.

**Instructions**:
1. Create a service definition file for a simple calculator (`AddTwoInts.srv`)
2. Implement a service server that can add two integers
3. Create a service client that sends requests to the server
4. Test the system with various input combinations
5. Handle edge cases like very large numbers or division by zero (if implementing more complex operations)

**Expected Outcome**:
- Service server responds correctly to client requests
- Client receives and processes responses properly
- Error handling for invalid inputs

**Difficulty**: Intermediate

### Exercise 1.3: Action Server for Navigation

**Objective**: Create an action server to handle long-running navigation tasks.

**Instructions**:
1. Define an action file for navigation (`Navigate.action`) with goal, result, and feedback
2. Implement an action server that simulates navigation to a goal position
3. Provide feedback during navigation execution
4. Create an action client that sends navigation goals
5. Test with multiple simultaneous navigation requests

**Expected Outcome**:
- Action server properly handles goal requests
- Feedback is provided during execution
- Result is returned upon completion
- Client can handle multiple concurrent actions

**Difficulty**: Intermediate

### Exercise 1.4: URDF Robot Modeling

**Objective**: Create a URDF model for a simple robot with multiple links and joints.

**Instructions**:
1. Design a simple robot with at least 3 links and 2 joints
2. Create a URDF file describing the robot's physical properties
3. Include visual and collision elements for each link
4. Add inertial properties for dynamic simulation
5. Visualize the robot in RViz2 using robot_state_publisher

**Expected Outcome**:
- Valid URDF file that parses without errors
- Robot displays correctly in RViz2
- Proper kinematic chain with joint relationships
- Realistic visual and collision geometries

**Difficulty**: Intermediate

### Exercise 1.5: Launch File Configuration

**Objective**: Create launch files to manage complex multi-node systems.

**Instructions**:
1. Create a launch file that starts the publisher and subscriber from Exercise 1.1
2. Add parameters to control the publishing frequency
3. Create a second launch file that starts the service server and client from Exercise 1.2
4. Include conditional startup of nodes based on launch arguments
5. Test both launch files to ensure proper node management

**Expected Outcome**:
- Launch files successfully start all required nodes
- Parameters are properly passed to nodes
- Conditional node startup works as expected
- Clean shutdown of all nodes

**Difficulty**: Advanced

## Module 2: The Digital Twin (Gazebo & Unity) Exercises

### Exercise 2.1: Gazebo Robot Simulation

**Objective**: Load a custom robot model into Gazebo and test basic simulation.

**Instructions**:
1. Take the URDF robot from Exercise 1.4 and convert it for Gazebo simulation
2. Add Gazebo-specific plugins for physics simulation
3. Create a simple world file with obstacles
4. Spawn the robot in the simulation
5. Test the robot's response to basic physics interactions

**Expected Outcome**:
- Robot loads successfully in Gazebo
- Physics simulation behaves realistically
- Robot responds appropriately to collisions and gravity
- Proper joint constraints and dynamics

**Difficulty**: Intermediate

### Exercise 2.2: Camera Sensor Integration

**Objective**: Add and configure camera sensors in Gazebo simulation.

**Instructions**:
1. Add a camera sensor to your robot model from Exercise 2.1
2. Configure the camera parameters (resolution, field of view, etc.)
3. Create a ROS 2 node to subscribe to the camera topic
4. Visualize the camera feed in RViz2
5. Test the camera in various lighting conditions and scenarios

**Expected Outcome**:
- Camera sensor publishes images at the expected rate
- Images are properly received and displayed
- Sensor parameters are configurable
- Realistic camera behavior in simulation

**Difficulty**: Intermediate

### Exercise 2.3: LiDAR Sensor Simulation

**Objective**: Integrate LiDAR sensors for navigation and mapping.

**Instructions**:
1. Add a LiDAR sensor to your robot model
2. Configure LiDAR parameters (range, resolution, noise model)
3. Implement a simple obstacle detection algorithm
4. Create a node that processes LiDAR data for navigation
5. Test obstacle detection in various simulated environments

**Expected Outcome**:
- LiDAR sensor publishes scan data correctly
- Obstacle detection algorithm works with simulated data
- Sensor data is properly integrated into navigation system
- Realistic sensor noise and limitations

**Difficulty**: Advanced

### Exercise 2.4: Unity Visualization Setup

**Objective**: Set up Unity for high-fidelity visualization of robot systems.

**Instructions**:
1. Install Unity and required robotics packages
2. Create a basic scene with your robot model
3. Set up communication between ROS 2 and Unity
4. Synchronize robot state between simulation and Unity visualization
5. Test real-time visualization with dynamic robot movements

**Expected Outcome**:
- Unity scene displays robot model correctly
- Robot movements are synchronized between ROS 2 and Unity
- Real-time visualization operates smoothly
- Communication channel functions reliably

**Difficulty**: Advanced

### Exercise 2.5: Multi-Sensor Fusion in Simulation

**Objective**: Combine data from multiple sensors for enhanced perception.

**Instructions**:
1. Integrate camera, LiDAR, and IMU data from previous exercises
2. Implement a sensor fusion algorithm (e.g., Extended Kalman Filter)
3. Create a unified perception system
4. Test the system with various sensor inputs
5. Evaluate the improvement in perception accuracy

**Expected Outcome**:
- Multiple sensors provide complementary data
- Fusion algorithm improves overall system accuracy
- Robust operation despite individual sensor limitations
- Quantified improvement in perception quality

**Difficulty**: Advanced

## Module 3: The AI-Robot Brain (NVIDIA Isaac™) Exercises

### Exercise 3.1: Isaac Sim Environment Setup

**Objective**: Install and configure Isaac Sim for humanoid robot simulation.

**Instructions**:
1. Install Isaac Sim via Omniverse Launcher
2. Set up the development environment with required dependencies
3. Load a basic humanoid robot model (e.g., A1, Atlas, or custom)
4. Configure the simulation environment with appropriate physics
5. Test basic robot control and simulation stability

**Expected Outcome**:
- Isaac Sim runs without errors
- Robot model loads and functions correctly
- Physics simulation behaves appropriately
- Basic control commands execute successfully

**Difficulty**: Intermediate

### Exercise 3.2: Synthetic Data Generation

**Objective**: Generate synthetic training data using Isaac Sim.

**Instructions**:
1. Set up a scene with various objects and lighting conditions
2. Configure domain randomization parameters
3. Implement a data collection pipeline
4. Generate labeled datasets for computer vision tasks
5. Evaluate the quality and diversity of generated data

**Expected Outcome**:
- High-quality synthetic datasets are generated
- Domain randomization produces diverse training samples
- Labels are correctly associated with synthetic images
- Data quality is suitable for training ML models

**Difficulty**: Advanced

### Exercise 3.3: Isaac ROS Perception Integration

**Objective**: Integrate Isaac ROS packages for accelerated perception.

**Instructions**:
1. Install Isaac ROS packages on your development system
2. Integrate Isaac ROS perception nodes with your robot
3. Configure hardware acceleration (GPU) for perception tasks
4. Test VSLAM, stereo vision, or DNN processing
5. Compare performance with CPU-only implementations

**Expected Outcome**:
- Isaac ROS packages integrate successfully
- Hardware acceleration improves performance significantly
- Perception algorithms run in real-time
- Performance gains are quantified

**Difficulty**: Advanced

### Exercise 3.4: Humanoid Navigation with Nav2

**Objective**: Configure Nav2 for humanoid robot navigation with balance considerations.

**Instructions**:
1. Adapt Nav2 configuration for humanoid robot kinematics
2. Configure footstep planners for bipedal locomotion
3. Integrate balance control with navigation system
4. Test navigation in various terrain conditions
5. Evaluate stability and navigation performance

**Expected Outcome**:
- Navigation system accounts for humanoid kinematics
- Balance is maintained during navigation
- Robot successfully navigates challenging terrain
- Safety measures prevent falls during navigation

**Difficulty**: Advanced

### Exercise 3.5: Sim-to-Real Transfer Experiment

**Objective**: Implement and test sim-to-real transfer techniques.

**Instructions**:
1. Train a perception or control model in simulation
2. Apply domain randomization techniques to improve transfer
3. Test the model on real-world data or robot
4. Analyze the sim-to-real gap and its causes
5. Implement techniques to reduce the performance drop

**Expected Outcome**:
- Model performs adequately in real-world scenarios
- Domain randomization reduces sim-to-real gap
- Analysis identifies key factors affecting transfer
- Transfer techniques improve real-world performance

**Difficulty**: Advanced

## Module 4: Vision-Language-Action (VLA) Exercises

### Exercise 4.1: Open-Source Whisper Integration

**Objective**: Integrate open-source speech recognition into your robot system.

**Instructions**:
1. Install and configure faster-whisper or similar open-source alternative
2. Integrate speech recognition with ROS 2 system
3. Implement voice activity detection to trigger recognition
4. Test recognition accuracy in various acoustic conditions
5. Add error handling for recognition failures

**Expected Outcome**:
- Speech recognition operates reliably
- Voice commands are accurately transcribed
- System handles various acoustic conditions
- Error recovery mechanisms function properly

**Difficulty**: Intermediate

### Exercise 4.2: LLM-Based Task Planning

**Objective**: Use large language models for generating robot action sequences.

**Instructions**:
1. Set up access to an LLM API (or local model like Llama)
2. Design prompts that generate appropriate ROS 2 action sequences
3. Implement a parser to convert LLM output to executable commands
4. Test with various natural language commands
5. Evaluate the accuracy and safety of generated plans

**Expected Outcome**:
- LLM generates appropriate action sequences for commands
- Parser correctly converts LLM output to robot commands
- Generated plans are safe and executable
- System handles ambiguous or complex commands appropriately

**Difficulty**: Advanced

### Exercise 4.3: Vision-Language Integration

**Objective**: Combine visual and language inputs for improved understanding.

**Instructions**:
1. Integrate camera input with language processing system
2. Implement visual question answering capabilities
3. Create a system that resolves ambiguous language with visual context
4. Test with commands that require visual scene understanding
5. Evaluate the improvement in command interpretation accuracy

**Expected Outcome**:
- System uses visual context to disambiguate language
- Visual question answering works correctly
- Combined inputs improve overall system performance
- Context-aware command interpretation is demonstrated

**Difficulty**: Advanced

### Exercise 4.4: VLA Pipeline Integration

**Objective**: Integrate vision, language, and action components into a complete pipeline.

**Instructions**:
1. Connect speech recognition, vision processing, and action planning
2. Implement a coordinator node that manages the complete pipeline
3. Test with complex commands requiring all three modalities
4. Implement error handling and fallback behaviors
5. Evaluate the complete system performance

**Expected Outcome**:
- Complete VLA pipeline operates cohesively
- All three modalities work together effectively
- System handles complex multi-modal commands
- Robust error handling and recovery mechanisms

**Difficulty**: Advanced

### Exercise 4.5: Human-Robot Interaction Scenario

**Objective**: Create a complete human-robot interaction scenario using VLA.

**Instructions**:
1. Design a practical scenario (e.g., object manipulation, navigation)
2. Implement the complete VLA pipeline for the scenario
3. Add natural interaction patterns and feedback mechanisms
4. Test with human operators providing natural language commands
5. Evaluate usability and effectiveness of the interaction

**Expected Outcome**:
- Natural and intuitive human-robot interaction
- Effective completion of the chosen scenario
- Good user experience and system responsiveness
- Demonstrated practical utility of VLA approach

**Difficulty**: Advanced

## Cross-Module Integration Exercises

### Exercise CM-1: End-to-End Robot System

**Objective**: Integrate components from all modules into a complete robotic system.

**Instructions**:
1. Combine ROS 2 architecture with simulation environment
2. Integrate AI-powered perception and navigation
3. Add VLA capabilities for natural interaction
4. Implement comprehensive safety and error handling
5. Test the complete system in both simulation and physical environments

**Expected Outcome**:
- Fully integrated robotic system with all capabilities
- Seamless interaction between all subsystems
- Robust operation across different environments
- Demonstrated practical application of the complete curriculum

**Difficulty**: Expert

### Exercise CM-2: Multi-Robot Coordination

**Objective**: Extend the system to coordinate multiple robots with shared VLA interface.

**Instructions**:
1. Deploy the complete system on multiple robots
2. Implement coordination protocols between robots
3. Enable natural language commands that coordinate multiple robots
4. Test scenarios requiring multi-robot cooperation
5. Evaluate the effectiveness of multi-robot coordination

**Expected Outcome**:
- Multiple robots operate in coordination
- Natural language commands can control multiple robots
- Coordination protocols function effectively
- Multi-robot scenarios execute successfully

**Difficulty**: Expert

## Assessment Rubrics

### Technical Proficiency Rubric
- **Excellent (A)**: Solution demonstrates deep understanding, innovative approach, and robust implementation
- **Good (B)**: Solution is correct and well-implemented with minor limitations
- **Adequate (C)**: Solution works but has notable limitations or inefficiencies
- **Needs Improvement (D)**: Solution has significant issues or doesn't fully meet requirements
- **Unsatisfactory (F)**: Solution fails to meet basic requirements

### Documentation Quality Rubric
- **Excellent (A)**: Clear, comprehensive, and professionally presented documentation
- **Good (B)**: Adequate documentation with minor gaps
- **Adequate (C)**: Basic documentation present but with significant gaps
- **Needs Improvement (D)**: Minimal documentation that doesn't explain key aspects
- **Unsatisfactory (F)**: Little to no documentation provided

## Submission Guidelines

### Code Submission
- Include all source code with appropriate comments
- Provide clear README with setup and usage instructions
- Include any required configuration files
- Document any dependencies and their versions

### Report Submission
- Executive summary of the exercise
- Technical approach and methodology
- Results and analysis
- Challenges encountered and solutions
- Lessons learned and future improvements

### Presentation Requirements
- Demonstrate the working system
- Explain key technical decisions
- Show results and performance metrics
- Discuss challenges and learning outcomes

## Resources and Support

### Additional Learning Resources
- ROS 2 Tutorials: Official ROS 2 documentation and tutorials
- Gazebo Documentation: Simulation environment guides
- Isaac Documentation: NVIDIA Isaac platform resources
- Research Papers: Relevant academic papers for advanced techniques

### Community Support
- Online forums and discussion boards
- Peer review and collaboration opportunities
- Office hours with instructors
- Technical support for hardware and software issues

## Next Steps

Complete these exercises progressively, building on concepts from earlier modules. Each exercise reinforces key concepts while introducing new challenges. The integration exercises at the end provide opportunities to apply everything you've learned in comprehensive, real-world scenarios.

Remember that these exercises are designed to be challenging but achievable with the knowledge gained from the curriculum. Don't hesitate to revisit earlier materials if you need to refresh concepts before tackling more advanced exercises.