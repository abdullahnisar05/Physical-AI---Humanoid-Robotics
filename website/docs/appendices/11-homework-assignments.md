---
sidebar_position: 9
---

# Homework Assignments

This appendix contains homework assignments that reinforce the concepts covered in each module of the Physical AI & Humanoid Robotics curriculum.

## Module 1: The Robotic Nervous System (ROS 2) Assignments

### Assignment 1.1: ROS 2 Basics and Publisher-Subscriber Patterns

**Due Date**: End of Week 1
**Estimated Time**: 8-10 hours
**Difficulty**: Beginner to Intermediate

#### Objective
Understand ROS 2 communication patterns and implement a basic publisher-subscriber system.

#### Tasks
1. **T001** [P] Create a new ROS 2 workspace and package called `hw1_basics`
2. **T002** [P] Create a publisher node that publishes a counter value every 2 seconds to a topic called `counter_topic`
3. **T003** Create a subscriber node that subscribes to `counter_topic` and logs received values to console
4. **T004** Add a parameter to the publisher to control the publishing frequency
5. **T005** Create a launch file that starts both the publisher and subscriber nodes
6. **T006** Test the system by running the launch file and verifying communication
7. **T007** Document the implementation and explain the communication flow

#### Submission Requirements
- Complete source code with proper comments
- Launch file for the system
- README with setup and execution instructions
- Brief report explaining the publisher-subscriber pattern

#### Grading Criteria
- Functionality: 40% (nodes communicate correctly)
- Code Quality: 25% (well-structured, commented code)
- Documentation: 20% (clear instructions and explanations)
- Understanding: 15% (explanation of ROS 2 concepts)

### Assignment 1.2: Service and Action Implementation

**Due Date**: End of Week 2
**Estimated Time**: 10-12 hours
**Difficulty**: Intermediate

#### Objective
Implement service and action servers to understand request-response and long-running task patterns.

#### Tasks
1. **T008** [P] Create a custom service definition for a simple calculator (`AddThreeInts.srv`)
2. **T009** [P] Implement a service server that adds three integers with validation
3. **T010** Create a service client that sends requests to the server
4. **T011** Define an action file for a navigation task (`NavigateToPose.action`)
5. **T012** Implement an action server that simulates navigation with feedback
6. **T013** Create an action client that sends navigation goals
7. **T014** Test both service and action implementations with various inputs
8. **T015** Compare the use cases for services vs. actions in a written analysis

#### Submission Requirements
- Complete service and action implementation
- Custom message/service/action definitions
- Test scripts demonstrating functionality
- Analysis comparing services and actions

#### Grading Criteria
- Service Implementation: 25% (properly defined and functional)
- Action Implementation: 30% (includes feedback and result handling)
- Testing: 20% (comprehensive testing of both patterns)
- Analysis: 25% (thoughtful comparison of use cases)

### Assignment 1.3: URDF Robot Modeling

**Due Date**: End of Week 3
**Estimated Time**: 12-15 hours
**Difficulty**: Intermediate to Advanced

#### Objective
Create a complete URDF model of a simple robot with multiple links and joints.

#### Tasks
1. **T016** [P] Design a simple robot with at least 4 links and 3 joints
2. **T017** [P] Create a URDF file describing the robot's physical properties
3. **T018** Include visual and collision elements for each link with realistic shapes
4. **T019** Add proper inertial properties for dynamic simulation
5. **T020** Include joint limits and safety controllers
6. **T021** Visualize the robot in RViz2 using robot_state_publisher
7. **T022** Test the robot model with joint_state_publisher for animation
8. **T023** Document the design decisions and validate the URDF file

#### Submission Requirements
- Complete URDF file with all required elements
- RViz2 configuration file for visualization
- Documentation of design decisions
- Screenshots of the robot in RViz2

#### Grading Criteria
- URDF Validity: 30% (parses without errors, valid structure)
- Physical Accuracy: 25% (realistic dimensions and properties)
- Visualization: 25% (displays correctly in RViz2)
- Documentation: 20% (clear explanations of design choices)

## Module 2: The Digital Twin (Gazebo & Unity) Assignments

### Assignment 2.1: Gazebo Simulation Setup

**Due Date**: End of Week 4
**Estimated Time**: 12-15 hours
**Difficulty**: Intermediate

#### Objective
Integrate your URDF robot model with Gazebo simulation and test basic functionality.

#### Tasks
1. **T024** [P] Convert your URDF robot model for Gazebo compatibility
2. **T025** [P] Add Gazebo-specific plugins for physics simulation (diff_drive_controller)
3. **T026** Create a simple world file with basic obstacles and lighting
4. **T027** Configure physics parameters for realistic behavior
5. **T028** Spawn the robot in the simulation environment
6. **T029** Test basic movement and physics interactions
7. **T030** Create a launch file that starts Gazebo with your robot and world
8. **T031** Document the simulation setup process and challenges faced

#### Submission Requirements
- Modified URDF with Gazebo plugins
- World file with obstacles
- Launch file for simulation
- Documentation of setup process
- Video or screenshots of simulation

#### Grading Criteria
- Simulation Integration: 35% (robot loads and functions in Gazebo)
- Physics Configuration: 25% (realistic physics behavior)
- Launch System: 20% (proper launch file creation)
- Documentation: 20% (clear setup guide)

### Assignment 2.2: Sensor Integration in Simulation

**Due Date**: End of Week 5
**Estimated Time**: 15-18 hours
**Difficulty**: Advanced

#### Objective
Add and configure various sensors in Gazebo simulation and verify data publication.

#### Tasks
1. **T032** [P] Add a camera sensor to your robot model with configurable parameters
2. **T033** [P] Add a LiDAR sensor (3D or 2D) with realistic parameters
3. **T034** Include an IMU sensor for orientation and acceleration data
4. **T035** Create ROS 2 nodes to subscribe to sensor data topics
5. **T036** Process and visualize sensor data in RViz2
6. **T037** Test sensor functionality in various simulated environments
7. **T038** Evaluate sensor performance with different physics parameters
8. **T039** Analyze the impact of sensor noise on perception quality

#### Submission Requirements
- Updated URDF with sensor definitions
- Sensor processing nodes
- RViz2 configuration for sensor visualization
- Performance analysis report
- Test results in different environments

#### Grading Criteria
- Sensor Integration: 30% (sensors properly defined and publishing data)
- Data Processing: 25% (nodes correctly process sensor data)
- Visualization: 20% (data properly visualized in RViz2)
- Analysis: 25% (thoughtful evaluation of sensor performance)

### Assignment 2.3: Unity Visualization (Optional Challenge)

**Due Date**: End of Week 6
**Estimated Time**: 15-20 hours
**Difficulty**: Advanced (Optional)

#### Objective
Set up Unity for high-fidelity visualization of your robot system.

#### Tasks
1. **T040** [P] Install Unity and required robotics packages
2. **T041** [P] Create a Unity scene with your robot model
3. **T042** Set up communication bridge between ROS 2 and Unity
4. **T043** Synchronize robot state between Gazebo/ROS 2 and Unity
5. **T044** Implement real-time visualization of sensor data in Unity
6. **T045** Test the visualization with dynamic robot movements
7. **T046** Compare visual quality between Gazebo and Unity
8. **T047** Document the Unity integration process and challenges

#### Submission Requirements
- Unity project files
- ROS 2-Unity bridge implementation
- Documentation of the integration process
- Comparison analysis of Gazebo vs. Unity visualization

#### Grading Criteria
- Unity Integration: 40% (successful ROS 2-Unity communication)
- Visualization Quality: 30% (high-fidelity rendering)
- Synchronization: 20% (real-time state synchronization)
- Analysis: 10% (comparison of simulation platforms)

## Module 3: The AI-Robot Brain (NVIDIA Isaac™) Assignments

### Assignment 3.1: Isaac Sim Setup and Basic Integration

**Due Date**: End of Week 7
**Estimated Time**: 15-20 hours
**Difficulty**: Advanced

#### Objective
Set up Isaac Sim environment and integrate your robot model with Isaac's simulation capabilities.

#### Tasks
1. **T048** [P] Install Isaac Sim via Omniverse Launcher and verify setup
2. **T049** [P] Import your robot model into Isaac Sim using USD format
3. **T050** Configure Isaac Sim physics properties for realistic simulation
4. **T051** Set up Isaac ROS bridge for communication with ROS 2
5. **T052** Test basic robot control in Isaac Sim environment
6. **T053** Compare Isaac Sim physics with Gazebo physics behavior
7. **T054** Document the setup process and initial impressions
8. **T055** Evaluate advantages and disadvantages of Isaac Sim vs. Gazebo

#### Submission Requirements
- Documentation of Isaac Sim setup process
- Screenshots of robot in Isaac Sim
- Comparison analysis of Isaac Sim vs. Gazebo
- Video demonstration of basic robot control

#### Grading Criteria
- Isaac Sim Setup: 30% (successful installation and configuration)
- Robot Integration: 25% (robot loads and functions in Isaac Sim)
- Comparison Analysis: 25% (thoughtful evaluation of platforms)
- Documentation: 20% (clear setup guide)

### Assignment 3.2: Isaac ROS Perception Integration

**Due Date**: End of Week 8
**Estimated Time**: 18-22 hours
**Difficulty**: Advanced

#### Objective
Integrate Isaac ROS perception packages for hardware-accelerated AI processing.

#### Tasks
1. **T056** [P] Install Isaac ROS packages and verify GPU acceleration
2. **T057** [P] Integrate Isaac ROS stereo vision pipeline with your robot
3. **T058** Implement Isaac ROS object detection using DNN packages
4. **T059** Test VSLAM capabilities with your robot in Isaac Sim
5. **T060** Compare performance with traditional ROS 2 perception packages
6. **T061** Analyze the benefits of hardware acceleration for perception
7. **T062** Document performance metrics and optimization techniques
8. **T063** Evaluate sim-to-real transfer potential of Isaac ROS packages

#### Submission Requirements
- Isaac ROS integration code
- Performance comparison data
- Documentation of optimization techniques
- Analysis of sim-to-real transfer potential

#### Grading Criteria
- Isaac ROS Integration: 35% (successful package integration)
- Performance Analysis: 30% (comprehensive performance evaluation)
- Hardware Acceleration: 20% (demonstration of GPU benefits)
- Documentation: 15% (clear performance analysis)

### Assignment 3.3: Isaac Navigation and Humanoid Control

**Due Date**: End of Week 9
**Estimated Time**: 20-25 hours
**Difficulty**: Advanced

#### Objective
Configure Isaac navigation for humanoid robots with balance considerations.

#### Tasks
1. **T064** [P] Configure Nav2 for humanoid robot kinematics in Isaac Sim
2. **T065** [P] Implement footstep planning for bipedal locomotion
3. **T066** Integrate balance control with navigation system
4. **T067** Test navigation in various Isaac Sim environments
5. **T068** Evaluate stability and navigation performance metrics
6. **T069** Compare with traditional wheeled robot navigation
7. **T070** Document safety measures and failure recovery
8. **T071** Analyze challenges specific to humanoid navigation

#### Submission Requirements
- Nav2 configuration for humanoid robots
- Footstep planning implementation
- Navigation performance data
- Safety analysis documentation
- Comparison with wheeled navigation

#### Grading Criteria
- Navigation Configuration: 30% (proper Nav2 setup for humanoid)
- Balance Integration: 25% (balance control with navigation)
- Performance Evaluation: 25% (comprehensive testing and metrics)
- Safety Analysis: 20% (thoughtful safety considerations)

## Module 4: Vision-Language-Action (VLA) Assignments

### Assignment 4.1: Voice Recognition and Language Processing

**Due Date**: End of Week 10
**Estimated Time**: 18-22 hours
**Difficulty**: Advanced

#### Objective
Implement voice recognition and natural language processing for robot command interpretation.

#### Tasks
1. **T072** [P] Set up open-source Whisper alternative (faster-whisper) for voice recognition
2. **T073** [P] Integrate voice recognition with ROS 2 communication system
3. **T074** Implement language understanding pipeline for command parsing
4. **T075** Create safety validation layer for incoming voice commands
5. **T076** Test system with various natural language commands
6. **T077** Evaluate recognition accuracy in different acoustic conditions
7. **T078** Document error handling and recovery mechanisms
8. **T079** Analyze privacy considerations for voice processing

#### Submission Requirements
- Voice recognition integration
- Language processing pipeline
- Safety validation implementation
- Accuracy evaluation data
- Privacy analysis documentation

#### Grading Criteria
- Voice Recognition: 30% (accurate voice-to-text conversion)
- Language Processing: 25% (effective command parsing)
- Safety Implementation: 20% (validation of incoming commands)
- Evaluation: 25% (comprehensive testing and analysis)

### Assignment 4.2: Vision-Language Integration

**Due Date**: End of Week 11
**Estimated Time**: 20-25 hours
**Difficulty**: Advanced

#### Objective
Combine vision and language systems to create a multimodal understanding system.

#### Tasks
1. **T080** [P] Integrate camera input with language processing system
2. **T081** [P] Implement visual question answering capabilities
3. **T082** Create system that resolves ambiguous language with visual context
4. **T083** Test with commands requiring visual scene understanding
5. **T084** Evaluate improvement in command interpretation accuracy
6. **T085** Implement object detection for visual context
7. **T086** Create visual-language grounding mechanisms
8. **T087** Document multimodal integration challenges and solutions

#### Submission Requirements
- Vision-language integration code
- Visual question answering implementation
- Grounding mechanisms
- Accuracy improvement evaluation
- Challenge analysis documentation

#### Grading Criteria
- Multimodal Integration: 35% (effective combination of vision and language)
- Visual Grounding: 25% (linking language to visual elements)
- Performance Evaluation: 20% (quantified improvement in accuracy)
- Documentation: 20% (clear explanation of challenges and solutions)

### Assignment 4.3: Complete VLA System Integration

**Due Date**: End of Week 12
**Estimated Time**: 25-30 hours
**Difficulty**: Advanced

#### Objective
Integrate all components into a complete Vision-Language-Action system.

#### Tasks
1. **T088** [P] Connect all subsystems into complete VLA pipeline
2. **T089** [P] Implement system coordinator for managing component interactions
3. **T090** Create end-to-end testing scenarios
4. **T091** Implement comprehensive error handling and recovery
5. **T092** Test complete system with complex natural language commands
6. **T093** Evaluate system performance and responsiveness
7. **T094** Document system architecture and design decisions
8. **T095** Analyze limitations and future improvement opportunities

#### Submission Requirements
- Complete VLA system integration
- System coordinator implementation
- End-to-end testing results
- Performance evaluation
- System architecture documentation
- Future improvements analysis

#### Grading Criteria
- System Integration: 30% (all components work together seamlessly)
- Performance: 25% (comprehensive testing and evaluation)
- Error Handling: 20% (robust error handling and recovery)
- Documentation: 25% (complete system documentation)

## Cross-Module Integration Assignments

### Assignment CM-1: Capstone Project Planning

**Due Date**: End of Week 13
**Estimated Time**: 15-20 hours
**Difficulty**: Advanced

#### Objective
Plan and design the complete capstone project integrating all curriculum modules.

#### Tasks
1. **T096** [P] Define complete system architecture integrating all modules
2. **T097** [P] Create detailed project timeline and milestone schedule
3. **T098** Identify potential technical challenges and mitigation strategies
4. **T099** Plan resource requirements and testing approaches
5. **T100** Design safety measures and validation procedures
6. **T101** Create detailed component specifications
7. **T102** Identify external dependencies and integration points
8. **T103** Document risk assessment and contingency plans

#### Submission Requirements
- Complete system architecture design
- Project timeline and milestones
- Risk assessment and mitigation plan
- Component specifications
- Safety and validation plan

#### Grading Criteria
- Architecture Design: 30% (comprehensive system design)
- Planning: 25% (realistic timeline and milestones)
- Risk Assessment: 25% (thoughtful identification of challenges)
- Documentation: 20% (complete planning documentation)

### Assignment CM-2: Final Capstone Implementation

**Due Date**: End of Week 15
**Estimated Time**: 40-50 hours
**Difficulty**: Expert

#### Objective
Implement the complete integrated system demonstrating all curriculum concepts.

#### Tasks
1. **T104** [P] Implement complete ROS 2 architecture with all components
2. **T105** [P] Integrate simulation environment with real-world deployment plan
3. **T106** Deploy Isaac AI components for perception and navigation
4. **T107** Integrate Vision-Language-Action system for natural interaction
5. **T108** Test complete system in both simulated and real environments
6. **T109** Validate system safety and reliability measures
7. **T110** Document complete system and create user manual
8. **T111** Prepare final demonstration and evaluation materials

#### Submission Requirements
- Complete integrated system implementation
- System documentation and user manual
- Test results and validation data
- Safety validation documentation
- Final demonstration materials

#### Grading Criteria
- System Integration: 35% (all modules integrated successfully)
- Functionality: 25% (complete system functionality)
- Safety Validation: 20% (comprehensive safety measures)
- Documentation: 20% (complete system documentation)

## Assessment Rubrics

### Technical Implementation Rubric
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

### Problem-Solving Rubric
- **Excellent (A)**: Creative and efficient solutions with thorough analysis
- **Good (B)**: Solid solutions with good analytical approach
- **Adequate (C)**: Basic solutions with acceptable analysis
- **Needs Improvement (D)**: Solutions have gaps or insufficient analysis
- **Unsatisfactory (F)**: Solutions are inadequate or lack proper analysis

## Late Submission Policy

- **Less than 24 hours late**: 5% deduction
- **24-48 hours late**: 10% deduction
- **48-72 hours late**: 20% deduction
- **More than 72 hours late**: Assignment receives zero credit

## Collaboration Policy

- Discussions with peers are encouraged for conceptual understanding
- Code sharing is prohibited unless explicitly allowed
- All submitted work must be your own
- Cite any external resources or code references used
- Group assignments will be specified separately

## Academic Integrity

- All work must represent your own understanding and effort
- Cite any sources, libraries, or code snippets used
- Plagiarism will result in disciplinary action
- Consult with instructors if you're unsure about appropriate collaboration

## Support Resources

### Office Hours
- Regular instructor office hours for assignment help
- Peer mentoring sessions available
- Online discussion forums for technical questions

### Technical Support
- Troubleshooting guides for common issues
- Development environment setup assistance
- Hardware access for testing (when available)

### Learning Resources
- Additional tutorials for challenging concepts
- Sample implementations for reference
- Recommended readings for deeper understanding

## Submission Guidelines

### Code Submission
- Include all source code with appropriate comments
- Provide clear README with setup and usage instructions
- Include any required configuration files
- Document any dependencies and their versions

### Report Submission
- Executive summary of the assignment
- Technical approach and methodology
- Results and analysis
- Challenges encountered and solutions
- Lessons learned and future improvements

### Demonstration Requirements
- Video recording of system functionality (when applicable)
- Clear narration explaining key features
- Demonstration of all required functionality
- Professional quality and presentation

## Grading Scale

- **A (90-100%)**: Outstanding work demonstrating mastery of concepts
- **B (80-89%)**: Good work demonstrating solid understanding
- **C (70-79%)**: Adequate work meeting minimum requirements
- **D (60-69%)**: Below expectations with significant deficiencies
- **F (Below 60%)**: Unsatisfactory work failing to meet requirements

## Next Steps

Complete these assignments sequentially, building on concepts from earlier modules. Each assignment reinforces key concepts while introducing new challenges. The integration assignments at the end provide opportunities to apply everything you've learned in comprehensive, real-world scenarios.

Remember that these assignments are designed to be challenging but achievable with the knowledge gained from the curriculum. Don't hesitate to revisit earlier materials if you need to refresh concepts before tackling more advanced assignments.