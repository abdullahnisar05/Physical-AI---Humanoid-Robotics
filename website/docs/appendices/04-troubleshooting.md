---
sidebar_position: 3
---

# Troubleshooting Guide

This appendix provides solutions to common issues encountered when working with the Physical AI & Humanoid Robotics curriculum materials.

## General System Issues

### ROS 2 Installation and Setup

**Issue**: `ros2` command not found after ROS 2 Humble installation
- **Cause**: ROS 2 environment not sourced or installation incomplete
- **Solution**:
  1. Verify installation: `sudo apt list --installed | grep ros-humble`
  2. Source the setup script: `source /opt/ros/humble/setup.bash`
  3. Add to your bashrc: `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`
  4. Restart terminal or run `source ~/.bashrc`

**Issue**: Permission denied when trying to run ROS 2 commands
- **Cause**: Colcon build permissions or workspace ownership issues
- **Solution**:
  1. Check workspace ownership: `ls -la ~`
  2. Fix permissions: `sudo chown -R $USER:$USER ~/ros2_ws`
  3. Recreate workspace if needed: `mkdir -p ~/ros2_ws/src && cd ~/ros2_ws`

**Issue**: `ament_cmake` or `colcon` build failures
- **Cause**: Missing dependencies or incorrect package.xml configuration
- **Solution**:
  1. Install missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`
  2. Check package.xml for correct dependencies
  3. Clean build: `rm -rf build/ install/ log/ && colcon build`

### Python Environment Issues

**Issue**: Python package import errors (e.g., `rclpy`, `cv2`, etc.)
- **Cause**: Wrong Python environment or packages not installed
- **Solution**:
  1. Check Python version: `python3 --version` (should be 3.8-3.10 for ROS 2 Humble)
  2. Verify ROS 2 packages: `python3 -c "import rclpy; print('rclpy imported successfully')"`
  3. Install missing packages: `pip3 install package_name`
  4. Check Python path: `echo $PYTHONPATH`

**Issue**: Virtual environment conflicts with ROS 2
- **Cause**: Virtual environment disabling ROS 2 Python packages
- **Solution**:
  1. Deactivate virtual environment: `deactivate`
  2. Source ROS 2: `source /opt/ros/humble/setup.bash`
  3. Or create virtual env with system packages: `python3 -m venv myenv --system-site-packages`

## Simulation Environment Troubleshooting

### Gazebo Issues

**Issue**: Gazebo fails to start or crashes immediately
- **Cause**: Graphics driver issues or missing OpenGL support
- **Solution**:
  1. Check graphics drivers: `nvidia-smi` (for NVIDIA) or `glxinfo | grep "OpenGL renderer"`
  2. Install missing packages: `sudo apt install nvidia-prime mesa-utils`
  3. Run without GUI: `gz sim -s`
  4. Try software rendering: `export LIBGL_ALWAYS_SOFTWARE=1`

**Issue**: Robot falls through the ground or physics behave strangely
- **Cause**: Incorrect inertial properties or physics parameters
- **Solution**:
  1. Check URDF inertial tags: ensure mass, ixx, iyy, izz are properly set
  2. Verify physics parameters in SDF/world file
  3. Increase physics update rate in world file
  4. Check for mesh collisions intersecting with each other

**Issue**: Sensor data not publishing or incorrect values
- **Cause**: Sensor plugin configuration issues or topic mismatches
- **Solution**:
  1. Verify sensor plugin in URDF: check plugin name and parameters
  2. Check topic names: `ros2 topic list | grep sensor`
  3. Verify sensor plugin is loaded: `gz topic -l`
  4. Test sensor independently: `ros2 run rviz2 rviz2`

### Isaac Sim Issues

**Issue**: Isaac Sim fails to launch or shows black screen
- **Cause**: Graphics card incompatibility or Omniverse client issues
- **Solution**:
  1. Check system requirements: NVIDIA GPU with Turing architecture or newer
  2. Verify Omniverse installation: Launch via Omniverse Launcher
  3. Check GPU memory: `nvidia-smi` (ensure sufficient VRAM)
  4. Update graphics drivers to latest version

**Issue**: USD assets not loading or textures missing
- **Cause**: Asset path configuration or file permissions
- **Solution**:
  1. Check asset paths in Isaac Sim: Verify paths are correct
  2. Verify file permissions: `ls -la /path/to/assets`
  3. Check for asset download issues in Omniverse Connect
  4. Clear Omniverse cache: `~/.nvidia-omniverse/cache`

**Issue**: Isaac ROS bridges not connecting or publishing data
- **Cause**: Network configuration or Isaac ROS extension not enabled
- **Solution**:
  1. Verify Isaac ROS extensions are enabled in Isaac Sim Extensions menu
  2. Check ROS 2 network configuration: `echo $ROS_DOMAIN_ID`
  3. Verify Isaac ROS bridge node is running: `ros2 node list`
  4. Check Isaac Sim logs for connection errors

## Hardware Acceleration Issues

### NVIDIA GPU Acceleration

**Issue**: Isaac ROS packages fail to initialize or show CPU fallback warnings
- **Cause**: GPU not detected or CUDA configuration issues
- **Solution**:
  1. Verify GPU: `nvidia-smi` (should show active GPU)
  2. Check CUDA installation: `nvcc --version`
  3. Verify Isaac ROS packages: `dpkg -l | grep isaac-ros`
  4. Check GPU compute capability: Should be 6.0 or higher

**Issue**: High GPU memory usage causing crashes
- **Cause**: Large models or inefficient memory management
- **Solution**:
  1. Monitor GPU usage: `watch -n 1 nvidia-smi`
  2. Reduce batch sizes in DNN models if possible
  3. Clear GPU memory: `sudo nvidia-smi --gpu-reset -i 0` (if GPU frozen)
  4. Use smaller model variants when possible

### Audio and Perception Issues

**Issue**: Whisper integration fails or produces poor recognition
- **Cause**: Audio input issues or model configuration problems
- **Solution**:
  1. Test audio input: `arecord -l` and `arecord -d 5 test.wav`
  2. Check microphone permissions
  3. Verify faster-whisper installation: `python3 -c "import faster_whisper; print('OK')"`
  4. Try different Whisper model sizes for better accuracy

**Issue**: Camera topics show no data in Isaac ROS stereo or image segmentation nodes
- **Cause**: Camera configuration or Isaac ROS bridge issues
- **Solution**:
  1. Verify camera is publishing in Isaac Sim: `gz topic -e /camera/rgb/image`
  2. Check Isaac ROS bridge connection: `ros2 topic list`
  3. Verify camera parameters in Isaac Sim
  4. Test with Isaac Sim's built-in camera viewer

## Network and Communication Issues

### ROS 2 Communication

**Issue**: Nodes cannot communicate across different terminals or machines
- **Cause**: ROS domain ID mismatch or network configuration
- **Solution**:
  1. Check domain ID: `echo $ROS_DOMAIN_ID` (should match on all nodes)
  2. Set domain ID: `export ROS_DOMAIN_ID=0` (or preferred domain)
  3. For multi-machine: `export ROS_IP=<your_ip_address>`
  4. Check firewall settings blocking DDS communication

**Issue**: High latency or dropped messages in ROS 2 communication
- **Cause**: Network congestion or QoS configuration issues
- **Solution**:
  1. Check QoS settings: Use appropriate reliability and durability settings
  2. Monitor network usage: `iftop` or `nethogs`
  3. Increase queue sizes for high-frequency topics
  4. Consider using reliable QoS for critical topics

### Isaac Sim Networking

**Issue**: Isaac Sim cannot connect to ROS 2 network
- **Cause**: Extension configuration or firewall blocking
- **Solution**:
  1. Enable Isaac ROS bridge extension in Isaac Sim
  2. Check firewall settings for ports 8080, 3000, 55557
  3. Verify ROS 2 environment is sourced in the terminal running Isaac Sim
  4. Check Isaac Sim logs for network error messages

## Build and Compilation Issues

### Colcon Build Problems

**Issue**: Package fails to build with undefined reference errors
- **Cause**: Missing library links or incorrect CMakeLists.txt
- **Solution**:
  1. Check CMakeLists.txt for correct target_link_libraries
  2. Verify package.xml has correct dependencies
  3. Clean build: `rm -rf build/ install/ && colcon build --packages-select package_name`
  4. Check for circular dependencies between packages

**Issue**: C++ compilation errors related to ROS 2 dependencies
- **Cause**: Missing ament packages or incorrect compiler settings
- **Solution**:
  1. Verify ament dependencies: `rosdep check --from-paths . --ignore-src`
  2. Check C++ standard: Ensure CMAKE_CXX_STANDARD matches ROS 2 requirements
  3. Install development packages: `sudo apt install ros-dev-tools`
  4. Verify compiler: Should be GCC 9 or higher for ROS 2 Humble

### Isaac ROS Package Issues

**Issue**: Isaac ROS packages fail to install or show version conflicts
- **Cause**: Repository not enabled or dependency conflicts
- **Solution**:
  1. Add NVIDIA repository: Follow Isaac ROS installation guide
  2. Update package lists: `sudo apt update`
  3. Check for conflicts: `apt-cache policy ros-humble-isaac-ros-*`
  4. Install specific versions if needed: `sudo apt install ros-humble-isaac-ros-package=version`

## Performance Optimization

### System Performance

**Issue**: Slow simulation or high CPU/GPU usage
- **Cause**: Inefficient configuration or resource constraints
- **Solution**:
  1. Reduce physics update rate in Gazebo/Isaac Sim
  2. Limit maximum threads: `export OMP_NUM_THREADS=4`
  3. Disable unnecessary visual elements in simulation
  4. Monitor resource usage: `htop`, `nvidia-smi`

**Issue**: Robot control instability or jittery movements
- **Cause**: Control loop timing issues or physics parameters
- **Solution**:
  1. Check control loop frequency: Should match physics update rate
  2. Adjust PID controller parameters for stability
  3. Verify robot's inertial properties in URDF
  4. Reduce solver iterations in physics configuration if too stiff

### Memory Management

**Issue**: System runs out of memory during training or simulation
- **Cause**: Large datasets or inefficient memory management
- **Solution**:
  1. Monitor memory usage: `free -h` and `htop`
  2. Clear system caches: `sudo sysctl -w vm.drop_caches=3`
  3. Limit Python memory usage with garbage collection
  4. Use swap space if physical RAM insufficient

## Docusaurus Site Issues

### Build and Deployment

**Issue**: Docusaurus site fails to build locally
- **Cause**: Node.js version mismatch or missing dependencies
- **Solution**:
  1. Check Node.js version: `node --version` (should be 18.x for Docusaurus v3)
  2. Install/update Node.js: Download from nodejs.org
  3. Clear cache: `npm ci` (instead of `npm install`)
  4. Check package.json dependencies

**Issue**: GitHub Pages deployment fails
- **Cause**: Workflow configuration or permission issues
- **Solution**:
  1. Verify GitHub Actions workflow file is correct
  2. Check repository permissions for Actions
  3. Ensure correct branch settings in workflow
  4. Review Actions logs for specific error messages

### Content and Navigation

**Issue**: Links are broken or navigation doesn't work properly
- **Cause**: Incorrect relative paths or sidebar configuration
- **Solution**:
  1. Verify relative links: Should be relative to the docs directory
  2. Check sidebar configuration in `sidebars.js`
  3. Test locally: `npm run start` before deployment
  4. Use absolute paths for external links

## Common Error Messages and Solutions

### ROS 2 Specific Errors

**Error**: `Failed to contact master` or `Unable to register with master`
- **Solution**: Source ROS 2 environment in each terminal: `source /opt/ros/humble/setup.bash`

**Error**: `Could not find a package configuration file provided by "ament_cmake"`
- **Solution**: Install ament packages: `sudo apt install python3-ament-cmake`

**Error**: `ImportError: No module named 'setuptools'`
- **Solution**: Install Python setup tools: `pip3 install setuptools`

### Simulation Specific Errors

**Error**: `gzclient: error while loading shared libraries`
- **Solution**: Update graphics drivers and reinstall Gazebo: `sudo apt reinstall gz-harmonic`

**Error**: `Isaac Sim: Failed to initialize CUDA context`
- **Solution**: Check NVIDIA drivers: `nvidia-smi` and restart Isaac Sim

### Isaac ROS Specific Errors

**Error**: `Could not import Isaac ROS package`
- **Solution**: Verify installation: `dpkg -l | grep isaac-ros` and source setup

**Error**: `Extension failed to load: omni.isaac.ros2_bridge`
- **Solution**: Enable extension in Isaac Sim Extensions menu

## Debugging Strategies

### General Debugging Approach

1. **Reproduce the issue**: Ensure you can consistently reproduce the problem
2. **Check logs**: Look at ROS 2 logs, system logs, and application-specific logs
3. **Isolate the problem**: Narrow down which component is causing the issue
4. **Verify assumptions**: Check that all prerequisites are met
5. **Test incrementally**: Build and test components one at a time
6. **Consult documentation**: Check official documentation for similar issues
7. **Seek help**: Use community forums with detailed error descriptions

### Log Analysis

**ROS 2 Logs Location**: `~/.ros/log/` or specified in launch files
**Gazebo Logs**: Console output and `~/.gazebo/logs/`
**Isaac Sim Logs**: Via Omniverse Logger extension or console output
**System Logs**: `journalctl -u service_name` or `/var/log/syslog`

### Diagnostic Tools

**ROS 2 Diagnostic Commands**:
- `ros2 doctor`: Check ROS 2 system health
- `ros2 topic info <topic>`: Check topic status
- `ros2 node info <node>`: Check node status
- `ros2 lifecycle list`: Check lifecycle nodes

**System Diagnostic Commands**:
- `htop`: Monitor system resources
- `iotop`: Monitor disk I/O
- `iftop`: Monitor network traffic
- `nvidia-smi`: Monitor GPU status

## Preventive Measures

### Best Practices to Avoid Issues

1. **Environment Consistency**: Always source ROS 2 environment in new terminals
2. **Regular Updates**: Keep system and packages updated
3. **Backup Work**: Regularly backup important code and configurations
4. **Version Control**: Use Git to track changes and enable rollbacks
5. **Incremental Testing**: Test components individually before integration
6. **Documentation**: Keep notes on custom configurations and changes

### System Maintenance

- **Regular Cleanup**: Clean build directories and temporary files periodically
- **Disk Space**: Monitor available disk space, especially in home directory
- **Package Management**: Remove unused packages to free space and reduce conflicts
- **Driver Updates**: Keep graphics and system drivers updated for optimal performance

If you encounter an issue not covered in this guide, please check the official documentation for the specific tool (ROS 2, Gazebo, Isaac Sim) and community forums for additional support. You can also contribute to this guide by sharing solutions to issues you encounter.