---
sidebar_position: 2
---

# Installing and Setting Up ROS 2 Humble on Ubuntu 22.04

This chapter provides step-by-step instructions for installing ROS 2 Humble Hawksbill on Ubuntu 22.04, the recommended LTS distribution for this curriculum.

## Prerequisites

Before installing ROS 2, ensure your system meets the following requirements:

- Ubuntu 22.04 LTS (Jammy Jellyfish)
- At least 5GB of free disk space
- Internet connection for package downloads
- Administrative (sudo) access

## Setup Locale

Ensure your locale is set to UTF-8 to prevent issues with ROS 2:

```bash
locale  # Check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## Setup Sources

Add the ROS 2 repository to your system:

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the repository to your sources list:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## Install ROS 2

Update your package list and install ROS 2 Humble:

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

This installs the full desktop package which includes Gazebo, RViz, and other useful tools.

## Environment Setup

Add ROS 2 to your environment by sourcing the setup script:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Install colcon Build Tool

Install colcon, the recommended build tool for ROS 2:

```bash
sudo apt install python3-colcon-common-extensions
```

## Install Additional Tools

Install additional tools for development:

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

## Verification

Test your installation by running a simple ROS 2 command:

```bash
ros2 topic list
```

You should see no output (indicating no topics are currently active) rather than an error.

## Troubleshooting

If you encounter issues:

1. Verify your Ubuntu version with `lsb_release -a`
2. Check your locale settings with `locale`
3. Ensure the ROS repository is properly added
4. Confirm all required packages are installed

## Next Steps

With ROS 2 installed, you can now proceed to the next chapter to learn about core concepts including nodes, topics, and messages.