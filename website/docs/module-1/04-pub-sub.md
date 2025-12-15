---
sidebar_position: 4
---

# Publishers and Subscribers in Python (rclpy) with Hands-On Example

This chapter provides a hands-on example of creating publisher and subscriber nodes in Python using the rclpy library, demonstrating the fundamental communication pattern in ROS 2.

## Understanding Publisher-Subscriber Pattern

The publisher-subscriber pattern is the most common communication method in ROS 2. Publishers send messages to topics, and subscribers receive messages from topics. This creates a decoupled communication system where publishers and subscribers don't need to know about each other.

## Creating a Workspace

First, create a workspace for your ROS 2 packages:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

## Creating a Package

Create a new package for our publisher-subscriber example:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_pubsub --dependencies rclpy std_msgs
```

## Creating the Publisher Node

Navigate to the package directory and create the publisher script:

```bash
cd ~/ros2_ws/src/py_pubsub/py_pubsub
```

Create a file named `publisher_member_function.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating the Subscriber Node

Create a file named `subscriber_member_function.py` in the same directory:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Setting up the Python Package

Update the `setup.py` file in the package root (`~/ros2_ws/src/py_pubsub`):

```python
from setuptools import find_packages, setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Python pubsub example',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
        ],
    },
)
```

## Building the Package

Go back to the workspace root and build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select py_pubsub
```

## Sourcing the Workspace

Source the workspace to make the new package available:

```bash
source install/setup.bash
```

## Running the Publisher and Subscriber

Open two terminal windows and source the workspace in both:

Terminal 1 (Publisher):
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_pubsub talker
```

Terminal 2 (Subscriber):
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_pubsub listener
```

You should see the publisher sending messages and the subscriber receiving them.

## Understanding the Code

- The publisher creates a timer that calls `timer_callback` every 0.5 seconds
- In the callback, it creates a String message and publishes it to the 'topic' topic
- The subscriber creates a subscription to the 'topic' topic
- When a message arrives, the `listener_callback` is called with the message

## Exercises

1. Modify the publisher to send a counter value instead of "Hello World"
2. Change the timer period to 1 second
3. Create a new message type that includes both a string and an integer
4. Add error handling to the publisher and subscriber

## Next Steps

Now that you understand the publisher-subscriber pattern, you can explore services and actions in the next chapter for request-response communication patterns.