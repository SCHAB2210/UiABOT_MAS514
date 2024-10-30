# UiAbot Mini Project

## Overview

This project is part of a series of assignments focusing on building and programming a mobile robot, the UiAbot Mini, using ROS 2. The robot utilizes an ESP32 for low-level control and communicates with a Raspberry Pi for high-level processing. The project involves motion control, sensor integration, and visualization in RViz.

Currently, two assignments have been implemented:

- **Assignment 1**: Motion Control
- **Assignment 2**: Perception

Future assignments will extend the functionality of the robot, including further sensor integration, autonomous navigation, and more advanced control strategies.

---

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Assignment 1: Implementation of Motion Control](#assignment-1-implementation-of-motion-control)
- [Assignment 2: Implementation of Perception](#assignment-2-implementation-of-perception)
- [Future Work](#future-work)
- [Contributing](#contributing)
- [License](#license)

---

## Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/your-username/uiabot.git
    cd uiabot
    ```
2. Set up the ROS 2 workspace:
    ```bash
    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash
    ```
3. Install any additional dependencies as required by the assignments.

---

## Usage

1. **Running the robot**:
    - Use the `multi_node_launch.py` script in the `launch_package` to start all the necessary nodes for driving the robot and visualizing Lidar data:
    ```bash
    ros2 launch launch_package multi_node_launch.py
    ```

2. **RViz Visualization**:
    - Make sure to configure RViz to visualize the `/scan` topic and related TF frames as described in Assignment 2.

---

## Assignment 1: Implementation of Motion Control

The first assignment focuses on getting the UiAbot Mini moving using differential drive control. Here's the breakdown:

1. **Implement Differential Drive Control on the ESP32**:
    - The ESP32 reads input from an Xbox controller and controls the robot’s motors.
    
2. **Establish Communication Between ESP32 and Raspberry Pi 5**:
    - Serial communication is set up between the ESP32 and the Raspberry Pi, which runs a ROS 2 node to interpret the data.

3. **Establish WiFi Communication**:
    - A wireless connection is established between the Development PC (running Ubuntu) and the UiAbot Mini for remote control.

4. **Use Xbox Controller to Drive the Robot**:
    - The `teleop_twist_joy` package is used to map Xbox controller inputs to `cmd_vel` messages, allowing manual driving of the robot.

---

## Assignment 2: Implementation of Perception

The second assignment focuses on adding perception capabilities to the robot by integrating sensors such as Lidar, wheel encoders, and an IMU.

1. **Lidar Integration**:
    - The RpLiDAR is used to publish on the `/scan` topic. This data is visualized in RViz alongside the static TF between `base_link` and the Lidar frame (`laser`).

2. **Wheel Encoder Integration**:
    - The teleop node from Assignment 1 is updated to read wheel encoder values over serial communication from the ESP32. This data is published to `/joint_states`, and the dynamic TF between `base_link` and the left/right wheels is visualized in RViz.

3. **IMU Integration**:
    - A new node is added to read IMU (yaw, pitch, roll) values from a BNO055 sensor over I2C. The IMU data is published to the `/imu` topic, and both the static and dynamic TFs for the IMU orientation are visualized in RViz.

---

## Future Work

### Assignment 3: Autonomous Navigation
- Implement a basic autonomous navigation stack, allowing the robot to move towards predefined waypoints using the Lidar for obstacle detection.

### Assignment 4: Sensor Fusion
- Combine data from the Lidar, IMU, and encoders using a Kalman Filter for more accurate state estimation.

### Assignment 5: Advanced Control Strategies
- Implement path planning algorithms such as Dijkstra’s or A* and integrate them with the robot’s navigation stack.

---

## Contributing

Contributions are welcome! Please fork the repository and create a pull request if you have any suggestions or improvements.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
