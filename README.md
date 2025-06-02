# Simple Obstacle Avoidance Navigator

A ROS2 C++ package that implements autonomous obstacle avoidance for TurtleBot3 using LiDAR sensor data.

## 🚀 Features

- **Real-time obstacle detection** using LiDAR sensor fusion
- **Dynamic obstacle avoidance** with turn-away behavior
- **Modern C++17** implementation with smart pointers
- **Multi-node architecture** for modular design
- **Launch file integration** for easy deployment

## 🛠️ Technologies Used

- **ROS2 Humble** (C++)
- **Gazebo Classic** simulation
- **TurtleBot3** robot platform
- **LiDAR sensor processing**
- **Modern C++17** standards

## 📁 Package Structure

```
simple_navigator/
├── src/
│   ├── obstacle_detector.cpp    # LiDAR processing node
│   └── simple_navigator.cpp     # Navigation control node
├── include/simple_navigator/
│   ├── obstacle_detector.hpp
│   └── simple_navigator.hpp
├── launch/
│   └── navigate_launch.py       # Launch both nodes
├── CMakeLists.txt
└── package.xml
```

## 🏗️ How It Works

1. **Obstacle Detector Node**: Processes LiDAR `/scan` data to find closest obstacle
2. **Navigator Node**: Receives obstacle info and controls robot movement via `/cmd_vel`
3. **Behavior**: If obstacle detected within 1m → turn away, else → move forward

## 🚀 Quick Start

### Prerequisites
```bash
# Install TurtleBot3 packages
sudo apt install ros-humble-turtlebot3*
```

### Build & Run
```bash
# Clone and build
git clone <your-repo-url>
cd obstacle_avoid_ws
colcon build
source install/setup.bash

# Terminal 1: Launch navigation system
ros2 launch simple_navigator navigate_launch.py

# Terminal 2: Start TurtleBot3 simulation
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## 🎯 Key ROS2 Concepts Demonstrated

- **Node Communication**: Publisher/Subscriber pattern
- **Custom Message Processing**: LiDAR sensor data handling
- **Timer-based Control**: 10Hz navigation loop
- **Modern C++ Features**: Smart pointers, auto keyword, range-based loops
- **Package Management**: Proper CMake and package.xml configuration

## 📊 System Architecture

```
[LiDAR Sensor] → [Obstacle Detector] → [Navigator] → [Robot Movement]
     /scan           /obstacle_info        /cmd_vel
```

## 🔧 Debugging Commands

```bash
# Monitor obstacle detection
ros2 topic echo /obstacle_info

# Monitor robot commands
ros2 topic echo /cmd_vel

# Check node status
ros2 node list
ros2 node info /obstacle_detector
```

## 🎓 Learning Outcomes

- ROS2 multi-node architecture design
- Sensor data processing and fusion
- Real-time robotics control systems
- Modern C++ best practices in robotics
- Launch file configuration and deployment

## 🤖 Demo

The robot autonomously navigates while avoiding obstacles, demonstrating:
- Continuous forward movement in clear paths
- Dynamic turning when obstacles detected
- Real-time sensor-based decision making

---

"A personal project to practice and improve my ROS2 and C++ skills, focusing on autonomous navigation concepts."
