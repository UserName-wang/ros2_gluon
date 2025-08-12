# ROS2 Gluon Robot Project

This repository contains the ROS2 packages for the INNFOS Gluon robotic arm.

## Project Structure

```
ros2_gluon/
├── src/
│   ├── gluon_moveit_config/    # MoveIt configuration package
│   └── gluon_py/              # Main robot description and control package
└── README.md                  # This file
```

## Packages

### gluon_py
This package contains:
- URDF model of the Gluon robot
- Mesh files for visualization
- Launch files for displaying the robot
- Configuration files for controllers
- ROS2 node implementations for robot control

### gluon_moveit_config
This package contains the MoveIt configuration for the Gluon robot, generated using the MoveIt Setup Assistant.

## Setup Instructions

### Prerequisites
- ROS2 (Foxy, Galactic, Humble, or Iron)
- MoveIt2
- ros2_control
- ros2_controllers

Install dependencies:
```bash
sudo apt install ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers
```

### Building the Workspace
```bash
cd ros2_gluon
colcon build
source install/setup.bash
```

### Running the Robot Visualization
```bash
ros2 launch gluon_py display.launch.py
```

## Using MoveIt Setup Assistant

The MoveIt Setup Assistant is a tool for configuring MoveIt packages for your robot. To use it with the Gluon robot:

### Launch MoveIt Setup Assistant
```bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```

This will open the graphical interface for configuring MoveIt.

### Steps in MoveIt Setup Assistant:
1. Click "Create New MoveIt Configuration Package"
2. Browse to select the URDF file:
   - Path: `src/gluon_py/urdf/gluon.urdf` or `src/gluon_py/urdf/gluon.urdf.xacro`
3. Generate the configuration files
4. Configure planning groups, robot poses, end effectors, and passive joints
5. Generate the MoveIt configuration package

### Updating MoveIt Configuration
After making changes with the MoveIt Setup Assistant:
```bash
# Rebuild the packages
colcon build
source install/setup.bash

# Launch MoveIt demo
ros2 launch gluon_moveit_config demo.launch.py
```

## Launch Files

### Display Robot
```bash
ros2 launch gluon_py display.launch.py
```

### MoveIt Demo
```bash
ros2 launch gluon_moveit_config demo.launch.py
```

## Directory Structure Details

### gluon_py Package
```
gluon_py/
├── config/           # Configuration files
│   ├── gluon.rviz   # RViz configuration
│   ├── gluon_controllers.yaml  # Controller configurations
│   └── joint_names_gluon.yaml  # Joint names mapping
├── gluon_py/         # Python package
│   └── __init__.py
├── launch/           # Launch files
│   └── display.launch.py
├── meshes/           # STL files for robot links
├── test/             # Test files
├── urdf/             # URDF model files
├── package.xml
└── setup.py
```

### gluon_moveit_config Package
```
gluon_moveit_config/
├── config/           # MoveIt configuration files
├── launch/           # MoveIt launch files
├── CMakeLists.txt
└── package.xml
```

## Testing

Run tests with:
```bash
colcon test
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the Apache 2.0 License - see the LICENSE file for details.