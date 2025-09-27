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
colcon build --packages-select gluon_py gluon_moveit_config
source install/setup.bash
```

### Running the Robot Visualization
```bash
ros2 launch gluon_py display.launch.py
```

## Troubleshooting

### Missing Dependencies

If you encounter errors related to missing libraries when launching the MoveIt demo, install the following packages:

```bash
# Install urdf_parser_py for URDF parsing
sudo apt install ros-humble-urdfdom-py

# Install geometric_shapes library
sudo apt install ros-humble-geometric-shapes

# Install object recognition messages
sudo apt install ros-humble-object-recognition-msgs

# Install MoveIt visualization components
sudo apt install ros-humble-moveit-ros-visualization ros-humble-moveit-ros-move-group

# Install OMPL (Open Motion Planning Library)
sudo apt install ros-humble-ompl
```

### RViz Motion Planning Plugin Error

If you see an error in RViz related to the MotionPlanning display not loading, with a message like:
```
The class required for this display, 'moveit_rviz_plugin/MotionPlanning', could not be loaded.
```

This is typically caused by missing dependencies. After installing the packages listed above and rebuilding the workspace, the issue should be resolved.

### MoveIt Demo Launch Issues

If the `demo.launch.py` fails with plugin loading errors or segmentation faults:

1. Make sure all dependencies are installed (see above)
2. Rebuild the package:
   ```bash
   colcon build --packages-select gluon_moveit_config --symlink-install
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```
4. Try launching again:
   ```bash
   ros2 launch gluon_moveit_config demo.launch.py
   ```

### Common Error Messages and Solutions

1. **libgeometric_shapes.so.2.3.2: cannot open shared object file**
   - Solution: Install the geometric-shapes package:
     ```bash
     sudo apt install ros-humble-geometric-shapes
     ```

2. **libobject_recognition_msgs__rosidl_typesupport_cpp.so: cannot open shared object file**
   - Solution: Install the object recognition messages package:
     ```bash
     sudo apt install ros-humble-object-recognition-msgs
     ```

3. **Could not load library libmoveit_motion_planning_rviz_plugin.so**
   - Solution: Install MoveIt visualization components:
     ```bash
     sudo apt install ros-humble-moveit-ros-visualization
     ```

4. **Failed to load library ... libompl.so.18: cannot open shared object file**
   - Solution: Install the OMPL library:
     ```bash
     sudo apt install ros-humble-ompl
     ```

After installing any of these packages, rebuild your workspace and source the setup files before trying to launch the demo again.