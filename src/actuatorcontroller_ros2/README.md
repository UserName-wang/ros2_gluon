# ActuatorController ROS2 Package

This package provides ROS2 integration for the INNFOS Gluon robotic arm actuators, enabling control via MoveIt2 motion planning.

## Overview

The package contains:
1. A ROS2 node that interfaces with the physical robot actuators using the ActuatorController SDK
2. Integration with MoveIt2 for motion planning and execution
3. Support for reading and publishing joint states
4. Smart detection of real robot hardware with automatic switching between real and simulated modes

## Components

### Actuator Controller Node
The main node (`innfos_actuator`) initializes the ActuatorController SDK and provides:
- Interface to control the physical robot actuators
- Subscription to joint trajectory commands from MoveIt2
- Publication of current joint states for visualization and feedback
- Automatic detection of connected actuators with fallback to simulation mode

### Trajectory Subscriber
Handles trajectory commands from MoveIt2:
- Subscribes to `/gluon_arm_controller/joint_trajectory`
- Maps joint names to physical actuator IDs
- Sends position commands to actuators using the `setPosition` method
- Publishes joint states to `/joint_states` using the `getPosition` method
- Automatically switches between real hardware control and simulation based on actuator detection

## Dependencies

- ROS2 (Humble)
- MoveIt2
- ActuatorController SDK (included in package)

## Building

To build this package:

```bash
cd ~/study/ros/ros_gluon/ros2_gluon
colcon build --packages-select actuatorcontroller_ros2
source install/setup.bash
```

## Usage

### Running with MoveIt2

1. Launch the actuator controller with MoveIt2:

```bash
ros2 launch actuatorcontroller_ros2 real_robot.launch.py
```

This will start:
- The actuator controller node with smart hardware detection
- MoveIt2 planning components
- RViz2 for visualization

The system automatically detects whether real actuators are connected:
- If actuators are found: Full control of real hardware with visualization in RViz2
- If no actuators are found: Simulation mode with only RViz2 visualization

2. Use RViz2 to plan and execute motions:
   - In the MotionPlanning panel, set the planning group to "gluon_arm"
   - Switch to the "Planning" tab
   - Drag the end-effector marker to a desired position
   - Click "Plan" to generate a trajectory
   - Click "Execute" to run the trajectory on the real robot or in simulation

### Running the Actuator Controller Standalone

```bash
ros2 run actuatorcontroller_ros2 innfos_actuator
```

## MoveIt2 Integration

This package is specifically designed to work with MoveIt2 for the Gluon robotic arm. The integration works as follows:

1. MoveIt2 generates joint trajectories based on motion planning requests
2. These trajectories are published to `/gluon_arm_controller/joint_trajectory`
3. The trajectory subscriber in this package receives these trajectories
4. Each point in the trajectory is sent to the corresponding physical actuator (if connected)
5. Joint states are continuously published to `/joint_states` for visualization in RViz2

## Configuration

The trajectory subscriber automatically detects connected actuators and creates a mapping between joint names and actuator IDs. The current implementation assumes the actuators are connected in order:
- axis_joint_1 -> First detected actuator
- axis_joint_2 -> Second detected actuator
- axis_joint_3 -> Third detected actuator
- axis_joint_4 -> Fourth detected actuator
- axis_joint_5 -> Fifth detected actuator
- axis_joint_6 -> Sixth detected actuator

## Smart Hardware Detection

This package features smart hardware detection that allows it to work in both real and simulated environments:

1. On startup, the system checks for connected actuators using the ActuatorController SDK
2. If 6 or more actuators are found:
   - The system enters hardware control mode
   - Commands are sent to real actuators
   - Joint states are read from real actuators
3. If fewer than 6 actuators are found:
   - The system enters simulation mode
   - No commands are sent to physical devices
   - Joint states are simulated for visualization

This feature allows the same launch file and code to work in both development/testing scenarios (without hardware) and production scenarios (with real hardware).

## Topics

### Subscribed Topics
- `/gluon_arm_controller/joint_trajectory` - Joint trajectory commands from MoveIt2

### Published Topics
- `/joint_states` - Current joint positions for visualization and feedback

## Parameters

- Control parameters can be configured via the ROS2 parameter server

## API Methods Used

The implementation uses the following methods from the ActuatorController SDK:
- `getActuatorIdArray()` - Get list of connected actuator IDs
- `setPosition(id, position)` - Set the target position for an actuator
- `getPosition(id, false)` - Get the current position of an actuator (without refreshing)

## Troubleshooting

### Actuator Connection Issues
- Make sure the actuators are properly connected via USB
- Check that the ActuatorController SDK can detect the actuators
- Verify that you have proper permissions to access the USB devices

### Trajectory Execution Issues
- Ensure that the trajectory points have strictly increasing time_from_start values
- Check that the actuators are enabled before sending commands
- Verify that the joint limits in MoveIt2 match the physical limits of the robot

### Mapping Issues
- If actuators are not being mapped correctly, check that you have at least 6 actuators connected
- The current implementation assumes actuators are connected in joint order (1-6)

## Known Issues

- First build may fail, please retry a few times
- In home mode, actuators may show as current mode but operation is still effective
- Do not use this package in workspaces with other packages using the raw Ethernet Communication SDK as conflicts may occur

## Recent Changes

### Smart Hardware Detection Feature
- Added automatic detection of real robot hardware
- Implemented fallback to simulation mode when no actuators are connected
- Modified launch files to support both real and simulated operation
- Enhanced trajectory subscriber to work in both modes

### Launch File Improvements
- Simplified real_robot.launch.py to use gluon_moveit_config's demo.launch.py
- Improved modularity and maintainability of launch files
- Ensured consistent behavior between real and simulated modes

## License

TODO