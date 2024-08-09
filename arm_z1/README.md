# Demonstration of the Unitree Z1 Arm Integration with ROS2 Humble

## Connecting to the arm
Follow the steps provided in [The Z1 Documentation](https://dev-z1.unitree.com/brief/poweron.html)


TIP: if you have aceess to a GUI you can set a manual IP address instead


## Building the Z1 SDK

```bash
cd src/arm/arm_z1/z1_controller
mkdir build
cd build
cmake ..
make
```

## Building the arm package

```bash

colcon build --packages-select arm_z1

```

## Launching the ros2 node 

```bash

ros2 launch arm_z1 arm_launch.py

```

NOTE: The demo must be launched through the launch file as the arm control program must also be run