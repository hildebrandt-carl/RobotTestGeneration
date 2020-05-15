# World Engine

This code is used to run the FlightGoggles simulator. This code also contains the instantiated quadrotor controllers. The structure is as follows:

* **flightcontroller**:  The instantiated quadrotor controllers
* **flightgoggles**: [FlightGoggles](https://flightgoggles.mit.edu) source code used to simulated the dynamics of the racing quadrotor.
* **imu_tools**: Tools used to filter the IMU data which is noisy.
* **simulator_com**: Connects the simulated dynamics with the Unity visualization engine.
* **universal_teleop**: Tools used to manually fly the quadrotor.

## Prerequisites
```
$ rosdep install --from-paths src --ignore-src -r -y
```

## Manually Testing Controllers

When Running the angle_test you can use this to control the angle
```
$ rostopic pub -r 1000 /uav/input/attitude geometry_msgs/Vector3 '{x: 0, y: 0.0, z: 1.57}'
```

When running the velocity_test you can use this command to test the velocity
```
$ rostopic pub -r 1000 /uav/input/velocity geometry_msgs/Vector3 '{x: 0, y: 0.0, z: 1}'
```

When running the position_test you can use this command to test the position
```
$ rostopic pub -r 1000 /uav/input/position geometry_msgs/Vector3 '{x: 0, y: 0.0, z: 0.0}'
```

## Quadrotor naming convention

The Controllers are named as follows:

* **speed-2_minsnap0**: Unstable waypoint controller
* **speed-1_minsnap0**: Stable waypoint controller
* **speed2_minsnap0**: Fixed velocity controller 2m/s
* **speed5_minsnap0**: Fixed velocity controller 5m/s
* **speed10_minsnap0**: Fixed velocity controller 10m/s
* **speed-1_minsnap-1**: Minsnap controller