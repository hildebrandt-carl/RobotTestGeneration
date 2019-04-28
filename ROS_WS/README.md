When Running the angle_test you can use this to control the angle

rostopic pub -r 1000 /uav/input/attitude geometry_msgs/Vector3 '{x: 0, y: 0.0, z: 1.57}'


When running the velocity_test you can use this command to test the velocity

rostopic pub -r 1000 /uav/input/velocity geometry_msgs/Vector3 '{x: 0, y: 0.0, z: 1}'


When running the position_test you can use this command to test the position

rostopic pub -r 1000 /uav/input/position geometry_msgs/Vector3 '{x: 0, y: 0.0, z: 0.0}'