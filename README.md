# Alphanumeric Viewer

![Alphanumeric Viewer](https://i.ibb.co/rwbJBj3/alpha1.png)
![Alphanumeric Viewer](https://i.ibb.co/VjQPK3H/alpha2.png)

# Subscribed topics

- **status** ([droneMsgsROS/droneStatus](https://bitbucket.org/joselusl/dronemsgsros/src/fa03af3fb09b943ea728d28683ff7b6032f74d66/msg/droneStatus.msg?at=master&fileviewer=file-view-default))   
One value of the following set {Unknown, Init, Landed, Flying, Hovering, Test, Taking off, Goto Fix Point, Landing, Looping}. This can be obtained from a field of the message Navdata of the AR Drone.

- **actuator_command/roll_pitch** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))           
Actuator command for the multirotor specifying roll and pitch (the rest of values for <x, y, z, yaw> are discarded).

- **actuator_command/altitude_rate_yaw_rate** ([geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html))           
Actuator command for the multirotor specifying rates for altitude (d_z) and yaw (d_yaw) (the rest of values for <d_x, d_y, d_yaw, d_pitch, d_roll> are discarded).

- **motion_reference/assumed_control_mode** ([aerostack_msgs/QuadrotorPidControllerMode](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/msg/QuadrotorPidControllerMode.msg))  
Current controller's control mode.

- **motion_reference/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))  
Pose reference for the controller.

- **motion_reference/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))  
Speed reference for the controller.

- **self_localization/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))      
Current pose of the vehicle

- **self_localization/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))      
Current speed of the vehicle

- **sensor_measurement/battery_state** ([sensor_msgs/BatteryState](http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html))   
Battery state.

- **sensor_measurement/altitude** ([geometry_msgs/PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html))   
Altitude respect to the ground.

- **sensor_measurement/sea_level_altitude** ([geometry_msgs/PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html))   
Sea level altitude.

- **sensor_measurement/temperature** ([sensor_msgs/Temperature](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Temperature.html))   
Temperature reading.

- **sensor_measurement/imu** ([sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html))   
Inertial Measurement Unit data.

- **sensor_measurement/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))   
Direct speed estimated by odometry sensors.


---
# Contributors
**Code Maintainer:** Alberto Rodelgo  
**Author:** Alberto Rodelgo