# Alphanumeric Viewer

![Alphanumeric Viewer](https://i.ibb.co/vLhJpbn/alphanumeric.png)

# Subscribed topics

- **status** ([droneMsgsROS/droneStatus](https://bitbucket.org/joselusl/dronemsgsros/src/fa03af3fb09b943ea728d28683ff7b6032f74d66/msg/droneStatus.msg?at=master&fileviewer=file-view-default))   
One value of the following set {Unknown, Init, Landed, Flying, Hovering, Test, Taking off, Goto Fix Point, Landing, Looping}. This can be obtained from a field of the message Navdata of the AR Drone.

- **actuator_command/quadrotor_command** ([mav_msgs/RollPitchYawrateThrust](http://docs.ros.org/api/mav_msgs/html/msg/RollPitchYawrateThrust.html))           
Actuator command for the quadrotor specifying the derivative of the robot altitude (thrust), yaw rate, roll and pitch.

- **motion_reference/assumed_control_mode** ([aerostack_msgs/QuadrotorPidControllerMode](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/msg/QuadrotorPidControllerMode.msg))  
Current controller's control mode.

- **motion_reference/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))  
Pose reference for the controller.

- **motion_reference/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))  
Speed reference for the controller.

- **self_localization/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))      
Current pose of the vehicle

- **battery** ([droneMsgsROS/battery](https://bitbucket.org/joselusl/dronemsgsros/src/2b47c507de4b636562f313f07abf07991b2432c4/msg/battery.msg?at=master&fileviewer=file-view-default))   
Percentage of charge of battery (0 means no battery, 100 means full battery)

- **altitude** ([droneMsgsROS/droneAltitude](https://bitbucket.org/joselusl/dronemsgsros/src/2b47c507de4b636562f313f07abf07991b2432c4/msg/droneAltitude.msg?at=master&fileviewer=file-view-default))   
The altitude is represented by the magnitude and its derivative. Includes altitude, and altitude speed together with their variances.

- **ground_speed** ([droneMsgsROS/Vector2Stamped](https://bitbucket.org/joselusl/dronemsgsros/src/fa03af3fb09b943ea728d28683ff7b6032f74d66/msg/vector2Stamped.msg?at=master))   
Ground speed, i.e., horizontal speed of the vehicle relative to the ground.

---
# Contributors
**Code Maintainer:** Alberto Rodelgo  
**Author:** Alberto Rodelgo