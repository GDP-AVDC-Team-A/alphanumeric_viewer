# Alphanumeric Viewer

# Brief

![Alphanumeric Viewer](https://i.ibb.co/dtnmQCv/alphanumeric-viewer.png)

# Subscribed topics

- **status** ([droneMsgsROS/droneStatus](https://bitbucket.org/joselusl/dronemsgsros/src/fa03af3fb09b943ea728d28683ff7b6032f74d66/msg/droneStatus.msg?at=master&fileviewer=file-view-default))   
One value of the following set {Unknown, Init, Landed, Flying, Hovering, Test, Taking off, Goto Fix Point, Landing, Looping}. This can be obtained from a field of the message Navdata of the AR Drone.


## Deprecated topics
- **command/dAltitude** ([droneMsgsROS/droneAltitudeCmd](https://bitbucket.org/joselusl/dronemsgsros/src/master/msg/droneAltitudeCmd.msg))  
Publishes altitude commands for controllers.

- **command/dYaw** ([droneMsgsROS/droneDYawCmd](https://bitbucket.org/joselusl/dronemsgsros/src/master/msg/droneDYawCmd.msg))  
Publishes yaw commands for controllers.

- **command/pitch_roll** ([droneMsgsROS/dronePitchRollCmd](https://bitbucket.org/joselusl/dronemsgsros/src/master/msg/dronePitchRollCmd.msg))  
Publishes pitch and roll commands for controllers.

- **droneTrajectoryController/ControlMode** ([droneMsgsROS/droneTrajectoryControllerControlMode](https://bitbucket.org/joselusl/dronemsgsros/src/c74c7bab2f60c2161dc3e7b548dde81f3702eede/msg/droneTrajectoryControllerControlMode.msg))   
Control mode of the trajectory controller

- **altitude** ([droneMsgsROS/droneAltitude](https://bitbucket.org/joselusl/dronemsgsros/src/2b47c507de4b636562f313f07abf07991b2432c4/msg/droneAltitude.msg?at=master&fileviewer=file-view-default))   
The altitude is represented by the magnitude and its derivative. Includes altitude, and altitude speed together with their variances.

- **ground_speed** ([droneMsgsROS/Vector2Stamped](https://bitbucket.org/joselusl/dronemsgsros/src/fa03af3fb09b943ea728d28683ff7b6032f74d66/msg/vector2Stamped.msg?at=master))   
Ground speed, i.e., horizontal speed of the vehicle relative to the ground.

- **battery** ([droneMsgsROS/battery](https://bitbucket.org/joselusl/dronemsgsros/src/2b47c507de4b636562f313f07abf07991b2432c4/msg/battery.msg?at=master&fileviewer=file-view-default))   
Percentage of charge of battery (0 means no battery, 100 means full battery)

- **EstimatedPose_droneGMR_wrt_GFF** ([droneMsgsROS/dronePose](https://bitbucket.org/joselusl/dronemsgsros/src/2b47c507de4b636562f313f07abf07991b2432c4/msg/dronePose.msg?at=master&fileviewer=file-view-default))   
Pose of the drone estimated with basic sensors (GMR: Ground Mobile Robotics, GFF: Global Fixed Frame)

- **trajectoryControllerPositionReferencesRebroadcast** ([droneMsgsROS/dronePositionRefCommandStamped](https://bitbucket.org/joselusl/dronemsgsros/src/master/msg/dronePositionRefCommandStamped.msg))  
Broadcast a trajectory reference.

---
# Contributors
**Code Maintainer:** Alberto Rodelgo  
**Author:** Alberto Rodelgo