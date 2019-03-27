#ifndef ALPHANUMERIC_VIEWER_ROSMODULE_H
#define ALPHANUMERIC_VIEWER_ROSMODULE_H


//STD CONSOLE
#include <sstream>
#include <stdio.h>
#include <curses.h>
#include <iostream>
#include <string>

//ROS
#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "droneMsgsROS/battery.h"
#include "droneMsgsROS/droneAltitude.h"
#include "droneMsgsROS/vector2Stamped.h"
#include "droneMsgsROS/droneStatus.h"
#include "droneMsgsROS/dronePitchRollCmd.h"
#include "droneMsgsROS/droneDYawCmd.h"
#include "droneMsgsROS/droneDAltitudeCmd.h"
#include "droneMsgsROS/droneCommand.h"
#include "control/Controller_MidLevel_controlModes.h"
#include "droneMsgsROS/setControlMode.h"
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/droneSpeeds.h"
#include "droneMsgsROS/dronePositionTrajectoryRefCommand.h"
#include "droneMsgsROS/dronePositionRefCommandStamped.h"
#include "droneMsgsROS/droneYawRefCommand.h"
#include "droneMsgsROS/droneAltitudeCmd.h"
#include "communication_definition.h"


//Column size
#define DISPLAY_COLUMN_SIZE 19

//MsgsROS
droneMsgsROS::dronePitchRollCmd DronePitchRollCmdMsgs;
droneMsgsROS::droneDAltitudeCmd DroneDAltitudeCmdMsgs;
droneMsgsROS::droneDYawCmd DroneDYawCmdMsgs;
droneMsgsROS::battery BatteryMsgs;
droneMsgsROS::droneAltitude AltitudeMsgs;
droneMsgsROS::vector2Stamped GroundSpeedMsgs;
droneMsgsROS::droneStatus DroneStatusMsgs;
droneMsgsROS::dronePose last_drone_estimated_GMRwrtGFF_pose_msg_;
droneMsgsROS::dronePose current_drone_position_reference_;

Controller_MidLevel_controlMode::controlMode last_received_control_mode;

//Subscribers
ros::Subscriber drone_estimated_GMR_pose_subscriber;
ros::Subscriber BatterySubs;
ros::Subscriber AltitudeSubs;
ros::Subscriber GroundSpeedSubs;
ros::Subscriber DroneStatusSubs;
ros::Subscriber DronePitchRollCmdSubs;
ros::Subscriber DroneDAltitudeCmdSubs;
ros::Subscriber DroneDYawCmdSubs;
ros::Subscriber controlModeSub;
ros::Subscriber drone_position_reference_subscriber;
    
//Variables
std::stringstream interface_printout_stream;
std::stringstream *pinterface_printout_stream;
std::string drone_id_namespace;

//Print-Stream Functions
void printout_stream( std::stringstream *pinterface_printout_stream, int *lineCommands, int *columCommands);
std::stringstream *getOdometryStream();
std::stringstream *getDroneCommandsStream();
std::stringstream *getPositionEstimates_GMRwrtGFF_Stream();
std::stringstream *getDroneState();
std::stringstream *getControllerState();
std::stringstream *getPositionReferences_GMRwrtGFF_Stream();

//Callback Functions
void drone_estimated_GMR_pose_callback_function(const  droneMsgsROS::dronePose  &msg) { last_drone_estimated_GMRwrtGFF_pose_msg_  = (msg); }
void batteryCallback(const droneMsgsROS::battery::ConstPtr& msg){ BatteryMsgs=*msg;}
void altitudeCallback(const droneMsgsROS::droneAltitude::ConstPtr& msg){AltitudeMsgs=*msg;}
void groundSpeedCallback(const droneMsgsROS::vector2Stamped::ConstPtr& msg){GroundSpeedMsgs=*msg;}
void droneStatusCallback(const droneMsgsROS::droneStatus::ConstPtr& msg){DroneStatusMsgs=*msg;}
void dronePitchRollCmdCallback(const droneMsgsROS::dronePitchRollCmd::ConstPtr& msg){DronePitchRollCmdMsgs=*msg;}
void droneDAltitudeCmdCallback(const droneMsgsROS::droneDAltitudeCmd::ConstPtr& msg){DroneDAltitudeCmdMsgs=*msg;}
void droneDYawCmdCallback(const droneMsgsROS::droneDYawCmd::ConstPtr& msg){DroneDYawCmdMsgs=*msg;}
void droneCurrentPositionRefsSubCallback(const droneMsgsROS::dronePose::ConstPtr &msg) {current_drone_position_reference_ = (*msg);}
void controlModeSubCallback(const droneMsgsROS::droneTrajectoryControllerControlMode::ConstPtr &msg) {
    switch (msg->command) {
    case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
        last_received_control_mode = Controller_MidLevel_controlMode::TRAJECTORY_CONTROL;
        break;
    case Controller_MidLevel_controlMode::POSITION_CONTROL:
        last_received_control_mode = Controller_MidLevel_controlMode::POSITION_CONTROL;
        break;
    case Controller_MidLevel_controlMode::SPEED_CONTROL:
        last_received_control_mode = Controller_MidLevel_controlMode::SPEED_CONTROL;
        break;
    default:
        last_received_control_mode = Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE;
        break;
    }
}

#endif 