/*!*******************************************************************************************
 *  \file       alphanumeric_viewer_process_main.cpp
 *  \brief      Alphanumeric Viewer implementation file.
 *  \details    This process shows data related to sensor measurements, self localization,
 *              actuator commands and motion references.
 *              It doesn't publish messages to any topic.
 *  \authors    Alberto Rodelgo.
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All rights reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

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