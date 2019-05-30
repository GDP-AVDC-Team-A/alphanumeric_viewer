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
#include <math.h> 

//ROS
#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "droneMsgsROS/battery.h"
#include "droneMsgsROS/droneAltitude.h"
#include "droneMsgsROS/vector2Stamped.h"
#include "droneMsgsROS/droneStatus.h"
#include "droneMsgsROS/dronePitchRollCmd.h"
#include "droneMsgsROS/droneDYawCmd.h"
#include "droneMsgsROS/droneDAltitudeCmd.h"
#include "droneMsgsROS/droneCommand.h"
#include "aerostack_msgs/SetControlMode.h"
#include "aerostack_msgs/QuadrotorPidControllerMode.h"
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/droneSpeeds.h"
#include "droneMsgsROS/dronePositionTrajectoryRefCommand.h"
#include "droneMsgsROS/dronePositionRefCommandStamped.h"
#include "droneMsgsROS/droneYawRefCommand.h"
#include "droneMsgsROS/droneAltitudeCmd.h"
#include "communication_definition.h"
#include "mav_msgs/RollPitchYawrateThrust.h" 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

//MsgsROS
droneMsgsROS::battery battery_msg;
droneMsgsROS::droneAltitude altitude_msg;
droneMsgsROS::vector2Stamped ground_speed_msg;
droneMsgsROS::droneStatus quadrotor_status_msg;
geometry_msgs::PoseStamped current_pose;;
geometry_msgs::PoseStamped current_position_reference;
geometry_msgs::TwistStamped current_speed_reference;
mav_msgs::RollPitchYawrateThrust quadrotor_command_msg;
int last_received_control_mode;

//Subscribers
ros::Subscriber self_localization_pose_sub;
ros::Subscriber battery_sub;
ros::Subscriber altitude_sub;
ros::Subscriber ground_speed_sub;
ros::Subscriber status_sub;
ros::Subscriber quadrotor_command_sub;
ros::Subscriber control_mode_sub;
ros::Subscriber position_reference_subscriber;
ros::Subscriber speed_reference_subscriber;
    
//Variables
std::stringstream interface_printout_stream;
std::stringstream pinterface_printout_stream;
std::string drone_id_namespace;

//Topics 
std::string battery_topic_name;
std::string altitude_topic_name;
std::string ground_speed_topic_name;
std::string quadrotor_command_topic_name;
std::string assumed_control_mode_topic_name;
std::string status_topic_name;
std::string self_localization_pose_topic_name;
std::string motion_reference_speed_topic_name;
std::string motion_reference_pose_topic_name;

//Print-Stream Functions
void printStream(float var);
void printStream(double var);
void printQuadrotorState();
void printControlMode();
void printBattery();
void printStaticMenu();

//Callback Functions
void selfLocalizationPoseCallback(const geometry_msgs::PoseStamped &msg) { current_pose  = (msg); }
void batteryCallback(const droneMsgsROS::battery::ConstPtr& msg){ battery_msg=*msg;}
void altitudeCallback(const droneMsgsROS::droneAltitude::ConstPtr& msg){altitude_msg=*msg;}
void groundSpeedCallback(const droneMsgsROS::vector2Stamped::ConstPtr& msg){ground_speed_msg=*msg;}
void droneStatusCallback(const droneMsgsROS::droneStatus::ConstPtr& msg){quadrotor_status_msg=*msg;}
void quadrotorCommandCallback(const mav_msgs::RollPitchYawrateThrust::ConstPtr& msg){quadrotor_command_msg=*msg;}
void positionRefsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {current_position_reference = (*msg);}
void speedRefsSubCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {current_speed_reference = (*msg);}
void controlModeSubCallback(const aerostack_msgs::QuadrotorPidControllerMode::ConstPtr &msg) { 
    if(msg->command <1 || msg->command >4 ){
        last_received_control_mode = aerostack_msgs::QuadrotorPidControllerMode::UNKNOWN;
    }else{
        last_received_control_mode = msg->command;    
    }
}
#endif 