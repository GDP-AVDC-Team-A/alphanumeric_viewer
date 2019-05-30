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

#include "../include/alphanumeric_viewer_process_main.h"

int main(int argc, char **argv)
{
    //OUTPUT FORMAT
    interface_printout_stream << std::fixed << std::setprecision(2) << std::setfill('0'); //<< std::showpos

    //ROS 
    ros::init(argc, argv, MODULE_NAME_DRONE_CONSOLE_INTERFACE);
    ros::NodeHandle n("~");

    //Configuration
    n.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");

    ros::param::get("~battery_topic_name", battery_topic_name);
    if ( battery_topic_name.length() == 0)
    {
        battery_topic_name="battery";
    }
   ros::param::get("~altitude_topic_name", altitude_topic_name);
    if ( altitude_topic_name.length() == 0)
    {
        altitude_topic_name="altitude";
    }
   ros::param::get("~ground_speed_topic_name", ground_speed_topic_name);
    if ( ground_speed_topic_name.length() == 0)
    {
        ground_speed_topic_name="ground_speed";
    }
    ros::param::get("~quadrotor_command_topic_name", quadrotor_command_topic_name);
    if ( quadrotor_command_topic_name.length() == 0)
    {
        quadrotor_command_topic_name="actuator_command/quadrotor_command";
    }
   ros::param::get("~assumed_control_mode_topic_name", assumed_control_mode_topic_name);
    if ( assumed_control_mode_topic_name.length() == 0)
    {
        assumed_control_mode_topic_name="motion_reference/assumed_control_mode";
    }
    ros::param::get("~status_topic_name", status_topic_name);
    if ( status_topic_name.length() == 0)
    {
        status_topic_name="status";
    }

    ros::param::get("~self_localization_pose_topic_name", self_localization_pose_topic_name);
    if ( self_localization_pose_topic_name.length() == 0)
    {
        self_localization_pose_topic_name="self_localization/pose";
    }
    ros::param::get("~motion_reference_speed_topic_name", motion_reference_speed_topic_name);
    if ( motion_reference_speed_topic_name.length() == 0)
    {
        motion_reference_speed_topic_name="motion_reference/speed";
    }
    ros::param::get("~motion_reference_pose_topic_name", motion_reference_pose_topic_name);
    if ( motion_reference_pose_topic_name.length() == 0)
    {
        motion_reference_pose_topic_name="motion_reference/pose";
    }

    //Sensor measurements subscribers
    battery_sub=n.subscribe("/"+drone_id_namespace+"/"+battery_topic_name, 1, &batteryCallback);
    altitude_sub=n.subscribe("/"+drone_id_namespace+"/"+altitude_topic_name, 1, &altitudeCallback);
    ground_speed_sub=n.subscribe("/"+drone_id_namespace+"/"+ground_speed_topic_name, 1, &groundSpeedCallback);

    //Actuator commands subscribers
    quadrotor_command_sub = n.subscribe("/"+drone_id_namespace+"/"+quadrotor_command_topic_name, 1, &quadrotorCommandCallback);
    control_mode_sub=n.subscribe(std::string("/"+drone_id_namespace + "/" +assumed_control_mode_topic_name), 1, &controlModeSubCallback);

    //Self localization subscriber
    self_localization_pose_sub= n.subscribe("/"+drone_id_namespace+"/"+self_localization_pose_topic_name, 1, &selfLocalizationPoseCallback);
    status_sub=n.subscribe("/"+drone_id_namespace+"/"+status_topic_name, 1, &droneStatusCallback);
    
    //Motion references subscriber
    position_reference_subscriber= n.subscribe("/"+drone_id_namespace+"/"+motion_reference_pose_topic_name, 1, &positionRefsCallback);
    speed_reference_subscriber= n.subscribe("/"+drone_id_namespace+"/"+motion_reference_speed_topic_name, 1, &speedRefsSubCallback);
    
    //ncurses initialization (output text)
    initscr();
    curs_set(0);
    noecho();
    nodelay(stdscr, TRUE);
    erase(); refresh();

    printw("         ----ALPHANUMERIC VIEWER OF AERIAL ROBOTIC PARAMETERS----");

    //Rate
    ros::Rate loop_rate(8);

    //Loop
    while (ros::ok()) {
        
        //Read messages
        ros::spinOnce();

        //Left column
        int lineCommands=1, columCommands=0;   

            //SENSOR MEASUREMENTS
            pinterface_printout_stream = getOdometryStream();
            printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);

            //SELF LOCALIZATION
            pinterface_printout_stream = getPositionStream();
            printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);
            pinterface_printout_stream = getQuadrotorState();
            printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);            
        
        //Right column
        lineCommands=2, columCommands=2*DISPLAY_COLUMN_SIZE+8;

            //ACTUATOR COMMANDS
            pinterface_printout_stream = getQuadrotorCommandsStream();
            printout_stream(pinterface_printout_stream, &lineCommands, &columCommands); 

            //Control Mode
            pinterface_printout_stream = getControllerState();
            printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);

            //SPEED REFERENCES
            pinterface_printout_stream = getSpeedReferencesStream();
            printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);

           lineCommands=9, columCommands=1*DISPLAY_COLUMN_SIZE+4;
            //MOTION REFERENCES
            pinterface_printout_stream = getPositionReferencesStream();
            printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);

            
        //Refresh
        refresh();
        loop_rate.sleep();
    }

    //End ncurses
    endwin();
    return 0;
}

//Print streams
void printout_stream( std::stringstream *pinterface_printout_stream, int *lineCommands, int *columCommands) {
    std::string line;
    move((*lineCommands),(*columCommands));
    while( std::getline( *pinterface_printout_stream, line, '\n') ) {
        for (int i = line.size(); i < DISPLAY_COLUMN_SIZE; i++)
            line += " ";
        printw(line.c_str());
        move(++(*lineCommands),(*columCommands));
    }
}

//Sensor Measurements Stream
std::stringstream *getOdometryStream(){
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    interface_printout_stream << std::endl
        << "SENSOR MEASUREMENTS" << std::endl
        << "Battery charge:     " << std::fixed << std::setprecision(0) << battery_msg.batteryPercent <<  "   %%"   << std::endl << std::fixed << std::setprecision(2);
                    if (altitude_msg.altitude > -0.01){
                        interface_printout_stream << "Altitude (z):       " << std::setw(5) << std::internal << fabs(altitude_msg.altitude)       << " m  "  << std::endl;
                    }else{
                        interface_printout_stream << "Altitude (z):      " << std::setw(6) << std::internal << altitude_msg.altitude       << " m  "  << std::endl;
                    }

                    if (ground_speed_msg.vector.x > -0.01){
                        interface_printout_stream << "Ground speed (x):   " << std::setw(5) << std::internal << fabs(ground_speed_msg.vector.x)    << " m/sec  "  << std::endl;
                    }else{
                        interface_printout_stream << "Ground speed (x):  " << std::setw(6) << std::internal << ground_speed_msg.vector.x    << " m/sec  "  << std::endl;
                    }

                    if (ground_speed_msg.vector.y > -0.01){
                        interface_printout_stream << "Ground speed (y):   " << std::setw(5) << std::internal << fabs(ground_speed_msg.vector.y)    << " m/sec  " << std::endl;
                    }else{
                        interface_printout_stream << "Ground speed (y):  " << std::setw(6) << std::internal << ground_speed_msg.vector.y    << " m/sec  " << std::endl;
                    }   
        interface_printout_stream <<  std::endl <<  std::endl;
    return &interface_printout_stream;
}

//Actuator commands Stream
std::stringstream *getQuadrotorCommandsStream(){
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    interface_printout_stream
        << "ACTUATOR COMMANDS" << std::endl;

                    if (quadrotor_command_msg.thrust.z > -0.01){
                        interface_printout_stream << "Altitude rate:     " << std::setw(5) << std::internal << fabs(quadrotor_command_msg.thrust.z) <<  " m/sec  "<<std::endl;
                    }else{
                        interface_printout_stream << "Altitude rate:    " << std::setw(6) << std::internal << quadrotor_command_msg.thrust.z <<  " m/sec  "<<std::endl;
                    }

                    if (quadrotor_command_msg.yaw_rate > -0.01){
                        interface_printout_stream << "Yaw rate:          " << std::setw(5) << std::internal << fabs(quadrotor_command_msg.yaw_rate )           <<  " rad/sec  " <<std::endl;
                    }else{
                        interface_printout_stream << "Yaw rate:         " << std::setw(6) << std::internal << quadrotor_command_msg.yaw_rate            <<  " rad/sec  " <<std::endl;
                    }

                    if (quadrotor_command_msg.pitch > -0.01){
                        interface_printout_stream << "Pitch:             " << std::setw(5) << std::internal << fabs(quadrotor_command_msg.pitch)     <<  " rad  "  <<std::endl;
                    }else{
                        interface_printout_stream << "Pitch:            " << std::setw(6) << std::internal << quadrotor_command_msg.pitch     <<  " rad  "  <<std::endl;
                    }


                    if (quadrotor_command_msg.roll > -0.01){
                        interface_printout_stream << "Roll:              " << std::setw(5) << std::internal << fabs(quadrotor_command_msg.roll)      <<  " rad  "  <<std::endl; 
                    }else{
                        interface_printout_stream << "Roll:             " << std::setw(6) << std::internal << quadrotor_command_msg.roll      <<  " rad  "  <<std::endl; 
                    }
    return &interface_printout_stream;
}

//Self Localization Stream
std::stringstream *getPositionStream(){
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    interface_printout_stream
        << "SELF LOCALIZATION" << std::endl;

                    if (current_pose.pose.position.x > -0.01){
                        interface_printout_stream << "x:       " << std::setw(5) << std::internal << fabs(current_pose.pose.position.x) << " m  " <<std::endl;
                    }else{
                        interface_printout_stream << "x:      " << std::setw(6) << std::internal << current_pose.pose.position.x << " m  " <<std::endl;
                    }

                    if (current_pose.pose.position.y > -0.01){
                        interface_printout_stream  << "y:       " << std::setw(5) << std::internal << fabs(current_pose.pose.position.y) << " m  " << std::endl;
                    }else{
                        interface_printout_stream  << "y:      " << std::setw(6) << std::internal << current_pose.pose.position.y << " m  " << std::endl;
                    }

                    if (current_pose.pose.position.z > -0.01){
                        interface_printout_stream << "z:       " << std::setw(5) << std::internal << fabs(current_pose.pose.position.z) << " m  " <<  std::endl;
                    }else{
                        interface_printout_stream << "z:      " << std::setw(6) << std::internal << current_pose.pose.position.z << " m  " <<  std::endl;
                    }

                    tf2::Matrix3x3 m(tf2::Quaternion (current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w));
                    double r, p, yaw;
                    m.getRPY(r, p, yaw);
                    
                    if (std::isnan(yaw)) yaw = 0.0;
                    if (std::isnan(r)) r = 0.0;
                    if (std::isnan(p)) p = 0.0;

                    if (yaw > -0.01){
                        interface_printout_stream << "yaw:     " << std::setw(5) << std::internal << fabs(yaw) << " rad  " <<  std::endl; 
                    }else{
                        interface_printout_stream << "yaw:    " << std::setw(6) << std::internal << yaw << " rad  " <<  std::endl; 
                    }

                    if (p > -0.01){
                        interface_printout_stream << "pitch:   " << std::setw(5) << std::internal << fabs(p) << " rad  " << std::endl;
                    }else{
                        interface_printout_stream << "pitch:  " << std::setw(6) << std::internal << p << " rad    " << std::endl;
                    }


                    if (r > -0.01){
                        interface_printout_stream << "roll:    " << std::setw(5) << std::internal << fabs(r) << " rad  " << std::endl;
                    }else{
                        interface_printout_stream << "roll:   " << std::setw(6) << std::internal << r << " rad  " << std::endl;
                    }    
    return &interface_printout_stream;
}

//Control mode Stream
std::stringstream *getControllerState(){
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    interface_printout_stream
        << "Control mode:      ";
    switch (last_received_control_mode) {
    case aerostack_msgs::QuadrotorPidControllerMode::SPEED:
        interface_printout_stream << "SPEED        " << std::endl;
        break;
    case aerostack_msgs::QuadrotorPidControllerMode::POSE:
        interface_printout_stream << "POSITION     " << std::endl;
        break;
    case aerostack_msgs::QuadrotorPidControllerMode::GROUND_SPEED:
        interface_printout_stream << "GROUND SPEED " << std::endl;
        break;
    case aerostack_msgs::QuadrotorPidControllerMode::SPEED_3D:
        interface_printout_stream << "SPEED 3D     " << std::endl;
        break;        
     case aerostack_msgs::QuadrotorPidControllerMode::UNKNOWN:
    default:
        interface_printout_stream << "UNKNOWN      " << std::endl;
        break;
    }
    interface_printout_stream << std::endl;
    return &interface_printout_stream;
}

//Status Stream
std::stringstream *getQuadrotorState(){
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    interface_printout_stream
        << "Status:  ";
    switch (quadrotor_status_msg.status) {
        case droneMsgsROS::droneStatus::UNKNOWN:
        interface_printout_stream << "UNKNOWN   " << std::endl;
            break;
        case droneMsgsROS::droneStatus::INITED:
            interface_printout_stream << "INIT  " << std::endl;
            break;
        case droneMsgsROS::droneStatus::LANDED:
            interface_printout_stream << "LANDED  " << std::endl;
            break;
        case droneMsgsROS::droneStatus::FLYING:
            interface_printout_stream << "FLYING   " << std::endl;
            break;
        case droneMsgsROS::droneStatus::HOVERING:
            interface_printout_stream << "HOVERING" << std::endl;
            break;
        case droneMsgsROS::droneStatus::TAKING_OFF:
            interface_printout_stream << "TAKING OFF" << std::endl;
            break;
        case droneMsgsROS::droneStatus::LANDING:
            interface_printout_stream << "LANDING   " << std::endl;
            break;
        case droneMsgsROS::droneStatus::LOOPING:
            interface_printout_stream << "LOOPING   " << std::endl;
            break;
    }
    return &interface_printout_stream;
}

//Motion References Stream
std::stringstream *getPositionReferencesStream(){
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    interface_printout_stream
        << "POSITION REFERENCES" << std::endl;
                    if (current_position_reference.pose.position.x > -0.01){
                        interface_printout_stream << "x:       " << std::setw(5) << std::internal << fabs(current_position_reference.pose.position.x) << " m  " <<std::endl;
                    }else{
                        interface_printout_stream << "x:      " << std::setw(6) << std::internal << current_position_reference.pose.position.x     << " m  " <<std::endl;
                    }

                    if (current_position_reference.pose.position.y > -0.01){
                        interface_printout_stream  << "y:       " << std::setw(5) << std::internal << fabs(current_position_reference.pose.position.y) << " m  " <<std::endl;
                    }else{
                        interface_printout_stream  << "y:      " << std::setw(6) << std::internal << current_position_reference.pose.position.y     << " m  " <<std::endl;
                    }

                    if (current_position_reference.pose.position.z > -0.01){
                        interface_printout_stream << "z:       " << std::setw(5) << std::internal << fabs(current_position_reference.pose.position.z) << " m  " <<std::endl;
                    }else{
                        interface_printout_stream << "z:      " << std::setw(6) << std::internal << current_position_reference.pose.position.z     << " m  " <<std::endl;
                    }

                    tf2::Matrix3x3 m(tf2::Quaternion (current_position_reference.pose.orientation.x,current_position_reference.pose.orientation.y,current_position_reference.pose.orientation.z,current_position_reference.pose.orientation.w));
                    double r, p, yaw = 0;
                    m.getRPY(r, p, yaw);
                    if (std::isnan(yaw)){
                        yaw = 0.0;
                    }

                    if (yaw > -0.01){
                        interface_printout_stream << "yaw:     " << std::setw(5) << std::internal << fabs(yaw) << " rad  " <<std::endl; 
                    }else{
                        interface_printout_stream << "yaw:    " << std::setw(6) << std::internal << yaw   << " rad  " <<std::endl; 
                    }

    return &interface_printout_stream;
}

//Motion Speed References Stream
std::stringstream *getSpeedReferencesStream(){
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    interface_printout_stream
        << "SPEED REFERENCES" << std::endl;
                    if (current_speed_reference.twist.linear.x > -0.01){
                        interface_printout_stream << "dx:       " << std::setw(5) << std::internal << fabs(current_speed_reference.twist.linear.x) << " m/sec  " <<std::endl;
                    }else{
                        interface_printout_stream << "dx:      " << std::setw(6) << std::internal << current_speed_reference.twist.linear.x     << " m/sec  " <<std::endl;
                    }

                    if (current_speed_reference.twist.linear.y > -0.01){
                        interface_printout_stream  << "dy:       " << std::setw(5) << std::internal << fabs(current_speed_reference.twist.linear.y) << " m/sec  " <<std::endl;
                    }else{
                        interface_printout_stream  << "dy:      " << std::setw(6) << std::internal << current_speed_reference.twist.linear.y    << " m/sec  " <<std::endl;
                    }

                    if (current_speed_reference.twist.linear.z > -0.01){
                        interface_printout_stream << "dz:       " << std::setw(5) << std::internal << current_speed_reference.twist.linear.z << " m/sec  " <<std::endl;
                    }else{
                        interface_printout_stream << "dz:      " << std::setw(6) << std::internal << current_speed_reference.twist.linear.z     << " m/sec  " <<std::endl;
                    }

                    if (current_speed_reference.twist.angular.z > -0.01){
                        interface_printout_stream << "dyaw:     " << std::setw(5) << std::internal << fabs(current_speed_reference.twist.angular.z) << " rad  " <<std::endl; 
                    }else{
                        interface_printout_stream << "dyaw:    " << std::setw(6) << std::internal << current_speed_reference.twist.angular.z   << " rad  " <<std::endl; 
                    }
    return &interface_printout_stream;
}

