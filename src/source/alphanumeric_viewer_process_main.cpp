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
    start_color();
    use_default_colors();  
    curs_set(0);
    noecho();
    nodelay(stdscr, TRUE);
    erase();
    refresh();
    init_pair(1, COLOR_GREEN, -1);
    init_pair(2, COLOR_RED, -1);
    init_pair(3, COLOR_YELLOW, -1);


    printStaticMenu();
       
    //Rate
    ros::Rate loop_rate(8);

    //Loop
    while (ros::ok()) {
        
        //Read messages
        ros::spinOnce();

        //Sensor measurements
        move(3,20);
        printBattery();
        move(4,20);
        printStream(altitude_msg.altitude);printw(" m   ");
        move(5,20);
        printStream(ground_speed_msg.vector.x);printw(" m/s   ");
        move(6,20);
        printStream(ground_speed_msg.vector.y);printw(" m/s   ");

        //Self localization
        move(10,11);
        printStream(current_pose.pose.position.x);printw(" m   ");
        move(11,11);
         printStream(current_pose.pose.position.y);printw(" m   ");
        move(12,11);
         printStream(current_pose.pose.position.z);printw(" m   ");
        tf2::Matrix3x3 m(tf2::Quaternion (current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w));
        double r, p, yaw;
        m.getRPY(r, p, yaw);
        if (std::isnan(yaw)) yaw = 0.0;
        if (std::isnan(r)) r = 0.0;
        if (std::isnan(p)) p = 0.0;
        move(13,11);
         printStream(yaw);printw(" rad   ");
        move(14,11);
         printStream(p);printw(" rad   ");
        move(15,11);
         printStream(r);printw(" rad   ");
        move(16,12);
         printQuadrotorState();

        //Position references
        move(10,34);
        printStream(current_position_reference.pose.position.x);printw(" m   ");
        move(11,34);
        printStream(current_position_reference.pose.position.y);printw(" m   ");
        move(12,34);
        printStream(current_position_reference.pose.position.z);printw(" m   ");
        tf2::Matrix3x3 m2(tf2::Quaternion (current_position_reference.pose.orientation.x,current_position_reference.pose.orientation.y,current_position_reference.pose.orientation.z,current_position_reference.pose.orientation.w));
        r = 0; p = 0; yaw = 0;
        m2.getRPY(r, p, yaw);
        if (std::isnan(yaw)) yaw = 0.0; 
        move(13,34);
        printStream(yaw);printw(" rad");

        //Speed references
        move(10,59);
        printStream(current_speed_reference.twist.linear.x);printw(" m/s  ");
        move(11,59);
        printStream(current_speed_reference.twist.linear.y);printw(" m/s   ");
        move(12,59);
        printStream(current_speed_reference.twist.linear.z);printw(" m/s    ");
        move(13,59);
        printStream(current_speed_reference.twist.angular.z);printw(" rad   ");

        //Actuator commands
        move(3,66);
        printStream(quadrotor_command_msg.thrust.z);printw(" m/s  ");
        move(4,66);
        printStream(quadrotor_command_msg.yaw_rate);printw(" rad/s  ");
        move(5,66);
        printStream(quadrotor_command_msg.pitch);printw(" rad  ");
        move(6,66);
        printStream(quadrotor_command_msg.roll);printw(" rad  ");
        move(7,67);
        printControlMode();  


        //Refresh
        refresh();
        loop_rate.sleep();
    }

    //End ncurses
    endwin();
    return 0;
}

//Print battery charge
void printBattery(){
    if(battery_msg.batteryPercent == 100) {
        move(3,23);
        attron(COLOR_PAIR(1));printw("%.0f",battery_msg.batteryPercent);attroff(COLOR_PAIR(1));
    }
    if(battery_msg.batteryPercent > 50 && battery_msg.batteryPercent < 100) {
        move(3,24);
        attron(COLOR_PAIR(1));printw("%.0f",battery_msg.batteryPercent);attroff(COLOR_PAIR(1));
    }
    if(battery_msg.batteryPercent <= 50 && battery_msg.batteryPercent > 20) {
        move(3,24);
        attron(COLOR_PAIR(3));printw("%.0f",battery_msg.batteryPercent);attroff(COLOR_PAIR(3));
    }
    if(battery_msg.batteryPercent <= 20) {
        if (battery_msg.batteryPercent > 10) move(3,24);
        else move(3,25);
        attron(COLOR_PAIR(2));printw("%.0f",battery_msg.batteryPercent);attroff(COLOR_PAIR(2));     
    }
move(3,27);
printw("%%");
}

//Print float using stringstream
void printStream(float var) {
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    if (var > -0.01){
        interface_printout_stream << std::setw(5) << std::internal << fabs(var);
        attron(COLOR_PAIR(1));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(1));
    }else{
        interface_printout_stream << std::setw(6) << std::internal << var;
        attron(COLOR_PAIR(2));printw("%s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(2));
    }
} 

//Print double using stringstream
void printStream(double var) {
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    if (var > -0.01){
        interface_printout_stream << std::setw(5) << std::internal << fabs(var);
        attron(COLOR_PAIR(1));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(1));
    }else{
        interface_printout_stream << std::setw(6) << std::internal << var;
        attron(COLOR_PAIR(2));printw("%s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(2));
    }
} 

//Print control mode
void printControlMode(){
    switch (last_received_control_mode) {
    case aerostack_msgs::QuadrotorPidControllerMode::SPEED:
        printw("SPEED        ");
        break;
    case aerostack_msgs::QuadrotorPidControllerMode::POSE:
        printw("POSITION     ");
        break;
    case aerostack_msgs::QuadrotorPidControllerMode::GROUND_SPEED:
        printw("GROUND SPEED ");
        break;
    case aerostack_msgs::QuadrotorPidControllerMode::SPEED_3D:
        printw("SPEED 3D     ");
        break;        
     case aerostack_msgs::QuadrotorPidControllerMode::UNKNOWN:
    default:
        printw("UNKNOWN      ");
        break;
    }
}

//Print status
void printQuadrotorState(){
    switch (quadrotor_status_msg.status) {
        case droneMsgsROS::droneStatus::UNKNOWN:
            printw("UNKNOWN   ");
            break;
        case droneMsgsROS::droneStatus::INITED:
            printw("INIT      ");
            break;
        case droneMsgsROS::droneStatus::LANDED:
            printw("LANDED    ");
            break;
        case droneMsgsROS::droneStatus::FLYING:
            printw("FLYING    ");
            break;
        case droneMsgsROS::droneStatus::HOVERING:
            printw("HOVERING  ");
            break;
        case droneMsgsROS::droneStatus::TAKING_OFF:
            printw("TAKING OFF");
            break;
        case droneMsgsROS::droneStatus::LANDING:
            printw("LANDING   ");
            break;
        case droneMsgsROS::droneStatus::LOOPING:
            printw("LOOPING   ");
            break;
    }
}

void printStaticMenu(){
    move(0,0);
    printw("             - ALPHANUMERIC VIEWER OF AERIAL ROBOTIC PARAMETERS -");
    //Sensor measurements
    move(2,0);
    printw(" SENSOR MEASUREMENTS");
    move(3,0);
    printw(" Battery charge:");
    move(4,0);
    printw(" Altitude (z):");
    move(5,0);
    printw(" Ground speed (x):");
    move(6,0);
    printw(" Ground speed (y):");

    //Actuator commands
    move(2,50);
    printw("ACTUATOR COMMANDS");
    move(3,50);
    printw("Altitude rate:");
    move(4,50);
    printw("Yaw rate:");
    move(5,50);
    printw("Pitch:");
    move(6,50);
    printw("Roll:");
    move(7,50);
    printw("Control mode:");

    //Self localization
    move(9,0);
    printw(" SELF LOCALIZATION");
    move(10,0);
    printw(" x:");
    move(11,0);
    printw(" y:");
    move(12,0);
    printw(" z:");
    move(13,0);
    printw(" yaw:");
    move(14,0);
    printw(" pitch:");
    move(15,0);
    printw(" roll:");
    move(16,0);
    printw(" Status:");

    //Position references
    move(9,26);
    printw("POSITION REFERENCES");
    move(10,26);
    printw("x:");
    move(11,26);
    printw("y:");
    move(12,26);
    printw("z:");
    move(13,26);
    printw("yaw:");

    //Speed references
    move(9,50);
    printw("SPEED REFERENCES");
    move(10,50);
    printw("dx:");
    move(11,50);
    printw("dy:");
    move(12,50);
    printw("dz:");
    move(13,50);
    printw("dyaw:");    
}