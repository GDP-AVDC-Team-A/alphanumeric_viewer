#include "../includes/alphanumeric_viewer_process_main.h"

int main(int argc, char **argv)
{
    //OUTPUT FORMAT
    interface_printout_stream << std::fixed << std::setprecision(2) << std::setfill('0'); //<< std::showpos

    //ROS 
    ros::init(argc, argv, MODULE_NAME_DRONE_CONSOLE_INTERFACE);
    ros::NodeHandle n;


    //Configuration
    n.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");

    //Sensor measurements subscribers
    BatterySubs=n.subscribe("/"+drone_id_namespace+"/"+DRONE_CONSOLE_INTERFACE_SENSOR_BATTERY, 1, &batteryCallback);
    AltitudeSubs=n.subscribe("/"+drone_id_namespace+"/"+DRONE_CONSOLE_INTERFACE_SENSOR_ALTITUDE, 1, &altitudeCallback);
    GroundSpeedSubs=n.subscribe("/"+drone_id_namespace+"/"+DRONE_CONSOLE_INTERFACE_SENSOR_GROUND_SPEED, 1, &groundSpeedCallback);

    //Actuator commands subscribers
    DronePitchRollCmdSubs=n.subscribe("/"+drone_id_namespace+"/"+DRONE_CONSOLE_INTERFACE_COMMAND_DRONE_COMMAND_PITCH_ROLL_SUBSCRIPTION, 1, &dronePitchRollCmdCallback);
    DroneDAltitudeCmdSubs=n.subscribe("/"+drone_id_namespace+"/"+DRONE_CONSOLE_INTERFACE_COMMAND_DRONE_COMMAND_DALTITUDE_SUBSCRIPTION, 1, &droneDAltitudeCmdCallback);
    DroneDYawCmdSubs=n.subscribe("/"+drone_id_namespace+"/"+DRONE_CONSOLE_INTERFACE_COMMAND_DRONE_COMMAND_DYAW_SUBSCRIPTION, 1, &droneDYawCmdCallback);
    controlModeSub=n.subscribe(std::string("/"+drone_id_namespace + "/controlMode"), 1, &controlModeSubCallback);
    
    //Self localization subscriber
    drone_estimated_GMR_pose_subscriber= n.subscribe("/"+drone_id_namespace+"/"+DRONE_STATE_ESTIMATOR_INTERFACE_POSE_SUBSCRIPTION_GMR   , 1, &drone_estimated_GMR_pose_callback_function);
    DroneStatusSubs=n.subscribe("/"+drone_id_namespace+"/"+DRONE_CONSOLE_INTERFACE_SENSOR_STATUS, 1, &droneStatusCallback);
    
    //Motion references subscriber
    drone_position_reference_subscriber= n.subscribe("/"+drone_id_namespace+"/"+DRONE_TRAJECTORY_CONTROLLER_INTERFACE_POSITION_REF_REBROADCAST_SUBSCRIPTION, 1, &droneCurrentPositionRefsSubCallback);


    //ncurses initialization (output text)
    initscr();
    curs_set(0);
    noecho();
    nodelay(stdscr, TRUE);
    erase(); refresh();

    printw("     ----ALPHANUMERIC VIEWER OF AERIAL ROBOTIC PARAMETERS----");

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
            pinterface_printout_stream = getPositionEstimates_GMRwrtGFF_Stream();
            printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);
            pinterface_printout_stream = getDroneState();
            printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);            
        
        //Right column
        lineCommands=2, columCommands=2*DISPLAY_COLUMN_SIZE+2;

            //ACTUATOR COMMANDS
            pinterface_printout_stream = getDroneCommandsStream();
            printout_stream(pinterface_printout_stream, &lineCommands, &columCommands); 

            //Control Mode
            pinterface_printout_stream = getControllerState();
            printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);

            //MOTION REFERENCES
            pinterface_printout_stream = getPositionReferences_GMRwrtGFF_Stream();
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
        << "Battery charge:     " << std::fixed << std::setprecision(0) << BatteryMsgs.batteryPercent <<  "   %%"   << std::endl << std::fixed << std::setprecision(2);
                    if (AltitudeMsgs.altitude > -0.01){
                        interface_printout_stream << "Altitude (z):       " << std::setw(5) << std::internal << fabs(AltitudeMsgs.altitude)       << " m"  << std::endl;
                    }else{
                        interface_printout_stream << "Altitude (z):      " << std::setw(6) << std::internal << AltitudeMsgs.altitude       << " m"  << std::endl;
                    }

                    if (GroundSpeedMsgs.vector.x > -0.01){
                        interface_printout_stream << "Ground speed (x):   " << std::setw(5) << std::internal << fabs(GroundSpeedMsgs.vector.x)    << " m/sec"  << std::endl;
                    }else{
                        interface_printout_stream << "Ground speed (x):  " << std::setw(6) << std::internal << GroundSpeedMsgs.vector.x    << " m/sec"  << std::endl;
                    }

                    if (GroundSpeedMsgs.vector.y > -0.01){
                        interface_printout_stream << "Ground speed (y):   " << std::setw(5) << std::internal << fabs(GroundSpeedMsgs.vector.y)    << " m/sec" << std::endl;
                    }else{
                        interface_printout_stream << "Ground speed (y):  " << std::setw(6) << std::internal << GroundSpeedMsgs.vector.y    << " m/sec" << std::endl;
                    }   
        interface_printout_stream <<  std::endl <<  std::endl;
    return &interface_printout_stream;
}

//Actuator commands Stream
std::stringstream *getDroneCommandsStream(){
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    interface_printout_stream
        << "ACTUATOR COMMANDS" << std::endl;
                    if (DroneDAltitudeCmdMsgs.dAltitudeCmd > -0.01){
                        interface_printout_stream << "Altitude rate:     " << std::setw(5) << std::internal << fabs(DroneDAltitudeCmdMsgs.dAltitudeCmd) <<  " m/sec"<<std::endl;
                    }else{
                        interface_printout_stream << "Altitude rate:    " << std::setw(6) << std::internal << DroneDAltitudeCmdMsgs.dAltitudeCmd <<  " m/sec"<<std::endl;
                    }

                    if (DroneDYawCmdMsgs.dYawCmd > -0.01){
                        interface_printout_stream << "Yaw rate:          " << std::setw(5) << std::internal << fabs(DroneDYawCmdMsgs.dYawCmd)           <<  " rad/sec" <<std::endl;
                    }else{
                        interface_printout_stream << "Yaw rate:         " << std::setw(6) << std::internal << DroneDYawCmdMsgs.dYawCmd           <<  " rad/sec" <<std::endl;
                    }

                    if (DronePitchRollCmdMsgs.pitchCmd > -0.01){
                        interface_printout_stream << "Pitch:             " << std::setw(5) << std::internal << fabs(DronePitchRollCmdMsgs.pitchCmd)     <<  " rad"  <<std::endl;
                    }else{
                        interface_printout_stream << "Pitch:            " << std::setw(6) << std::internal << DronePitchRollCmdMsgs.pitchCmd     <<  " rad"  <<std::endl;
                    }


                    if (DronePitchRollCmdMsgs.rollCmd > -0.01){
                        interface_printout_stream << "Roll:              " << std::setw(5) << std::internal << fabs(DronePitchRollCmdMsgs.rollCmd)      <<  " rad"  <<std::endl; 
                    }else{
                        interface_printout_stream << "Roll:             " << std::setw(6) << std::internal << DronePitchRollCmdMsgs.rollCmd      <<  " rad"  <<std::endl; 
                    }
    return &interface_printout_stream;
}

//Self Localization Stream
std::stringstream *getPositionEstimates_GMRwrtGFF_Stream(){
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    interface_printout_stream
        << "SELF LOCALIZATION" << std::endl;

                    if (last_drone_estimated_GMRwrtGFF_pose_msg_.x > -0.01){
                        interface_printout_stream << "x:       " << std::setw(5) << std::internal << fabs(last_drone_estimated_GMRwrtGFF_pose_msg_.x) << " m" <<std::endl;
                    }else{
                        interface_printout_stream << "x:      " << std::setw(6) << std::internal << last_drone_estimated_GMRwrtGFF_pose_msg_.x << " m" <<std::endl;
                    }

                    if (last_drone_estimated_GMRwrtGFF_pose_msg_.y > -0.01){
                        interface_printout_stream  << "y:       " << std::setw(5) << std::internal << fabs(last_drone_estimated_GMRwrtGFF_pose_msg_.y) << " m" << std::endl;
                    }else{
                        interface_printout_stream  << "y:      " << std::setw(6) << std::internal << last_drone_estimated_GMRwrtGFF_pose_msg_.y << " m" << std::endl;
                    }

                    if (last_drone_estimated_GMRwrtGFF_pose_msg_.z > -0.01){
                        interface_printout_stream << "z:       " << std::setw(5) << std::internal << fabs(last_drone_estimated_GMRwrtGFF_pose_msg_.z) << " m" <<  std::endl;
                    }else{
                        interface_printout_stream << "z:      " << std::setw(6) << std::internal << last_drone_estimated_GMRwrtGFF_pose_msg_.z << " m" <<  std::endl;
                    }


                    if (last_drone_estimated_GMRwrtGFF_pose_msg_.yaw > -0.01){
                        interface_printout_stream << "yaw:     " << std::setw(5) << std::internal << fabs(last_drone_estimated_GMRwrtGFF_pose_msg_.yaw) << " rad" <<  std::endl; 
                    }else{
                        interface_printout_stream << "yaw:    " << std::setw(6) << std::internal << last_drone_estimated_GMRwrtGFF_pose_msg_.yaw << " rad" <<  std::endl; 
                    }

                    if (last_drone_estimated_GMRwrtGFF_pose_msg_.pitch > -0.01){
                        interface_printout_stream << "pitch:   " << std::setw(5) << std::internal << fabs(last_drone_estimated_GMRwrtGFF_pose_msg_.pitch) << " rad" << std::endl;
                    }else{
                        interface_printout_stream << "pitch:  " << std::setw(6) << std::internal << last_drone_estimated_GMRwrtGFF_pose_msg_.pitch << " rad" << std::endl;
                    }


                    if (last_drone_estimated_GMRwrtGFF_pose_msg_.roll > -0.01){
                        interface_printout_stream << "roll:    " << std::setw(5) << std::internal << fabs(last_drone_estimated_GMRwrtGFF_pose_msg_.roll) << " rad" << std::endl;
                    }else{
                        interface_printout_stream << "roll:   " << std::setw(6) << std::internal << last_drone_estimated_GMRwrtGFF_pose_msg_.roll << " rad" << std::endl;
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
    case Controller_MidLevel_controlMode::SPEED_CONTROL:
        interface_printout_stream << "SPEED     " << std::endl;
        break;
    case Controller_MidLevel_controlMode::POSITION_CONTROL:
        interface_printout_stream << "POSITION  " << std::endl;
        break;
    case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
        interface_printout_stream << "TRAJECTORY" << std::endl;
        break;
     case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
    default:
        interface_printout_stream << "UNKNOWN   " << std::endl;
        break;
    }
    interface_printout_stream << std::endl;
    return &interface_printout_stream;
}

//Status Stream
std::stringstream *getDroneState(){
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    interface_printout_stream
        << "Status:  ";
    switch (DroneStatusMsgs.status) {
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
std::stringstream *getPositionReferences_GMRwrtGFF_Stream(){
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    interface_printout_stream
        << "MOTION REFERENCES" << std::endl;
                    if (current_drone_position_reference_.x > -0.01){
                        interface_printout_stream << "x:       " << std::setw(5) << std::internal << fabs(current_drone_position_reference_.x) << " m" <<std::endl;
                    }else{
                        interface_printout_stream << "x:      " << std::setw(6) << std::internal << current_drone_position_reference_.x     << " m" <<std::endl;
                    }

                    if (current_drone_position_reference_.y > -0.01){
                        interface_printout_stream  << "y:       " << std::setw(5) << std::internal << fabs(current_drone_position_reference_.y) << " m" <<std::endl;
                    }else{
                        interface_printout_stream  << "y:      " << std::setw(6) << std::internal << current_drone_position_reference_.y     << " m" <<std::endl;
                    }

                    if (current_drone_position_reference_.z > -0.01){
                        interface_printout_stream << "z:       " << std::setw(5) << std::internal << fabs(current_drone_position_reference_.z) << " m" <<std::endl;
                    }else{
                        interface_printout_stream << "z:      " << std::setw(6) << std::internal << current_drone_position_reference_.z     << " m" <<std::endl;
                    }


                    if (current_drone_position_reference_.yaw > -0.01){
                        interface_printout_stream << "yaw:     " << std::setw(5) << std::internal << fabs(current_drone_position_reference_.yaw) << " rad" <<std::endl; 
                    }else{
                        interface_printout_stream << "yaw:    " << std::setw(6) << std::internal << current_drone_position_reference_.yaw   << " rad" <<std::endl; 
                    }

                    if (current_drone_position_reference_.pitch > -0.01){
                        interface_printout_stream << "pitch:   " << std::setw(5) << std::internal << fabs(current_drone_position_reference_.pitch) << " rad" <<std::endl;
                    }else{
                        interface_printout_stream << "pitch:  " << std::setw(6) << std::internal << current_drone_position_reference_.pitch << " rad" <<std::endl;
                    }


                    if (current_drone_position_reference_.roll > -0.01){
                        interface_printout_stream << "roll:    " << std::setw(5) << std::internal << fabs(current_drone_position_reference_.roll) << " rad" <<std::endl;
                    }else{
                        interface_printout_stream << "roll:   " << std::setw(6) << std::internal << current_drone_position_reference_.roll  << " rad" <<std::endl;
                    }
    return &interface_printout_stream;
}