#ifndef TOOLS_MANAGER_H
#define TOOLS_MANAGER_H


#include "robot_manager/tools_raw_manager.hpp"

/*
    Si TRUE, la tâche est fini
    Si FALSE, la tâche est en cours
*/
// static MapBool flagMapBool;
// static MapBool tasksStatusMapBool;

ManagerParam managerParam;
DynamicReconfigureClient reconfigure_client;


void timerCallback_Waiting_Camera_Capture(const ros::TimerEvent&);
void timerCallback_Warning_Time_Alarm(const ros::TimerEvent&);
void timerCallback_Stop_Time_Alarm(const ros::TimerEvent&);

void get_all_bool_data_param_from_node_step(ros::NodeHandle& n);
void fast_motion_step(ros::NodeHandle& n);
void camera_detection_step(ros::NodeHandle& n, ros::Timer timer);
void slow_motion_step(ros::NodeHandle& n);
// void warning_alarm_activated(ros::NodeHandle& n);
// void stop_alarm_activated(ros::NodeHandle& n);


bool condition_STOP_CAMERA_TRACKING_CAN_POSE(ros::NodeHandle& n);
bool condition_RUN_MOVEBASE_SLOW_MOTION(ros::NodeHandle& n);
bool condition_RUN_MOVEBASE_FAST_MOTION(ros::NodeHandle& n);
bool condition_AIM_TARGET(ros::NodeHandle& n);

#endif