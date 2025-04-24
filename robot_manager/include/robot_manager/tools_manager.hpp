#ifndef TOOLS_MANAGER_H
#define TOOLS_MANAGER_H


#include "robot_manager/tools_raw_manager.hpp"

/*
    Si TRUE, la tâche est fini
    Si FALSE, la tâche est en cours
*/
// static MapBool flagMapBool;
// static MapBool tasksStatusMapBool;

extern ManagerParam managerParam;
// DynamicReconfigureClient reconfigure_client;

// void init_all_robot_param(ManagerParam& managerParamObj);// fonction pour charger tout les paramètres du robot

void init_all_telecom_msg(ros::nodeHandle& nh);// fonction pour charger tout les messages de communication du robot
void init_robot_param(ros::NodeHandle& nh);// fonction pour initialiser les différents paramétres du robot

void timerCallback_Waiting_Camera_Capture(const ros::TimerEvent&);
void timerCallback_Warning_Time_Alarm(const ros::TimerEvent&);
void timerCallback_Stop_Time_Alarm(const ros::TimerEvent&);

void get_all_bool_data_param_from_node_step(ros::NodeHandle& n);
void fast_motion_step(ros::NodeHandle& n);
void camera_detection_step(ros::NodeHandle& n, ros::Timer timer);
void slow_motion_step(ros::NodeHandle& n);
// void warning_alarm_activated(ros::NodeHandle& n);
// void stop_alarm_activated(ros::NodeHandle& n);
void motion_speed(ros::nodeHandle& nh, std::string speed = MEDIUM_SPEED);


bool condition_STOP_CAMERA_TRACKING_CAN_POSE(ros::NodeHandle& n);
bool condition_RUN_MOVEBASE_SLOW_MOTION(ros::NodeHandle& n);
bool condition_RUN_MOVEBASE_FAST_MOTION(ros::NodeHandle& n);
bool condition_AIM_TARGET(ros::NodeHandle& n);

#endif