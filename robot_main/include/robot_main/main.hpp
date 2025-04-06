#ifndef MAIN_H
#define MAIN_H
#include <ros/ros.h>
#include <robot_tools/tools.hpp>
#include <vector>



/*
Toujours mettre warningAlarm = false avant d'utiliser timer2
timer pour demander au robot de rentrer dans une zone d'arrivé avant la fin du match.
*/
ros::Timer timer_Warning_Time_Alarm;

/*
Toujours mettre stopMotion = false avant d'utiliser timer3
timer pour arrêter le robot au moment de la fin du temps de jeu.
*/
ros::Timer timer_Stop_Time_Alarm;

void starting_detected();// fonction pour détecter le lancement manuel

void init_timer(ros::NodeHandle& nh);// fonction pour initialiser les timers


#endif