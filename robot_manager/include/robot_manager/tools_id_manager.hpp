#ifndef TOOLS_ID_MANAGER_H
#define TOOLS_ID_MANAGER_H

#include <cmath> // Inclure la bibliothèque cmath pour M_PI

#include "robot_manager/telecom_tools_id_manager.hpp"
#include "robot_manager/gui_tools_id_manager.hpp"

#define TASK_DURATION_IN_SECOND 2
#define MATCH_DURATION_IN_SECOND 90
#define ALERT_FLAG_DURATION_IN_SECOND 10


/********************************************/
/*********                          *********/
/********************************************/
#define FLAG_MAP_BOOL "flagMapBool"

#define STATUS_GUI "/statusGUI" //STATUS_GUI - true si main valide l'envoi des coordonnées cibles aux programmes de navigation autonome et non-autonom, false sinon
#define MOTION_MODE "/motionMode" //MOTION_MODE - true si main selectionne le déplacement rapide, false pour le déplacement lent.
#define STATUS_MOTION "/statusMotion" //STATUS_MOTION - true si main valide la lecture des coordonnées cibles par les programmes de navigation rapide et lente.
#define CONTROL_MOTION_END "/controlMotionEnd" //CONTROL_MOTION_END - true si moveride a terminé la navigation non-autonome, false sinon.
#define STATUS_MOVEBASE_END "/statusMoveBaseEnd" //STATUS_MOVEBASE_END - true si movebase a terminé la navigation autonome, false sinon.
#define MOVEBASE_CONFIG_FLAG "/movebaseConfigFlag" // flag d'autorisation de la modification des configurations des élements de movebase  
#define STATUS_CAM "/statusCAM" //STATUS_CAM - true si l'asservissement en position avec la camera est autorisé, false sinon.
#define TASK_FINISHED "/taskFinished" //TASK_FINISHED = false; booléen du timer1, true si la tâche est terminé, false si la tâche n'est pas terminé.
#define WARNING_ALARM "/warningAlarm" //WARNING_ALARM = false;booléen du timer2, true si c'est le moment de rentrer dans une zone d'arrivée, false sinon .
#define STOP_MOTION "/stopMotion" //STOP_MOTION = false; booléen du timer3, true si le robot doit s'arrête, false sinon.
#define CURRENT_TIME "/currentTime" // temps actuel depuis l'activation du système

/********************************************/
/*********                          *********/
/********************************************/

#define RAW_POSE_TOPIC "/rawPose"
#define TOF_TOPIC "/TOF_data"
#define CMD_VEL_TOPIC "/cmdVel"
#define GUI_RX_TOPIC "/GUI_rx" // nom du topic pour recevoir des données du module GUI de feedback
#define GUI_TX_TOPIC "/GUI_tx" // nom du topic pour envoyer des données du module GUI de commande
#define RAW_VELOCITY_TOPIC "/rawVel"
#define ACTUATOR_FEEDBACK_TOPIC "/actuatorFeedback"
#define TARGET_LOCATION_TOPIC "/targetLocation"
#define TARGET_PATH_TOPIC "/targetPath"

/********************************************/
/*********                          *********/
/********************************************/

#define FAST_SPEED "fast"
#define MEDIUM_SPEED "medium"
#define LOW_SPEED "low"

/********************************************/
/*********                          *********/
/********************************************/
#define TASKS_STATUS_MAP_BOOL "tasksStatusMapBool"
#define MOVEBASE_LOCAL_PLANNNER_PATH "/move_base_node/TrajectoryPlannerROS/"

/********************************************/
/*********                          *********/
/********************************************/
const double _5_MILLISECONDS = 0.005;
const double slow_motion_MaxAccel = 0.05;
const double slow_motion_MaxVel = 0.2;
const double slow_motion_MaxVel_Theta = M_PI/3;
const double slow_motion_MaxAccel_Theta = M_PI/6;
const double fast_motion_MaxAccel = 0.5;
const double fast_motion_MaxVel = 1.0;
const double fast_motion_MaxVel_Theta = M_PI/2;
const double fast_motion_MaxAccel_Theta = M_PI/4;
const double yaw_tolerance = M_PI/12; // 15 dégré de tolérance pour l'angle d'orientation du robot

#endif // TOOLS_ID_MANAGER_H