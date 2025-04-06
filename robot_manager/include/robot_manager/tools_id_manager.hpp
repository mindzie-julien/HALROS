#ifndef TOOLS_ID_MANAGER_H
#define TOOLS_ID_MANAGER_H

#include <cmath> // Inclure la bibliothèque cmath pour M_PI

#define TASK_DURATION_IN_SECOND 2
#define MATCH_DURATION_IN_SECOND 90
#define ALERT_FLAG_DURATION_IN_SECOND 10


/********************************************/
/*********                          *********/
/********************************************/
#define FLAG_MAP_BOOL "flagMapBool";

#define STATUS_GUI "/statusGUI" //STATUS_GUI - true si main valide l'envoi des coordonnées cibles aux programmes de navigation autonome et non-autonom, false sinon
#define MOTION_MODE "/motionMode" //MOTION_MODE - true si main selectionne le déplacement rapide, false pour le déplacement lent.
#define STATUS_MOTION "/statusMotion" //STATUS_MOTION - true si main valide la lecture des coordonnées cibles par les programmes de navigation rapide et lente.
#define CONTROL_MOTION_END "/controlMotionEnd" //CONTROL_MOTION_END - true si moveride a terminé la navigation non-autonome, false sinon.
#define STATUS_MOVEBASE_END "/statusMoveBaseEnd" //STATUS_MOVEBASE_END - true si movebase a terminé la navigation autonome, false sinon.
#define STATUS_CAM "/statusCAM" //STATUS_CAM - true si l'asservissement en position avec la camera est autorisé, false sinon.
#define TASK_FINISHED "/taskFinished" //TASK_FINISHED = false; booléen du timer1, true si la tâche est terminé, false si la tâche n'est pas terminé.
#define WARNING_ALARM "/warningAlarm" //WARNING_ALARM = false;booléen du timer2, true si c'est le moment de rentrer dans une zone d'arrivée, false sinon .
#define STOP_MOTION "/stopMotion" //STOP_MOTION = false; booléen du timer3, true si le robot doit s'arrête, false sinon.
#define CURRENT_TIME "/currentTime" // temps actuel depuis l'activation du système

/********************************************/
/*********                          *********/
/********************************************/
#define TASKS_STATUS_MAP_BOOL "tasksStatusMapBool";

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

#endif