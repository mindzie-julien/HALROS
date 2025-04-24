import math

TASK_DURATION_IN_SECOND = 2
MATCH_DURATION_IN_SECOND = 90
ALERT_FLAG_DURATION_IN_SECOND = 10


#/********************************************/
#/*********                          *********/
#/********************************************/
FLAG_MAP_BOOL = "flagMapBool"

STATUS_GUI = "/statusGUI" #//STATUS_GUI - true si main valide l'envoi des coordonnées cibles aux programmes de navigation autonome et non-autonom, false sinon
MOTION_MODE = "/motionMode" #//MOTION_MODE - true si main selectionne le déplacement rapide, false pour le déplacement lent.
STATUS_MOTION = "/statusMotion" #//STATUS_MOTION - true si main valide la lecture des coordonnées cibles par les programmes de navigation rapide et lente.
CONTROL_MOTION_END = "/controlMotionEnd" #//CONTROL_MOTION_END - true si moveride a terminé la navigation non-autonome, false sinon.
STATUS_MOVEBASE_END = "/statusMoveBaseEnd" #//STATUS_MOVEBASE_END - true si movebase a terminé la navigation autonome, false sinon.
MOVEBASE_CONFIG_FLAG = "/movebaseConfigFlag" #// flag d'autorisation de la modification des configurations des élements de movebase  
STATUS_CAM = "/statusCAM" #//STATUS_CAM - true si l'asservissement en position avec la camera est autorisé, false sinon.
TASK_FINISHED = "/taskFinished" #//TASK_FINISHED = false booléen du timer1, true si la tâche est terminé, false si la tâche n'est pas terminé.
WARNING_ALARM = "/warningAlarm" #//WARNING_ALARM = falsebooléen du timer2, true si c'est le moment de rentrer dans une zone d'arrivée, false sinon .
STOP_MOTION = "/stopMotion" #//STOP_MOTION = false booléen du timer3, true si le robot doit s'arrête, false sinon.
CURRENT_TIME = "/currentTime" #// temps actuel depuis l'activation du système

#/********************************************/
#/*********                          *********/
#/********************************************/
TASKS_STATUS_MAP_BOOL = "tasksStatusMapBool"

#/********************************************/
#/*********                          *********/
#/********************************************/
_5_MILLISECONDS = 0.005
slow_acc_lim_x = 0.05
slow_max_vel_x = 0.2
slow_max_vel_theta = (math.pi)/3
slow_acc_lim_theta = (math.pi)/6
fast_acc_lim_x = 0.5
fast_max_vel_x = 1.0
fast_max_vel_theta = (math.pi)/2
fast_acc_lim_theta = (math.pi)/4
min_in_place_vel_theta = math.radians((math.pi)/12) # 15 deg/s de tolérance pour la vitesse angulaire d'orientation du robot
min_vel_x = 0.05
min_vel_theta = math.radians((math.pi)/12)# 15 deg/s de tolérance pour la vitesse angulaire d'orientation du robot
yaw_goal_tolerance = math.radians((math.pi)/12) # 15 degrée de tolérance pour l'angle d'orientation du robot
xy_goal_tolerance = 0.05
escape_vel = -0.9
holonomic_robot = True
y_vels = [-1, -0.2, 0.2, 1]
latch_xy_goal_tolerance =False