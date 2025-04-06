
#include "robot_manager/tools_manager.hpp"



/*
callback du timer1
Toujours mettre taskFinished = false avant d'utiliser le timer associé
*/
void timerCallback_Waiting_Camera_Capture(const ros::TimerEvent&)
{
    managerParam.flagMapItem.setValue(TASK_FINISHED, true);
    ROS_INFO("Waiting_Camera_Capture Timer triggered");
    
}

/*
callback du timer2
Toujours mettre warningAlarm = false avant d'utiliser le timer associé
*/
void timerCallback_Warning_Time_Alarm(const ros::TimerEvent&)
{
    managerParam.flagMapItem.setValue(WARNING_ALARM, true);
    ROS_INFO("Warning_Time_Alarm Timer triggered");
    
}

/*
callback du timer3
Toujours mettre stopMotion = false avant d'utiliser le timer associé
*/
void timerCallback_Stop_Time_Alarm(const ros::TimerEvent&)
{
    managerParam.flagMapItem.setValue(STOP_MOTION, true);
    ROS_INFO("Stop_Time_Alarm Timer triggered");
    
}


void get_all_bool_data_param_from_node_step(ros::NodeHandle& n)
{
    managerParam.get_bool(n, STATUS_GUI);
    managerParam.get_bool(n, MOTION_MODE);
    managerParam.get_bool(n, STATUS_MOTION);
    managerParam.get_bool(n, CONTROL_MOTION_END);
    managerParam.get_bool(n, STATUS_MOVEBASE_END);
    managerParam.get_bool(n, WARNING_ALARM);
}

void fast_motion_step(ros::NodeHandle& n)
{
    // Modifier dynamiquement la vitesse et l'accélération
    reconfigure_client.updateMaxVel(fast_motion_MaxVel, fast_motion_MaxVel_Theta);  // Nouvelle valeur pour max_vel_x
    reconfigure_client.updateMaxAccel(fast_motion_MaxAccel, fast_motion_MaxAccel_Theta); // Nouvelle valeur pour max_accel_x

    // déclenchement de la lecture de coordonnée cible après configuration de paramètre de déplacement
    managerParam.flagMapItem.setValue(STATUS_MOTION, true); // chargement de la donnée dans le gestionnaire de donnée manager
    managerParam.set_bool(n, STATUS_MOTION); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager

    // dans la boucle suivante nous vérifions que le robot est terminé sont déplacement rapide et que le temps d'action n'est pas terminé
    while (condition_RUN_MOVEBASE_FAST_MOTION()) 
    {
        ROS_INFO("robot is moving on movebase software for fast motion");
        
        // verification de la fin du logiciel movebase pour les déplacements rapide
        managerParam.get_bool(n, STATUS_MOVEBASE_END);
        managerParam.get_bool(n, WARNING_ALARM);
    }

    // bloquage de la lecture de coordonnée cible après le déplacemnt effectif du système
    managerParam.flagMapItem.setValue(STATUS_MOTION, false); // chargement de la donnée dans le gestionnaire de donnée manager
    managerParam.set_bool(n, STATUS_MOTION); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager
}

void slow_motion_step(ros::NodeHandle& n)
{

    // Modifier dynamiquement la vitesse et l'accélération
    reconfigure_client.updateMaxVel(slow_motion_MaxVel, slow_motion_MaxVel_Theta);  // Nouvelle valeur pour max_vel_x
    reconfigure_client.updateMaxAccel(slow_motion_MaxAccel, slow_motion_MaxAccel_Theta); // Nouvelle valeur pour max_accel_x

    // déclenchement de la lecture de coordonnée cible après configuration de paramètre de déplacement
    managerParam.flagMapItem.setValue(STATUS_MOTION, true); // chargement de la donnée dans le gestionnaire de donnée manager
    managerParam.set_bool(n, STATUS_MOTION); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager// autorisation de la  lecture des coordonnées cibles pour le déplacement autonome et non-autonome

    // dans la boucle suivante nous vérifions que le robot est terminé sont déplacement non-autonome et que le temps d'action n'est pas terminé
    while (condition_RUN_MOVEBASE_SLOW_MOTION(n)) 
    {
        ROS_INFO("robot is moving on movebase software for slow motion");

        // verification de la fin du logiciel movebase pour les déplacements lent
        managerParam.get_bool(n, STATUS_MOVEBASE_END);
        managerParam.get_bool(n, WARNING_ALARM);
    }

    // bloquage de la lecture de coordonnée cible après le déplacemnt effectif du système
    managerParam.flagMapItem.setValue(STATUS_MOTION, false); // chargement de la donnée dans le gestionnaire de donnée manager
    managerParam.set_bool(n, STATUS_MOTION); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager
}

void camera_detection_step(ros::NodeHandle& n, ros::Timer timer)
{
    // vérification de l'autorisation de l'asservissement en position entre les canettes et la pince
    managerParam.get_bool(n, STATUS_CAM);

    managerParam.flagMapItem.setValue(TASK_FINISHED, false);// mise en condition normale de la variable pour détecter si l'attente d'une tâche est terminé
    
    timer.start();// démarrage du timer

    managerParam.get_bool(n, TASK_FINISHED);
    managerParam.get_bool(n, WARNING_ALARM);
    managerParam.get_bool(n, STATUS_CAM);

    /*
    la boucle suivante permet d'attendre que le programme de détection des tags aruco de canette ait détecté les canettes, 
    si les canettes sont détecté il poursuit avec de l'asservissement en position,
    sinon il poursuit une nouvelle coordonnées.
    */
    while(condition_STOP_CAMERA_TRACKING_CAN_POSE(n))
    {
        managerParam.set_bool(n, MOTION_MODE);
        managerParam.set_bool(n, STATUS_CAM);// remise en condition normale de la validation de l'asservissement en position
        timer.stop();
        managerParam.flagMapItem.setValue(TASK_FINISHED, true);
        
    }
}



bool condition_AIM_TARGET(ros::NodeHandle& n)
{
    bool value, op_value; // value - valeur récupèrer, op_value - résultat d'opérations logique pour une condition
    
    managerParam.get_bool(n, STATUS_GUI); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager
    managerParam.flagMapItem.getValue(STATUS_GUI, value); // chargement de la donnée à partir du gestionnaire de donnée manager
    op_value = value; // (statusGUI)

    managerParam.get_bool(n, WARNING_ALARM); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager
    managerParam.flagMapItem.getValue(WARNING_ALARM, value); // chargement de la donnée à partir du gestionnaire de donnée manager
    op_value = (op_value &&(!value)); // (statusGUI)&&(!warningAlarm)

    return op_value;
}

bool condition_RUN_MOVEBASE_SLOW_MOTION(ros::NodeHandle& n)
{
    bool value, op_value; // value - valeur récupèrer, op_value - résultat d'opérations logique pour une condition
    
    managerParam.get_bool(n, WARNING_ALARM); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager
    managerParam.flagMapItem.getValue(WARNING_ALARM, value); // chargement de la donnée à partir du gestionnaire de donnée manager
    op_value = !value; // (!warningAlarm)
    
    managerParam.get_bool(n, STATUS_MOVEBASE_END); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager
    managerParam.flagMapItem.getValue(STATUS_MOVEBASE_END, value); // chargement de la donnée à partir du gestionnaire de donnée manager
    op_value = (op_value && (!value)); // ((!warningAlarm)&&(!statusMoveBaseEnd))

    managerParam.get_bool(n, MOTION_MODE); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager
    managerParam.flagMapItem.getValue(MOTION_MODE, value); // chargement de la donnée à partir du gestionnaire de donnée manager
    op_value = (op_value && (!value)); // ((!warningAlarm)&&(!statusMoveBaseEnd)&&(motionMode))

    return op_value;
}

bool condition_RUN_MOVEBASE_FAST_MOTION(ros::NodeHandle& n)
{
    bool value, op_value; // value - valeur récupèrer, op_value - résultat d'opérations logique pour une condition
    
    managerParam.get_bool(n, WARNING_ALARM); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager
    managerParam.flagMapItem.getValue(WARNING_ALARM, value); // chargement de la donnée à partir du gestionnaire de donnée manager
    op_value = !value; // (!warningAlarm)
    
    managerParam.get_bool(n, STATUS_MOVEBASE_END); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager
    managerParam.flagMapItem.getValue(STATUS_MOVEBASE_END, value); // chargement de la donnée à partir du gestionnaire de donnée manager
    op_value = (op_value && (!value)); // ((!warningAlarm)&&(!statusMoveBaseEnd))

    managerParam.get_bool(n, MOTION_MODE); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager
    managerParam.flagMapItem.getValue(MOTION_MODE, value); // chargement de la donnée à partir du gestionnaire de donnée manager
    op_value = (op_value && (value)); // ((!warningAlarm)&&(!statusMoveBaseEnd)&&(!motionMode))

    return op_value;
}

bool condition_STOP_CAMERA_TRACKING_CAN_POSE(ros::NodeHandle& n)
{
    bool value, op_value; // value - valeur récupèrer, op_value - résultat d'opérations logique pour une condition
    
    managerParam.get_bool(n, WARNING_ALARM); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager
    managerParam.flagMapItem.getValue(WARNING_ALARM, value);
    op_value = !value; // (!warningAlarm)

    managerParam.get_bool(n, TASK_FINISHED); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager
    managerParam.flagMapItem.getValue(TASK_FINISHED, value);
    op_value = (op_value)&&(!value); // (!warningAlarm)&&(!taskFinished)

    managerParam.get_bool(n, STATUS_CAM); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager
    managerParam.flagMapItem.getValue(STATUS_CAM, value);
    op_value = (op_value)&&(value); // (!warningAlarm)&&(!taskFinished)&&(statusCAM)
    return op_value;
}
