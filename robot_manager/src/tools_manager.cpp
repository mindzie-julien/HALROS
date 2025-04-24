
#include "robot_manager/tools_manager.hpp"

ManagerParam managerParam;

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
    managerParam.get_bool(n, WARNING_ALARM);MOVEBASE_CONFIG_FLAG;
}

void fast_motion_step(ros::NodeHandle& n)
{

    /******************************************************/
    /* configuration des paramètres dynamique de movebase */
    /******************************************************/ 

    // Créer un client dynamic_reconfigure pour se connecter à base_local_planner de move_base
    // dynamic_reconfigure::Client<dynamic_reconfigure::Config> client("base_local_planner/TrajectoryPlannerROS");

    // // Créer une requête pour modifier la configuration du planner
    // dynamic_reconfigure::ReconfigureRequest req;
    // dynamic_reconfigure::ReconfigureResponse resp;

    // // Exemple : définir un paramètre spécifique, par exemple max_vel_x
    // dynamic_reconfigure::Config conf;
    // dynamic_reconfigure::DoubleParameter param_acc_lim_x;// Modifier dynamiquement l'accélération
    // dynamic_reconfigure::DoubleParameter param_max_vel_x;// Modifier dynamiquement la vitesse
    // dynamic_reconfigure::BoolParameter param_holonomic_robot;
    // param_max_vel_x.name = "max_vel_x";  // Paramètre que nous voulons changer
    // param_max_vel_x.value = 0.2;  // Nouvelle valeur
    // param_acc_lim_x.name = "acc_lim_x";  // Paramètre que nous voulons changer
    // param_acc_lim_x.value = 0.05;  // Nouvelle valeur
    // param_holonomic_robot.name = "holonomic_robot";  // Paramètre que nous voulons changer
    // param_holonomic_robot.value = true;  // Nouvelle valeur

    // // Ajouter le paramètre à la configuration
    // conf.doubles.push_back(param_acc_lim_x);
    // conf.doubles.push_back(param_max_vel_x);
    // conf.bools.push_back(param_holonomic_robot);

    // // Passer la configuration au client
    // req.config = conf;

    // // Envoyer la requête au serveur dynamic_reconfigure
    // if (client.setConfiguration(conf)) {
    //     ROS_INFO("Successfully updated parameters for move_base/DWAPlannerROS");
    // } else {
    //     ROS_ERROR("Failed to update parameters for move_base/DWAPlannerROS");
    // }

    /****************************************************************/
    /* fin de la configuration des paramètres dynamique de movebase */
    /****************************************************************/ 
    
    // déclenchement de la lecture de coordonnée cible après configuration de paramètre de déplacement
    managerParam.flagMapItem.setValue(STATUS_MOTION, true); // chargement de la donnée dans le gestionnaire de donnée manager
    managerParam.set_bool(n, STATUS_MOTION); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager

    // dans la boucle suivante nous vérifions que le robot est terminé sont déplacement rapide et que le temps d'action n'est pas terminé
    while (condition_RUN_MOVEBASE_FAST_MOTION(n)) 
    {
        ROS_INFO("robot is moving on movebase software for fast motion");
        
        // activation de la configuration du local planner de movebase
        n.setParam(MOVEBASE_CONFIG_FLAG, true);

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

    // activation de la configuration du local planner de movebase
    n.setParam(MOVEBASE_CONFIG_FLAG, true);
    
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


// void init_all_robot_param(ManagerParam& managerParamObj)
// {
//     bool value;

//     managerParamObj.flagMapItem.getValue(STATUS_GUI, value);
//     managerParamObj.flagMapItem.setValue(STATUS_GUI, value);

//     managerParamObj.flagMapItem.getValue(MOTION_MODE, value);
//     managerParamObj.flagMapItem.setValue(MOTION_MODE, value);

//     managerParamObj.flagMapItem.getValue(STATUS_MOTION, value);
//     managerParamObj.flagMapItem.setValue(STATUS_MOTION, value);

//     managerParamObj.flagMapItem.getValue(CONTROL_MOTION_END, value);
//     managerParamObj.flagMapItem.setValue(CONTROL_MOTION_END, value);

//     managerParamObj.flagMapItem.getValue(STATUS_MOVEBASE_END, value);
//     managerParamObj.flagMapItem.setValue(STATUS_MOVEBASE_END, value);

//     managerParamObj.flagMapItem.getValue(MOVEBASE_CONFIG_FLAG, value);
//     managerParamObj.flagMapItem.setValue(MOVEBASE_CONFIG_FLAG, value);

//     managerParamObj.flagMapItem.getValue(STATUS_CAM, value);
//     managerParamObj.flagMapItem.setValue(STATUS_CAM, value);

//     managerParamObj.flagMapItem.getValue(TASK_FINISHED, value);
//     managerParamObj.flagMapItem.setValue(TASK_FINISHED, value);

//     managerParamObj.flagMapItem.getValue(WARNING_ALARM, value);
//     managerParamObj.flagMapItem.setValue(WARNING_ALARM, value);

//     managerParamObj.flagMapItem.getValue(STOP_MOTION, value);
//     managerParamObj.flagMapItem.setValue(STOP_MOTION, value);
    

// }


void init_robot_param(ros::NodeHandle& nh)
{
    nh.setParam(STATUS_GUI, false);
    nh.setParam(MOTION_MODE, true);
    nh.setParam(STATUS_MOTION, false);
    nh.setParam(CONTROL_MOTION_END, true);
    nh.setParam(STATUS_MOVEBASE_END, true);
    nh.setParam(MOVEBASE_CONFIG_FLAG, true);
    nh.setParam(STATUS_CAM, false);
    nh.setParam(TASK_FINISHED, true);
    nh.setParam(WARNING_ALARM, false);
    nh.setParam(STOP_MOTION, false);
    nh.setParam(CURRENT_TIME, 0.0);

}

void init_all_telecom_msg(ros::nodeHandle& nh)
{
    nh.setParam(GUI_MSG_CHECK, false );
    nh.setParam(MOTOR_COMMAND_CHECK, false );
    nh.setParam(GUI_MSG, "" );
    nh.setParam(MOTOR_FEEDBACK_MSG, "" );
    nh.setParam(MOTOR_COMMAND_MSG, "" );
    nh.setParam(POSE_FEEDBACK_MSG, "" );
}

/*
speed : valeur de vitesse du déplacemnt du robot
- FAST_SPEED
- MEDIUM_SPEED
- LOW_SPEED
*/
void motion_speed(ros::nodeHandle& nh, std::string speed = MEDIUM_SPEED)
{
    std::string robot_json_motion_config = getenv("ROBOT_JSON_MOTION_CONFIG");// utilisation de la variable d'environnemnent ROBOT_JSON_PATH
    std::ifstream file_robot_json_motion_config(robot_json_motion_config);// chargement du fichier
    nlojmman::json data_json_motion_config;// conversion du fichier en élément nlomman::json
    file_robot_json_motion_config >> data_json_motion_config;// conversion du fichier en élément nlomman::json

    nh.setParam(MOVEBASE_CONFIG_FLAG + "/acc_lim_x", data_json_motion_config[speed]["acc_lim_x"]);
    nh.setParam(MOVEBASE_CONFIG_FLAG + "/max_vel_x", data_json_motion_config[speed]["max_vel_x"]);
    nh.setParam(MOVEBASE_CONFIG_FLAG + "/min_vel_x", data_json_motion_config[speed]["min_vel_x"]);
    nh.setParam(MOVEBASE_CONFIG_FLAG + "/max_vel_theta", data_json_motion_config[speed]["max_vel_theta"]);
    nh.setParam(MOVEBASE_CONFIG_FLAG + "/acc_lim_theta", data_json_motion_config[speed]["acc_lim_theta"]);
    nh.setParam(MOVEBASE_CONFIG_FLAG + "/xy_goal_tolerance", data_json_motion_config[speed]["xy_goal_tolerance"]);
    nh.setParam(MOVEBASE_CONFIG_FLAG + "/yaw_goal_tolerance", data_json_motion_config[speed]["yaw_goal_tolerance"]);
    nh.setParam(MOVEBASE_CONFIG_FLAG + "/min_in_place_vel_theta", data_json_motion_config[speed]["min_in_place_vel_theta"]);
    nh.setParam(MOVEBASE_CONFIG_FLAG + "/min_vel_theta", data_json_motion_config[speed]["min_vel_theta"]);
    nh.setParam(MOVEBASE_CONFIG_FLAG + "/xy_goal_tolerance", data_json_motion_config[speed]["xy_goal_tolerance"]);
    nh.setParam(MOVEBASE_CONFIG_FLAG + "/escape_vel", data_json_motion_config[speed]["escape_vel"]);
    nh.setParam(MOVEBASE_CONFIG_FLAG + "/holonomic_robot", data_json_motion_config[speed]["holonomic_robot"]);
    nh.setParam(MOVEBASE_CONFIG_FLAG + "/y_vels", data_json_motion_config[speed]["y_vels"]);
    nh.setParam(MOVEBASE_CONFIG_FLAG + "/latch_xy_goal_tolerance", data_json_motion_config[speed]["latch_xy_goal_tolerance"]);

    
}