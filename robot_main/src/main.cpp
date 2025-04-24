#include "robot_main/main.hpp"




int main(int argc, char** argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle n;
    ros::Rate loopRate(_5_MILLISECONDS);

    init_robot_param(n);
    init_all_telecom_msg(n);

    starting_detected(); // détection de l'activation de système

    init_timer(n);// initialisation des timers

    bool value; // value - valeur de condition
    double currentTime = 0;

    // boucle principale permettant le fonctionnement des timers en arrière plan
    do
    {   
        managerParam.get_bool(n, STOP_MOTION); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager
        managerParam.flagMapItem.getValue(STOP_MOTION, value); // chargement de la donnée à partir du gestionnaire de donnée manager
        ros::spinOnce();
        loopRate.sleep();
        
        currentTime = currentTime + _5_MILLISECONDS;
        managerParam.flagMapItem.setValue(CURRENT_TIME, currentTime); // chargement de la donnée à partir du gestionnaire de donnée manager
        managerParam.set_double(n, CURRENT_TIME); // mise à jour des données à partir du système ROS dans le gestionnaire de donnée manager
    
    }while(value);
    

    ROS_INFO("time is over \t BYE BYE !!!!! \n robot turning off");

    return 0;
}

/*
fonction pour détecter le lancement manuel
*/
void starting_detected()
{

}

/*
fonction pour initialiser les timers
*/
void init_timer(ros::NodeHandle& nh)
{
    // Création de deux timers avec des périodes différentes
    timer_Warning_Time_Alarm = nh.createTimer(ros::Duration(MATCH_DURATION_IN_SECOND - ALERT_FLAG_DURATION_IN_SECOND), timerCallback_Warning_Time_Alarm, false, false); // 80 secondes
    timer_Stop_Time_Alarm = nh.createTimer(ros::Duration(MATCH_DURATION_IN_SECOND), timerCallback_Stop_Time_Alarm, false, false); // 90 secondes
}
