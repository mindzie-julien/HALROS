#include "robot_manager/general_manager.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "general_manager");
    ros::NodeHandle n;
    ros::Rate loopRate(2 * _5_MILLISECONDS);

    ros::Timer timer1;
    timer1 = n.createTimer(ros::Duration(TASK_DURATION_IN_SECOND), timerCallback_Waiting_Camera_Capture, false, false); // 2 secondes
   

    while (ros::ok()) 
    {
        get_all_bool_data_param_from_node_step(n);
        
        if(condition_AIM_TARGET(n))
        {
            
            // détection du mode de déplacement et du temps d'action du robot, dans la condition suivante le robot est entrain rouler de façon rapide
            if(condition_RUN_MOVEBASE_FAST_MOTION(n))
            {
                fast_motion_step(n);
                camera_detection_step(n, timer1);
            }

            else if(condition_RUN_MOVEBASE_SLOW_MOTION(n))
            {
                slow_motion_step(n);
            }
        
        }

        ros::spinOnce();
        loopRate.sleep();
    }

    ROS_INFO("general manager is over \t BYE BYE !!!!! \n ");

    return 0;
}

