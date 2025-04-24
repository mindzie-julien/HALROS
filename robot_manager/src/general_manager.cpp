#include "robot_manager/general_manager.hpp"
// #include <ros/ros.h>
// #include <dynamic_reconfigure/client.h>
// #include <base_local_planner/BaseLocalPlannerConfig.h>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <move_base_msgs/MoveBaseActionGoal.h>
// #include <actionlib/client/simple_action_client.h>
// #include <std_msgs/String.h>
// #include <nav_msgs/Odometry.h>
// #include <tf/transform_datatypes.h>
// #include <geometry_msgs/Quaternion.h> 
// #include <geometry_msgs/PointStamped.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/point_cloud2_iterator.h>

nlohmann::json target_position;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "general_manager");
    ros::NodeHandle n;
    ros::Rate loopRate(2 * _5_MILLISECONDS);

    ros::Timer timer1;
    timer1 = n.createTimer(ros::Duration(TASK_DURATION_IN_SECOND), timerCallback_Waiting_Camera_Capture, false, false); // 2 secondes
   
    std::string robot_json_path = getenv("ROBOT_JSON_PATH");// utilisation de la variable d'environnemnent ROBOT_JSON_PATH
    std::ifstream file_robot_json_path(robot_json_path);// chargement du fichier
    nlojmman::json data_json_path;// conversion du fichier en élément nlomman::json
    file_robot_json_path >> data_json_path;// conversion du fichier en élément nlomman::json
    
    ros::Publisher _gui_pub_ = n.advertise<std_msgs::String>(TARGET_PATH_TOPIC);
    std_msgs::String ros_msg;

    bool state, stop;
    // ros::Subscriber json_target_sub = n.subscribler(TARGET_PATH_TOPIC, 5, callback_json_target_sub)

    // std::string team = json_target_sub[];
    int size_target_position = target_position["path"].size();
    while (ros::ok() && GUI_ACTIVATED) 
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
    
    while (ros::ok() && !GUI_ACTIVATED && !stop) // Boucle pour determiner le point cible dans le fichier json
    {
        nlojmman::json json_data;
        
        // target_position
        for (auto& [key, value] : data_json_path.items()) {
            // std::cout << key << " : " << value << std::endl;
            
            if(key != "team")
            {
                do
                {
                    n.getParam(WARNING_ALARM, stop);
                    n.getParam(STATUS_MOVEBASE_END, state);
                    
                } while (state == false); // vérification du déclenchement du mouvement
                json_data[key] = value; // chargement des données de position
                ros_msg.data = json_data.dumps();
                _gui_pub_.publish(ros_msg);
            }
            else
            {
                json_data["team"] = data_json_path["team"];// récupération de l'équipe
                json_data["end"] = data_json_path["end"];// récupération de la zone d'arrivé
            }
            
            n.getParam(WARNING_ALARM, stop);
            
        }

        ros::spinOnce();
        loopRate.sleep();
    }

    ROS_INFO("general manager is over \t BYE BYE !!!!! \n ");

    return 0;
}


void callback_json_target_sub(const std_msgs::String& ros_msg)
{
    std::string data = ros_msg.data ;
    target_position = nlomman::json::parse(data);
    
}

