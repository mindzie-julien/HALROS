#include "robot_manager/general_manager.hpp"


// Alias pour simplifier le code
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


nlohmann::json target_position;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "movebase_manager");
    ros::NodeHandle n;
    ros::Rate loopRate(2 * _5_MILLISECONDS);
   
    std::string robot_json_location = getenv("ROBOT_JSON_LOCATION");// utilisation de la variable d'environnemnent ROBOT_JSON_LOCATION
    std::ifstream file_robot_json_location(robot_json_location);// chargement du fichier
    nlojmman::json data_json_location;// conversion du fichier en élément nlomman::json
    file_robot_json_location >> data_json_location;// conversion du fichier en élément nlomman::json

    std_msgs::String ros_msg;

    bool state, warning, stop = false;
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    ros::Subscriber json_target_sub = n.subscribler(TARGET_PATH_TOPIC, 5, callback_json_target_sub)

    // Création du client pour interagir avec le serveur move_base
    MoveBaseClient ac("move_base", true);

    // Définir un goal
    move_base_msgs::MoveBaseGoal goal;

    while (ros::ok() && !stop) // Boucle pour determiner le point cible dans le fichier json
    {
        nlojmman::json json_data;
        
        ac.waitForServer();
        // target_position

        
        for (auto& [key, value] : data_json_path.items())
        {
            if((key != "team") && (key != "end"))
            {
                json_data = target_position[key];
            }
        }
        if(json_data.contains("AC_E"))
        {
            ;
        }
        else
        {
            for (auto& [key, value] : json_data.items())
            {
                goal.target_pose.header.frame_id = "map";  // ou "odom", "base_link" selon ta config
                goal.target_pose.header.stamp = ros::Time::now();
                
                for (auto& [key_id, value_id] : value.items())
                {
                    // Position à atteindre
                    goal.target_pose.pose.position.x = data_json_location[ target_position["team"] ][ key ][ key_id ]["x"];
                    goal.target_pose.pose.position.y = data_json_location[ target_position["team"] ][ key ][ key_id ]["y"];
                    goal.target_pose.pose.position.z = 0.0;
                    double angle_deg = data_json_location[ target_position["team"] ][ key ][ key_id ]["angle"]
                    
                    double angle = angle_deg*(M_PI / 180); // 45 degrés en radians

                    // Définir l'axe de rotation (par exemple, autour de l'axe Z)
                    tf::Vector3 axis(0.0, 0.0, 1.0);  // Axe Z

                    // Créer un quaternion à partir de l'angle et de l'axe
                    tf::Quaternion quaternion;
                    quaternion.setRotation(axis, angle);  // rotation autour de l'axe Z
                    // Orientation en quaternion
                    goal.target_pose.pose.orientation.x = quaternion.x();
                    goal.target_pose.pose.orientation.y = quaternion.y();
                    goal.target_pose.pose.orientation.z = quaternion.z();
                    goal.target_pose.pose.orientation.w = quaternion.w();

                    // Envoyer le goal
                    ROS_INFO("Envoi du goal...");
                    ac.sendGoal(goal);
                    // Attendre le résultat (optionnel : avec timeout)
                    ac.waitForResult();
                }
                
            }
        }
        n.getParam(WARNING_ALARM, warning);
        n.getParam(STATUS_MOVEBASE_END, state);
        n.getParam(STOP_MOTION, stop);
        if(warning)
        {
            ;
        }
        else
        {
            
        }
        
        ros::spinOnce();
        loopRate.sleep();
    }

    ROS_INFO("movebase manager is over \t BYE BYE !!!!! \n ");

    return 0;
}


void callback_json_target_sub(const std_msgs::String& ros_msg)
{
    std::string data = ros_msg.data ;
    target_position = nlomman::json::parse(data);
    
}

