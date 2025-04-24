#include "robot_manager/telecom_manager.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "telecom_manager");
  ros::NodeHandle n;

  ROS_INFO("telecom manager is ready to work !");
  
  ros::Rate loopRate(_5_MILLISECONDS);

  ros::Subscriber cmd_vel_sub = n.subscribe(CMD_VEL_TOPIC, 5, callbackCmdVel);
  ros::Subscriber GUI_sub = n.subscribe(GUI_TX_TOPIC, 5, callbackGUI);
  ros::Publisher feedback_vel_pub = n.advertise<geometry_msgs::TwistStamped>(RAW_VELOCITY_TOPIC, 5);
  ros::Publisher feedback_pose_pub = n.advertise<geometry_msgs::PoseStamped>(RAW_POSE_TOPIC, 5);
  ros::Publisher feedback_actuator_pub = n.advertise<std_msgs::String>(ACTUATOR_FEEDBACK_TOPIC, 5);
  ros::Publisher GUI_pub = n.advertise<std_msgs::String>(GUI_RX_TOPIC, 5);

  do
  {
    // n.getParam(GUI_MSG, GUI_MSG_check);
    // n.getParam(MOTOR_COMMAND_MSG, MOTOR_COMMAND_MSG_check);
    
    // if(GUI_MSG_check)
    // {
    //     GUI_MSG_check = false;
    // }
    // if(MOTOR_COMMAND_MSG_check)
    // {
    //     chatter_pub.publish();
    //     MOTOR_COMMAND_MSG_check = false;
    // }

    std::string data;
    nlohmann::json jsonObject;
    try
    {
      receiveUART(data, "ttyAMA0", 115200);
      jsonObject = nlohmann::json::parse(data);
      
      if(jsonObject.contains(GUI_MSG))
      {
        data = jsonObject[GUI_MSG].dump();
        GUI_pub.publish(data);
      }
      if(jsonObject.contains(MOTOR_FEEDBACK_MSG))
      {
        nlohmann::json jsonData, twist, linear, angular, header;
        geometry_msgs::TwistStamped twistStamped;
        jsonData = jsonObject[MOTOR_FEEDBACK_MSG];
        twist = jsonData["twist"]; linear = twist["linear"]; angular = twist["angular"];
        header = jsonData["header"];
        twistStamped.twist.linear.x = linear["x"];
        twistStamped.twist.linear.y = linear["y"];
        twistStamped.twist.linear.z = linear["z"];
        twistStamped.twist.angular.x = angular["x"];
        twistStamped.twist.angular.y = angular["y"];
        twistStamped.twist.angular.z = angular["z"];
        twistStamped.header.seq = header["seq"];
        twistStamped.header.stamp = linear["stamp"];
        twistStamped.header.frame_id = linear["frame_id"];
        feedback_vel_pub.publish(twistStamped);
      }
      if(jsonObject.contains(POSE_FEEDBACK_MSG))
      {
        nlohmann::json jsonData, position, orientation;
        geometry_msgs::PoseStamped poseStamped;
        jsonData = jsonObject[POSE_FEEDBACK_MSG];
        position = jsonData["position"];
        orientation = jsonData["orientation"];
        poseStamped.position.x = position["x"];
        poseStamped.position.y = position["y"];
        poseStamped.position.z = position["z"];
        poseStamped.orientation.w = orientation["w"];
        poseStamped.orientation.x = orientation["x"];
        poseStamped.orientation.y = orientation["y"];
        poseStamped.orientation.z = orientation["z"];
        poseStamped.header.seq = header["seq"];
        poseStamped.header.stamp = header["stamp"];
        poseStamped.header.frame_id = header["frame_id"];
        feedback_pose_pub.publish(poseStamped);
      }
      if(jsonObject.contains(ACTUATOR_FEEDBACK_MSG))
      {
        data = jsonObject[ACTUATOR_FEEDBACK_MSG].dump();
        feedback_actuator_pub.publish(data);
      }

    }catch(const std::exception& e)
    {
      // Pour attraper toute autre exception générique
      std::cerr << "Exception: " << e.what() << std::endl;
      ROS_ERROR("Exception: %s", e.what());
      return 0;
    }

    ros::spinOnce();
    loopRate.sleep();
  }while(ros::ok())
  

  return 0;
}

void callbackGUI(std_msgs::String::constPtr& data)
{
  sendUART(data, "ttyAMA0", 115200);
  ROS_INFO("gui message send");
}

void callbackCmdVel(geometry_msgs::TwistStamped::constPtr& data)
{
  sendUART(data, "ttyAMA0", 115200);
  ROS_INFO("velocity commands message send");
}

std::string twist_to_json_string(const geometry_msgs::TwistStamped& data)
{
 
  try
    {
        // Créez un objet JSON vide
        nlohmann::json jsonObject, linear, angular, twist, header;

        // Remplissez l'objet JSON avec les clés et les valeurs
        linear["x"] = data->twist.linear.x;
        linear["y"] = data->twist.linear.y;
        linear["z"] = data->twist.linear.z;
        twist["linear"] = linear;
        angular["x"] = data->twist.angular.x;
        angular["y"] = data->twist.angular.y;
        angular["z"] = data->twist.angular.z;
        twist["angular"] = angular;
        jsonObject["twist"] = twist;
        header["seq"] = data->header.seq;
        header["stamp"] = data->header.stamp;
        header["frame_id"] = data->header.frame_id;
        jsonObject["header"] = header;
        
        // Retourne la chaîne JSON sous forme de string
        return jsonObject.dump(); // retour ici après avoir construit le JSON

    } catch(const std::exception& e) {
        // Pour attraper toute autre exception générique
        std::cerr << "Exception: " << e.what() << std::endl;
        ROS_ERROR("Exception: %s", e.what());
        return "";  // Retourner une chaîne vide en cas d'erreur
    }

}

geometry_msgs::PoseStamped json_string_to_pose(std::string data)
{
  geometry_msgs::PoseStamped poseStamped;
  try {
    // Parse the JSON string
    nlohmann::json jsonObject = nlohmann::json::parse(data);
    nlohmann::json position, orientation;

    // Check if the key exists in the JSON object
    if (jsonObject.contains("position")) {
        // Return the value associated with the key
        position = jsonObject["position"].get<nlohmann::json>();
        poseStamped.position.x = position["x"];
        poseStamped.position.y = position["y"];
        poseStamped.position.z = position["z"];
    }
    if (jsonObject.contains("orientation"))
    {
      // Return the value associated with the key
      orientation = jsonObject["orientation"].get<nlohmann::json>();
      poseStamped.orientation.w = orientation["w"];
      poseStamped.orientation.x = orientation["x"];
      poseStamped.orientation.y = orientation["y"];
      poseStamped.orientation.z = orientation["z"];

      return poseStamped;
    }
    else {
        throw std::invalid_argument("Key not found in JSON");
    }
} catch (const nlohmann::json::parse_error& e) {
    throw std::invalid_argument("Invalid JSON string");
}
}

geometry_msgs::TwistStamped json_string_to_vel(std::string data)
{
  geometry_msgs::TwistStamped twistStamped;
  try {
    // Parse the JSON string
    nlohmann::json jsonObject = nlohmann::json::parse(data);
    nlohmann::json header, twist, linear, angular;

    // Check if the key exists in the JSON object
    if (jsonObject.contains("header")) {
        // Return the value associated with the key
        header = jsonObject["header"].get<nlohmann::json>();
        TwistStamped.header.seq = header["seq"];
        TwistStamped.header.stamp = header["stamp"];
        TwistStamped.header.frame_id = header["frame_id"];
    }
    if (jsonObject.contains("twist"))
    {
      // Return the value associated with the key
      twist = jsonObject["twist"].get<nlohmann::json>();
      if (jsonObject.contains("linear"))
      {
        TwistStamped.twist.linear.z = linear["z"];
        TwistStamped.twist.linear.x = linear["x"];
        TwistStamped.twist.linear.y = linear["y"];
      }
      if (jsonObject.contains("angular"))
      {
        TwistStamped.twist.angular.z = angular["z"];
        TwistStamped.twist.angular.x = angular["x"];
        TwistStamped.twist.angular.y = angular["y"];
      }
      else {
        throw std::invalid_argument("Key (angular or linear) not found in JSON");
      }
      return TwistStamped;
    }
    else {
        throw std::invalid_argument("Key (twist or header) not found in JSON");
    }
} catch (const nlohmann::json::parse_error& e) {
    throw std::invalid_argument("Invalid JSON string");
}
}
