#include "robot_manager/gui_tools_manager.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gui_manager");
  ros::NodeHandle n;

  // ros::ServiceServer service = n.advertiseService("gui_manager_server", callback_service);
  ROS_INFO("GUI manager is ready to work !");
  ros::Publisher gui_pub = n.advertise<std_msgs::String>(GUI_TX_TOPIC);
  ros::Subcriber gui_sub = n.subscriber(GUI_RX_TOPIC ,5,  callback_gui_sub);
  ros::Rate loopRate(_5_MILLISECONDS);

  bool stop_motion = false, warning_alarm = false, status_gui = false;

  try
  {
    while(ros::ok() && GUI_ACTIVATED)
    {
      n.getParam(STOP_MOTION, stop_motion);
      n.getParam(WARNING_ALARM, warning_alarm);
      // n.getParam(STATUS_GUI, status_gui);

      if(stop_motion == true)
      {
        
        // nlohmman::json jsonObject;
        // std::string data;
        // jsonObject[GUI_MSG] = END;
        // sendUART( data, "ttyAMA0", 115200);
      }
      else
      {
        ;
      }
      ros::spinOnce();
      loopRate.sleep();
    }

    // if(!GUI_ACTIVATED)
    // {
    //   // 
    //   std::string robot_json_path = getenv("ROBOT_JSON_PATH");// utilisation de la variable d'environnemnent ROBOT_JSON_PATH
    //   std::ifstream file_robot_json_path(robot_json_path);// chargement du fichier
    //   nlojmman::json data_json_path;// conversion du fichier en élément nlomman::json
    //   file_robot_json_path >> data_json_path;// conversion du fichier en élément nlomman::json
    //   robot_json_path = data_json_path.dumps();
    //   // robot_json_location = data_json_location.dumps();
    //   ros::Publisher _gui_pub_ = n.advertise<std_msgs::String>(TARGET_PATH_TOPIC);
    //   std_msgs::String ros_msg;
    //   ros_msg.data = robot_json_path
    //   _gui_pub_.publish(ros_msg);
    // }
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  catch (...) {
    std::cerr << "error : gui_manager ." << std::endl;
}
  
  
  

  return 0;
}

