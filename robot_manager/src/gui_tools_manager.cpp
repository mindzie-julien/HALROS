#include "robot_manager/gui_tools_manager.hpp"

void gui_request(std::string message)
{
    ;
}

bool callback_service(robot_manager::ServiceDataString::Request &req, robot_manager::ServiceDataString::Request &res)
{
    manager_to_GUI = req.request;
    res.response = GUI_to_manager;
}

void callback_gui_sub(const std_msgs::String::constPtr& msg)
{
  manager_to_GUI = msg->data;
}
