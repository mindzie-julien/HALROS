#ifndef TELECOM_MANAGER_H
#define TELECOM_MANAGER_H

#include "robot_tools/tools.hpp"
#include "robot_manager/tools_manager.hpp"

std::map<std::string, std::string> msgMap;

bool GUI_MSG_check;// variable pour autoriser l'envoi de données au GUI, true si l'envoi est validé, false sinon
bool MOTOR_COMMAND_MSG_check;// variable pour autoriser l'envoi de données de commandes aux moteurs, true si l'envoi est validé, false sinon

#endif // TELECOM_MANAGER_H