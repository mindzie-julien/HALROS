#ifndef TOOLS_RAW_MANAGER_H
#define TOOLS_RAW_MANAGER_H

#include <fstream>
#include <format>
#include <iostream>
#include <cstdlib> // pour setenv, getenv
#include <type_traits>
#include <string>
#include <vector>
#include <map>
#include <nlohmann/json.hpp>
#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>
#include <base_local_planner/BaseLocalPlannerConfig.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h> 
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
// #include <robot_tools/tools.hpp>

#include "robot_manager/tools_id_manager.hpp"
#include "robot_tools/tools.hpp"

using namespace std;
using namespace dynamic_reconfigure;

class ListBool {
    public:
        // Déclaration du vector de booléens
        std::vector<bool> vectorListBool;
    
        // Constructeur
        ListBool();
    
        // Méthode pour obtenir la taille du vecteur
        size_t getSize() const;

        // Méthode pour ajouter un élément au vecteur
        void add();

        // Méthode pour supprimer un élément au vecteur
        void remove();
};

class FlagMap {
    public:
        
    
        // Méthode pour obtenir la valeur de la clé
        bool getValue(const std::string& key,  bool& value) const;
        bool getValue(const std::string& key,  double& value) const;
        bool getValue(const std::string& key,  int& value) const;
        bool getValue(const std::string& key,  std::string& value) const;

        // Méthode pour ajouter/modifier un élément au dictionnaire
        void setValue(const std::string& key, bool value);
        void setValue(const std::string& key, double value);
        void setValue(const std::string& key, int value);
        void setValue(const std::string& key, std::string value);

        // Méthode pour supprimer un élément au dictionnaire
        void remove_bool(const std::string& key);
        void remove_double(const std::string& key);
        void remove_int(const std::string& key);
        void remove_string(const std::string& key);

    private:
        // Création d'une map avec des clés de type int et des valeurs de type bool
        std::map<std::string, bool> theMapBool;
        std::map<std::string, double> theMapDouble;
        std::map<std::string, int> theMapInt;
        std::map<std::string, std::string> theMapString;
};

class ManagerParam {
    public:

        FlagMap flagMapItem; // gestionnaire de donnée

       // Méthode pour obtenir la valeur du paramètre
        void get_bool(ros::NodeHandle& nh, const std::string& name) ;
        void get_string(ros::NodeHandle& nh, const std::string& name) ;
        void get_double(ros::NodeHandle& nh, const std::string& name) ;
        void get_int(ros::NodeHandle& nh, const std::string& name) ;

        // Méthode pour ajouter/modifier la valeur du paramètre
        void set_bool(ros::NodeHandle& nh, const std::string& name);
        void set_string(ros::NodeHandle& nh, const std::string& name);
        void set_double(ros::NodeHandle& nh, const std::string& name);
        void set_int(ros::NodeHandle& nh, const std::string& names);
        

};

#endif