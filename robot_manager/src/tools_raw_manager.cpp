#include "robot_manager/tools_raw_manager.hpp"

/********************************************/
/*********                          *********/
/********************************************/

// Définition du constructeur
ListBool::ListBool() {
    // Initialisation du vecteur avec quelques éléments
    vectorListBool.push_back(false);

}

// Définition de la méthode pour obtenir la taille du vecteur
size_t ListBool::getSize() const {
    return vectorListBool.size();  // Renvoie la taille du vecteur
}

void ListBool::add() {
    vectorListBool.push_back(false);  // ajoute un élément de valeur false au vecteur
}

// Méthode pour supprimer le dernier élément au vecteur
void ListBool::remove(){
    vectorListBool.pop_back();  // supprime le dernier élément au vecteur
}

/********************************************/
/*********                          *********/
/********************************************/

// Méthode pour obtenir la valeur de la clé
bool FlagMap::getValue(const std::string& key,  bool &value) const{
    auto it = theMapBool.find(key);
    if (it != theMapBool.end()) {
      value = it->second;
        return true;  // Retourne la valeur associée à la clé
    } else {
        std::cerr << "Clé non trouvée!" << std::endl;
        // value = false;
        return false;  // Valeur par défaut pour les clés non trouvées
    }
}

bool FlagMap::getValue(const std::string& key,  double &value) const{
  auto it = theMapDouble.find(key);
  if (it != theMapDouble.end()) {
    value = it->second;// Retourne la valeur associée à la clé
      return true;  
  } else {
      std::cerr << "Clé non trouvée!" << std::endl;
      value = 0;// Valeur par défaut pour les clés non trouvées
      return false;  
  }
}

bool FlagMap::getValue(const std::string& key,  int &value) const{
  auto it = theMapInt.find(key);
  if (it != theMapInt.end()) {
    value = it->second;// Retourne la valeur associée à la clé
      return true;  
  } else {
      std::cerr << "Clé non trouvée!" << std::endl;
      value = 0;// Valeur par défaut pour les clés non trouvée
      return false;
  }
}

bool FlagMap::getValue(const std::string& key,  std::string &value) const{
  auto it = theMapString.find(key);
  if (it != theMapString.end()) {
    value = it->second;// Retourne la valeur associée à la clé
      return true;  
  } else {
      std::cerr << "Clé non trouvée!" << std::endl;
      value = "error_404";// Valeur par défaut pour les clés non trouvées
      return false;  
  }
}

// Méthode pour ajouter/modifier un élément dans le gestionnaire de donnée
void FlagMap::setValue(const std::string& key, bool value){
    theMapBool[key] = value;
}

void FlagMap::setValue(const std::string& key, double value){
  theMapDouble[key] = value;
}

void FlagMap::setValue(const std::string& key, int value){
  theMapInt[key] = value;
}

void FlagMap::setValue(const std::string& key, std::string value){
  theMapString[key] = value;
}

// Méthode pour supprimer un élément dans le gestionnaire de donnée
void FlagMap::remove_bool(const std::string& key){
    theMapBool.erase(key);
}

void FlagMap::remove_double(const std::string& key){
  theMapDouble.erase(key);
}

void FlagMap::remove_int(const std::string& key){
  theMapInt.erase(key);
}

void FlagMap::remove_string(const std::string& key){
  theMapString.erase(key);
}

/********************************************/
/*********                          *********/
/********************************************/


DynamicReconfigureClient::DynamicReconfigureClient()
{
  // Crée un client pour 'TrajectoryPlannerROS' (ou le contrôleur que vous utilisez)
  client_ = new Client("/move_base/TrajectoryPlannerROS", ros::Duration(30));
}

DynamicReconfigureClient::~DynamicReconfigureClient()
{
  delete client_;
}

// Fonction pour mettre à jour la vitesse maximale en dynamique
void DynamicReconfigureClient::updateMaxVel(double new_max_vel, double new_max_vel_theta)
{
  Config cfg;
  // Mettre à jour le paramètre max_vel_x et max_vel_y
  cfg.introspect("max_vel_x", new_max_vel);
  client_->updateConfiguration(cfg);
  cfg.introspect("max_vel_y", new_max_vel);
  client_->updateConfiguration(cfg);
  new_max_vel = (-1)*new_max_vel;
  cfg.introspect("min_vel_y", new_max_vel);
  new_max_vel = (-1)*new_max_vel;
  client_->updateConfiguration(cfg);
  cfg.introspect("max_vel_theta", new_max_vel_theta);
  client_->updateConfiguration(cfg);
  ROS_INFO("Vitesse maximale modifiée à : %f m/s et %f rad/s", new_max_vel, new_max_vel_theta);
}

// Fonction pour mettre à jour l'accélération maximale en dynamique
void DynamicReconfigureClient::updateMaxAccel(double new_max_accel, double new_max_accel_theta)
{
  Config cfg;
  // Mettre à jour le paramètre max_accel_x
  cfg.introspect("acc_lim_x", new_max_accel);
  client_->updateConfiguration(cfg);
  cfg.introspect("acc_lim_y", new_max_accel);
  client_->updateConfiguration(cfg);
  cfg.introspect("max_decel", new_max_accel);
  client_->updateConfiguration(cfg);
  cfg.introspect("acc_lim_theta", new_max_accel_theta);
  client_->updateConfiguration(cfg);
  ROS_INFO("Accélération maximale modifiée à : %f m/s/s et %f rad/s/s", new_max_accel, new_max_accel_theta);
}





/********************************************/
/*********                          *********/
/********************************************/


// Méthode de mise à jour pour obtenir la valeur du paramètre dans le système ROS dans le gestionnaire de donnée manager
void ManagerParam::get_bool(ros::NodeHandle& nh, const std::string& name) const{
  bool value;
  nh.getParam(name, value);
  flagMapItem.setValue(name, value);
}
void ManagerParam::get_string(ros::NodeHandle& nh, const std::string& name) const{
  std::string value;
  nh.getParam(name, value);
  flagMapItem.setValue(name, value);
}
void ManagerParam::get_double(ros::NodeHandle& nh, const std::string& name) const{
  double value;
  nh.getParam(name, value);
  flagMapItem.setValue(name, value);
}
void ManagerParam::get_int(ros::NodeHandle& nh, const std::string& name) const{
  int value;
  nh.getParam(name, value);
  flagMapItem.setValue(name, value);
}

// Méthode de mise à jour pour ajouter/modifier la valeur du paramètre dans le système ROS dans le gestionnaire de donnée manager
void ManagerParam::set_bool(ros::NodeHandle& nh, const std::string& name){
  bool value;
  flagMapItem.setValue(name, value);
  nh.setParam(name, value);
}
void ManagerParam::set_string(ros::NodeHandle& nh, const std::string& name){
  std::string value;
  flagMapItem.setValue(name, value);
  nh.setParam(name, value);
}
void ManagerParam::set_double(ros::NodeHandle& nh, const std::string& name){
  double value;
  flagMapItem.setValue(name, value);
  nh.getParam(name, value);
}
void ManagerParam::set_int(ros::NodeHandle& nh, const std::string& name){
  int value;
  flagMapItem.setValue(name, value);
  nh.getParam(name, value);
}


