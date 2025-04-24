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


// Méthode de mise à jour pour obtenir la valeur du paramètre dans le système ROS dans le gestionnaire de donnée manager
void ManagerParam::get_bool(ros::NodeHandle& nh, const std::string& name) {
  bool value;
  nh.getParam(name, value);
  flagMapItem.setValue(name, value);
}
void ManagerParam::get_string(ros::NodeHandle& nh, const std::string& name) {
  std::string value;
  nh.getParam(name, value);
  flagMapItem.setValue(name, value);
}
void ManagerParam::get_double(ros::NodeHandle& nh, const std::string& name) {
  double value;
  nh.getParam(name, value);
  flagMapItem.setValue(name, value);
}
void ManagerParam::get_int(ros::NodeHandle& nh, const std::string& name) {
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


