import rospy
from dynamic_reconfigure.srv import SetParameters
from dynamic_reconfigure.config import Config
from dynamic_reconfigure.parameter_generator import *

def set_local_planner_parameters():
    rospy.wait_for_service('/move_base/TrajectoryPlannerROS/set_parameters')
    
    try:
        # Créer un client pour appeler le service set_parameters
        set_parameters = rospy.ServiceProxy('/move_base/TrajectoryPlannerROS/set_parameters', SetParameters)
        
        # Créer un objet Config contenant les nouveaux paramètres
        config = Config()
        # Ajouter des paramètres dans le config. Exemple ici pour "max_vel_x" (vitesse maximale en X)
        config.doubles = [Param(name="max_vel_x", value=0.5)]
        
        # Appeler le service pour mettre à jour les paramètres
        response = set_parameters(config)
        rospy.loginfo("Paramètres mis à jour avec succès !")
    except rospy.ServiceException as e:
        rospy.logerr("Erreur lors de l'appel du service : %s" % e)

if __name__ == "__main__":
    rospy.init_node('set_local_planner_parameters')
    set_local_planner_parameters()
