#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.client import Client

import config

def reconfigure_local_planner():
    
    
    
    # vérification du mode de déplacement
    MOTION_MODE = True#rospy.get_param(config.MOTION_MODE)
    # vérification de la validation de configuration du robot
    MOVEBASE_CONFIG_FLAG = True#rospy.get_param(config.MOVEBASE_CONFIG_FLAG)

    if (MOVEBASE_CONFIG_FLAG and MOTION_MODE):
        # Liste des paramètres à modifier
        new_config = {
            'acc_lim_x': config.fast_acc_lim_x,
            'max_vel_x': config.fast_max_vel_x,
            'min_vel_x': config.min_vel_x,
            'max_vel_theta': config.fast_max_vel_theta,
            'acc_lim_theta': config.fast_acc_lim_theta,
            'xy_goal_tolerance': config.xy_goal_tolerance,
            'yaw_goal_tolerance': config.yaw_goal_tolerance,
            "min_in_place_vel_theta": config.min_in_place_vel_theta,
            'min_vel_theta': config.min_vel_theta,
            'xy_goal_tolerance': config.xy_goal_tolerance,
            'escape_vel': config.escape_vel,
            'holonomic_robot': config.holonomic_robot,
            'y_vels': config.y_vels,
            'latch_xy_goal_tolerance': config.latch_xy_goal_tolerance
        }
        
        # désactivation de la configuration du local planner de movebase
        rospy.set_param(config.MOVEBASE_CONFIG_FLAG, False)

    elif(MOVEBASE_CONFIG_FLAG and (not MOTION_MODE)):
        # Liste des paramètres à modifier
        new_config = {
            'acc_lim_x': config.slow_acc_lim_x,
            'max_vel_x': config.slow_max_vel_x,
            'min_vel_x': config.min_vel_x,
            'max_vel_theta': config.slow_max_vel_theta,
            'acc_lim_theta': config.slow_acc_lim_theta,
            'xy_goal_tolerance': config.xy_goal_tolerance,
            'yaw_goal_tolerance': config.yaw_goal_tolerance,
            "min_in_place_vel_theta": config.min_in_place_vel_theta,
            'min_vel_theta': config.min_vel_theta,
            'xy_goal_tolerance': config.xy_goal_tolerance,
            'escape_vel': config.escape_vel,
            'holonomic_robot': config.holonomic_robot,
            'y_vels': config.y_vels,
            'latch_xy_goal_tolerance': config.latch_xy_goal_tolerance
        }

        # désactivation de la configuration du local planner de movebase
        rospy.set_param(config.MOVEBASE_CONFIG_FLAG, False)

    # Nom du noeud du planner local (vérifie bien dans rqt_graph ou avec rosnode list)
    client = Client('/move_base/TrajectoryPlannerROS', timeout=1)
    configServer = client.update_configuration(new_config)
    rospy.loginfo("Paramètres mis à jour : %s", configServer)

if __name__ == '__main__':
    
    rospy.init_node('dynamic_reconfigure_movebase_local_planner', anonymous=False)
    flag = 0
    while not rospy.is_shutdown():

        STOP_MOTION = False
        MOVEBASE_CONFIG_FLAG = False
        pauseLoop = rospy.Rate(1)
        try:
            # rospy.init_node('')
            count = 0
            pause = rospy.Rate(1000)
            while True:
                reconfigure_local_planner()
                # détermination de d'état d'arrêt du robot
                if rospy.has_param(config.STOP_MOTION):
                    count = 0
                    flag = 0
                    STOP_MOTION = rospy.get_param(config.STOP_MOTION)
                    if STOP_MOTION:
                        break 
                else:
                    rospy.logwarn("Le paramètre STOP_MOTION n'existe pas.")
                    count += 1
                    if (count == 5000):
                        break          
                if (rospy.has_param(config.MOVEBASE_CONFIG_FLAG) and rospy.has_param(config.MOTION_MODE)):
                    count = 0
                    flag = 0
                    reconfigure_local_planner()
                else:
                    rospy.logwarn("Le paramètre MOVEBASE_CONFIG_FLAG ou MOTION_MODE n'existe pas.")
                    count += 1
                    if (count == 5000):
                        break  

                # temps de repos de la boucle
                pause.sleep()

        except rospy.ROSInterruptException:
            rospy.logwarn("un problème de reconfiguration des paramètres du local_planner de movebase est survenu.")
            pass

        flag += 1
        if(flag == 120):
            rospy.logwarn("un problème est survenu dans movebase.py .")
            break
        pauseLoop.sleep()
