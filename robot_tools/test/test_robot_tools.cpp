#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

// Fonction qui effectue la transformation d'un point
geometry_msgs::PointStamped transformPoint(tf::TransformListener& listener, 
                                           const std::string& target_frame, 
                                           const geometry_msgs::PointStamped& point_in) {
    geometry_msgs::PointStamped point_out;
    try {
        listener.waitForTransform("map", "odom", ros::Time(0), ros::Duration(5.0));  // Attendre la disponibilité du TF
        listener.transformPoint(target_frame, point_in, point_out);
        // Afficher les résultats
        ROS_INFO("Point dans odom : (%.2f, %.2f, %.2f)", 
                  point_in.point.x, point_in.point.y, point_in.point.z);
        ROS_INFO("Point dans map : (%.2f, %.2f, %.2f)", 
                  point_out.point.x, point_out.point.y, point_out.point.z);

    } catch (tf::TransformException &ex) {
        ROS_ERROR("Erreur de transformation: %s", ex.what());
    }
    return point_out;
}

// Test pour vérifier la transformation
TEST(TFTransformTest, TransformPointFromOdomToMap) {
    // ros::NodeHandle nh;
    // tf::TransformListener listener;
    

    // // Définir un point dans "odom"
    // geometry_msgs::PointStamped point_odom;
    // point_odom.header.frame_id = "odom";
    // point_odom.header.stamp = ros::Time();
    // point_odom.point.x = 0.0;
    // point_odom.point.y = 0.0;
    // point_odom.point.z = 0.0;

    // ros::Duration(2.0).sleep();

    // // Transformer vers "map"
    // geometry_msgs::PointStamped point_map = transformPoint(listener, "map", point_odom);

    // // Attendu : le point doit être décalé par la transformation (offset de 1,2,0)
    // EXPECT_NEAR(point_map.point.x, 1.0, 0.01);
    // EXPECT_NEAR(point_map.point.y, 1.0, 0.01);
    // EXPECT_NEAR(point_map.point.z, 1.0, 0.01);
}

// Main du test
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_tf_transform");
    testing::InitGoogleTest(&argc, argv);
    ros::NodeHandle nh;
    tf::TransformListener listener;
    

    // Définir un point dans "odom"
    geometry_msgs::PointStamped point_odom;
    point_odom.header.frame_id = "odom";
    point_odom.header.stamp = ros::Time();
    point_odom.point.x = 0.0;
    point_odom.point.y = 0.0;
    point_odom.point.z = 0.0;

    ros::Duration(2.0).sleep();

    // Transformer vers "map"
    geometry_msgs::PointStamped point_map = transformPoint(listener, "map", point_odom);
    return RUN_ALL_TESTS();
}
