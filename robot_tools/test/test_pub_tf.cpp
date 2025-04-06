#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publisher");
    ros::NodeHandle nh;
    tf::TransformBroadcaster broadcaster;
    ros::Rate rate(10.0); // 10 Hz

    while (ros::ok()) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(1.0, 1.0, 1.0)); // Translation (x=1, y=1, z=1)
        tf::Quaternion q;
        q.setRPY(0, 0, 0); // Pas de rotation
        transform.setRotation(q);

        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
