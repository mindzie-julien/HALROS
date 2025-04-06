#ifndef ADDONS_H
#define ADDONS_H

#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <ros/ros.h>
#include <termios.h>
#include <std_msgs/Bool.h>
#include <nlohmann/json.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>

#define SPEED_FILTER_LIMIT_VALUE 0.05


std::string getDataFromJson(const std::string& jsonString, const std::string& key);

std::string convertStringIntoJson(const std::vector<std::string>& keys, const std::vector<std::string>& values);

std::string floatToString(float value);

float stringToFloat(const std::string& str);

geometry_msgs::Pose2D createPose2D(double x, double y, double theta);

std::vector<geometry_msgs::Pose2D> createPose2DList();

std::string pose2DListToJson(const std::vector<geometry_msgs::Pose2D>& pose_list);

geometry_msgs::Pose2D getPose2DFromJson(const std::string& json_str, size_t index);

std::string getUserInputAndConvertToJson();

void tfUpdate(const nav_msgs::Odometry::ConstPtr& msg);

bool sendUART(std::string& data, std::string& device, int baudrate);

bool receiveUART(std::string& data, std::string& device, int baudrate);

geometry_msgs::PointStamped getPositionPointFromParentFrame(std::string parentFrameName, std::string childFrameName, tf::Vector3 pointInChildFrame);

double computeDistance(double x2, double y2, double z2, double x1=0, double y1=0, double z1=0);




#endif
