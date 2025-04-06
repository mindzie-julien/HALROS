#include "robot_tools/tools.hpp"
#include <iterator>



std::string convertStringIntoJson(const std::vector<std::string>& keys, const std::vector<std::string>& values) 
{
    try
    {
        // Vérifiez que les vecteurs ont la même taille
        if (keys.size() != values.size()) {
            throw std::invalid_argument("Keys and values must have the same length");
        }

        // Créez un objet JSON vide
        nlohmann::json jsonObject;

        // Remplissez l'objet JSON avec les clés et les valeurs
        for (size_t i = 0; i < keys.size(); ++i) {
            jsonObject[keys[i]] = values[i];
        }

        // Retourne la chaîne JSON sous forme de string
        return jsonObject.dump(); // retour ici après avoir construit le JSON

    } catch(const std::invalid_argument& e) {
        // En cas de problème avec les tailles des vecteurs
        std::cerr << "Argument error: " << e.what() << std::endl;
        ROS_ERROR("Argument error: %s", e.what());
        return "";  // Retourner une chaîne vide en cas d'erreur

    } catch(const std::exception& e) {
        // Pour attraper toute autre exception générique
        std::cerr << "Exception: " << e.what() << std::endl;
        ROS_ERROR("Exception: %s", e.what());
        return "";  // Retourner une chaîne vide en cas d'erreur
    }
}


std::string getDataFromJson(const std::string& jsonString, const std::string& key) 
{
    try {
        // Parse the JSON string
        nlohmann::json jsonObject = nlohmann::json::parse(jsonString);

        // Check if the key exists in the JSON object
        if (jsonObject.contains(key)) {
            // Return the value associated with the key
            return jsonObject[key].get<std::string>();
        } else {
            throw std::invalid_argument("Key not found in JSON");
        }
    } catch (const nlohmann::json::parse_error& e) {
        throw std::invalid_argument("Invalid JSON string");
    }
}

std::string floatToString(float value) 
{
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

float stringToFloat(const std::string& str) 
{
    std::istringstream iss(str);
    float value;
    iss >> value;
    return value;
}

geometry_msgs::Pose2D createPose2D(double x, double y, double theta)
{
    geometry_msgs::Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    return pose;
}

std::vector<geometry_msgs::Pose2D> createPose2DList()
{
    std::vector<geometry_msgs::Pose2D> pose_list;
    pose_list.push_back(createPose2D(1.0, 2.0, 0.5));
    pose_list.push_back(createPose2D(3.0, 4.0, 1.0));
    pose_list.push_back(createPose2D(5.0, 6.0, 1.5));
    return pose_list;
}

std::string pose2DListToJson(const std::vector<geometry_msgs::Pose2D>& pose_list)
{
    nlohmann::json j;
    for (size_t i = 0; i < pose_list.size(); ++i)
    {
        j[i]["x"] = pose_list[i].x;
        j[i]["y"] = pose_list[i].y;
        j[i]["theta"] = pose_list[i].theta;
    }
    return j.dump();
}

geometry_msgs::Pose2D getPose2DFromJson(const std::string& json_str, size_t index)
{
    nlohmann::json j = nlohmann::json::parse(json_str);
    geometry_msgs::Pose2D pose;
    pose.x = j[index]["x"].get<double>();
    pose.y = j[index]["y"].get<double>();
    pose.theta = j[index]["theta"].get<double>();
    return pose;
}

std::string getUserInputAndConvertToJson()
{
    double x, y, theta;

    try
    {
        

        // Demander à l'utilisateur d'entrer les données de position 2D
        std::cout << "Entrez la position x : ";
        if (!(std::cin >> x)) throw std::invalid_argument("Entrée invalide pour x");

        std::cout << "Entrez la position y : ";
        if (!(std::cin >> y)) throw std::invalid_argument("Entrée invalide pour y");

        std::cout << "Entrez l'angle theta : ";
        if (!(std::cin >> theta)) throw std::invalid_argument("Entrée invalide pour theta");

        
        
        // Créer un objet JSON et y ajouter les données
        nlohmann::json j;
        j["x"] = x;
        j["y"] = y;
        j["z"] = theta;

        // Convertir l'objet JSON en chaîne de caractères
        std::string json_str = j.dump();

        return json_str;
    }
    catch (const std::invalid_argument& e)
    {
        
        std::cerr << "Erreur : " << e.what() << std::endl;
        
        return "";
    }
}

bool sendUART(std::string& data, std::string& device, int baudrate) {
    int uart_filestream = open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_filestream == -1) {
        std::cerr << "Error - Unable to open UART. Ensure it is not in use by another application\n";
        return false;
    }

    struct termios options;
    tcgetattr(uart_filestream, &options);
    options.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart_filestream, TCIFLUSH);
    tcsetattr(uart_filestream, TCSANOW, &options);

    int count = write(uart_filestream, data.c_str(), data.length());
    if (count < 0) {
        std::cerr << "UART TX error\n";
        close(uart_filestream);
        return false;
    }

    close(uart_filestream);
    return true;
}


bool receiveUART(std::string& data, std::string& device, int baudrate) {
    int uart_filestream = open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_filestream == -1) {
        std::cerr << "Error - Unable to open UART. Ensure it is not in use by another application\n";
        return false;
    }

    struct termios options;
    tcgetattr(uart_filestream, &options);
    options.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart_filestream, TCIFLUSH);
    tcsetattr(uart_filestream, TCSANOW, &options);

    char buffer[256];
    int length = read(uart_filestream, (void*)buffer, sizeof(buffer) - 1);
    if (length < 0) {
        std::cerr << "UART RX error\n";
        close(uart_filestream);
        return false;
    } else if (length == 0) {
        std::cerr << "No data received\n";
        close(uart_filestream);
        return false;
    } else {
        buffer[length] = '\0';
        data = std::string(buffer);
    }

    close(uart_filestream);
    return true;
}

void tfUpdate(const nav_msgs::Odometry::ConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    // Récupérer les données d'odométrie
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
    double ox = msg->pose.pose.orientation.x;
    double oy = msg->pose.pose.orientation.y;
    double oz = msg->pose.pose.orientation.z;
    double ow = msg->pose.pose.orientation.w;

    // Définir la transformation pour base_link
    transform.setOrigin(tf::Vector3(x, y, z));
    q.setX(ox);
    q.setY(oy);
    q.setZ(oz);
    q.setW(ow);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

    // Définir et diffuser d'autres transformations
    geometry_msgs::TransformStamped transformStamped;

    // Transformation de base_link à footprint
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "footprint";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);
    br.sendTransform(transformStamped);

    // Transformation de base_link à lidar_frame
    transformStamped.child_frame_id = "lidar_frame";
    transformStamped.transform.translation.x = 0.2; // Exemple de décalage
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.1;
    br.sendTransform(transformStamped);

    // Transformation de base_link à sensor_frame
    transformStamped.child_frame_id = "sensor_frame";
    transformStamped.transform.translation.x = 0.1;
    transformStamped.transform.translation.y = 0.1;
    transformStamped.transform.translation.z = 0.2;
    br.sendTransform(transformStamped);

    // Transformation de base_link à camera_frame
    transformStamped.child_frame_id = "camera_frame";
    transformStamped.transform.translation.x = 0.3;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.5;
    br.sendTransform(transformStamped);
}

geometry_msgs::PointStamped getPositionPointFromParentFrame(std::string parentFrameName, std::string childFrameName, tf::Vector3 pointInChildFrame)
{
    tf::TransformListener listener;
    // ros::Duration(1.0).sleep();
    geometry_msgs::PointStamped point_parent_frame;
    geometry_msgs::PointStamped point_child_frame;
    
    point_child_frame.header.frame_id = "odom"; // childFrameName;
    point_child_frame.header.stamp = ros::Time();

    // point_parent_frame.header.frame_id = "map"; // childFrameName;
    // point_parent_frame.header.stamp = ros::Time();

    point_child_frame.point.x = pointInChildFrame.x();
    point_child_frame.point.y = pointInChildFrame.y();
    point_child_frame.point.z = pointInChildFrame.z();

    try
    {
        tf::StampedTransform transform;
        listener.lookupTransform("map", "odom", ros::Time(0), transform);
        // listener.waitForTransform("map", "odom", ros::Time(0), ros::Duration(3.0));
        // listener.waitForTransform(parentFrameName, childFrameName, ros::Time(0), ros::Duration(0.001));
        // listener.transformPoint(parentFrameName, point_child_frame, point_parent_frame);
        listener.transformPoint("map", point_child_frame, point_parent_frame);

        ROS_INFO("point_child_frame: (%.2f, %.2f. %.2f) -----> point_parent_frame: (%.2f, %.2f, %.2f) at time %.2f",
                point_child_frame.point.x, 
                point_child_frame.point.y, 
                point_child_frame.point.z,
                point_parent_frame.point.x, 
                point_parent_frame.point.y, 
                point_parent_frame.point.z, 
                point_parent_frame.header.stamp.toSec());
                // point_parent_frame.header.frame_id = parentFrameName;
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("Received an exception trying to transform a point from \"point_child_frame\" to \"point_parent_frame\": %s", ex.what());
    }

    return point_parent_frame;

}

double computeDistance(double x2, double y2, double z2, double x1, double y1, double z1) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

