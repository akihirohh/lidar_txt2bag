/*
 * Laser filter algorithm, to filter the leaves from a corn crop
 * using histogram approach
 * Developed by Mateus Valverde Gasparino
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <rosbag/bag.h>

// --------- Node Params: -----------
std::string fixed_frame, cloud_frame;
int         max_scans;
double angle_min_degree, angle_max_degree;

// --------- Global Variables: -----------
ros::Publisher *scanPtr;

rosbag::Bag bag;
// ------ Prototype functions: ---------

// ------- Callback functions: ---------
void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    // Scan treatment
    sensor_msgs::LaserScan aux;
    aux = *scan_in;
    aux.header.frame_id = "laser";

    scanPtr->publish(aux);
    bag.write("scan_laser", ros::Time::now(), aux);
    ROS_INFO("cb");
}

// -------- Main function: ----------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_filter");
    
    ros::NodeHandle n;
    ros::NodeHandle n_param ("~");
    
    std::string folder_aux, name;
    std::string point_cloud_topic; 
    // Params management:
    n_param.param<std::string>("fixed_frame", fixed_frame, "/map");
    n_param.param<std::string>("cloud_frame", cloud_frame, "base_link");
    n_param.param("max_scans", max_scans, 100);
    n_param.param("angle_min_degree", angle_min_degree, -135.0);
    n_param.param("angle_max_degree", angle_max_degree, 135.0);
    n_param.param<std::string>("point_cloud_topic", point_cloud_topic, "laser_point_cloud");
    n_param.param<std::string>("name", name, "/home/hiro/remapped_scan2");
    // Publishers:   
    ros::Publisher pub_scan = n.advertise<sensor_msgs::LaserScan>("scan_laser",1);
    bag.open((name+".bag").c_str(), rosbag::bagmode::Write);

    // Subscribers:
    ros::Subscriber laser_sub = n.subscribe("scan", 1, scanCallback);
    
    scanPtr = &pub_scan;

    ros::spin();
    
    return 0;
}