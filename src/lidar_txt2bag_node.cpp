#include "ros/ros.h"
#include <rosbag/bag.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include <fstream>
#include <vector>

void parseLidarFromFile(std::string str, long long &timestamp, std::vector<long> &vec);

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "lidar_txt2bag");
    ros::NodeHandle nh;
    std::string node_name = ros::this_node::getName();

    ros::NodeHandle _nh("~");
    std::string folder_aux, name;
    _nh.getParam("file", folder_aux);
    _nh.getParam("name", name);
    ROS_INFO_STREAM("Got param: " << folder_aux);

    sensor_msgs::LaserScan lidar_msg;
    std_msgs::Header header;
    lidar_msg.angle_min = -3*M_PI/4;
    lidar_msg.angle_max = 3*M_PI/4;
    lidar_msg.angle_increment = M_PI/720;

    ros::Publisher pub_scan = nh.advertise<sensor_msgs::LaserScan>("scan",1);
    rosbag::Bag bag;
    bag.open((name+".bag").c_str(), rosbag::bagmode::Write);

    // Files
	std::fstream log_file, perception_file, aux_file;
	std::string file_timestamp, perception_name, aux_line;
	std::string folder = std::string(folder_aux), folderpath;
	std::vector<std::string> daylist, folderlist, filelist;

	//LIDAR
	std::ifstream lidar_input_file;
	std::string lidar_line, complete_lidar_name, lidar_name, lidar_folder;
	std::vector<long> lidar_readings;
    long long current_ts = 0, previous_lidar_ts = 0;

    // General
	std::size_t found;
    int n_done = 0, loop;
    bool b_continue = true;

    lidar_input_file.open(folder_aux.c_str(), std::fstream::in);
    if(!lidar_input_file.is_open()) 
    {
        ROS_INFO_STREAM( "\n\n\nCheck " << complete_lidar_name << "...\n\n\n" );
        exit(1);
    }

    current_ts = 0;
    loop = 1;
    double nsecs;
    while(loop)
    {		
        if (!getline(lidar_input_file, lidar_line)) 
        {
            loop = 0;
        }
        previous_lidar_ts = current_ts;
        parseLidarFromFile(lidar_line, current_ts, lidar_readings);         
        if(previous_lidar_ts != current_ts)	
        {                   
            if(n_done == 0) previous_lidar_ts = current_ts;
            lidar_msg.ranges.clear();
            for(int i = 0; i < lidar_readings.size(); i++)
            {
                lidar_msg.ranges.push_back(lidar_readings[i]*0.001);
            }
            header.stamp.sec = (int) current_ts*0.001;

            nsecs = (current_ts*0.001 - header.stamp.sec)*pow(10,9) ;
            header.stamp.nsec = nsecs;
            lidar_msg.header = header;

            pub_scan.publish(lidar_msg);
            ROS_INFO_STREAM(previous_lidar_ts << "\t" << current_ts << "\t" << (current_ts-previous_lidar_ts)*pow(10,-3) << " nsecs: " << nsecs << " sec: " << header.stamp.sec << " current_ts/1000: " << current_ts*0.001 );

            bag.write("scan", ros::Time::now(), lidar_msg);
            ros::Duration((current_ts-previous_lidar_ts)*pow(10,-3)).sleep();
            n_done++;      
        }          
    }
    bag.close();
    ROS_INFO_STREAM("Done");
    lidar_input_file.close();

    return 0;
}

void parseLidarFromFile(std::string str, long long &timestamp, std::vector<long> &vec)
{
    std::vector<long long> aux;
    std::string delimiter = "|", token;
    size_t pos = 0;
	int h, m, s;
    if(str.find(delimiter) > str.size())
    {
        delimiter = ",";
    }

    while((pos=str.find(delimiter)) != std::string::npos)
    {
        token = str.substr(0,pos);
		aux.push_back(atoll(token.c_str()));
        str.erase(0, pos + delimiter.length() );
    }
    aux.push_back(atoi(str.c_str()));
    vec.clear();
    if (aux.size()==1083)
    {
        h = std::floor(aux[1]/10000);
        m = std::floor(aux[1]/100) - 100*h;
        s = aux[1] - 10000*h - 100*m;
        //std::cout << "\nh: " << h << " m: " << m << " s: " << s;
        timestamp = (h*3600 + m*60 + s)*1000 + aux[2]  ;
        for (int i = 3; i < aux.size(); i++ )
        {
            vec.push_back(aux[i]);
        }
    }
    else if(aux.size()==1081)
    {
		timestamp = aux[0];
		//std::cout << "\nts: " << timestamp;
        for (int i = 1; i < aux.size(); i++ )
        {
            vec.push_back(aux[i]);
        }
    }
}