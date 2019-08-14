#ifndef _ITST2SCAN_HPP_
#define _ITST2SCAN_HPP_ 
#include<iostream>
#include<vector>
#include<sstream>
#include<iomanip>
#include<fstream>

#include<ros/ros.h>
#include<std_msgs/Int32.h>
// #include<nav_msgs/Odometry.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>


class Itst2data_pub{

	private:
		ros::Publisher estimate_num_pub;
		ros::Publisher pr_num_vis_pub;
		
		ros::Subscriber best_num_sub;

		std::string file_dir, file_dir2, file_dir3, file_name, file_ext;

		std::vector <geometry_msgs::Point> pr_poses;

		visualization_msgs::Marker make_vis_marker(const double now_x, const double now_y, const double now_z, const int num, const int id_num);

		std::vector<std::string> split(const std::string &str, char sep);
		
		void pr_list_pub();
		void num_vis(const int num);

	public:
		Itst2data_pub(ros::NodeHandle n,ros::NodeHandle priv_nh);
		~Itst2data_pub();

		void itstcallback(const std_msgs::Int32ConstPtr &msg);
		// void flagCallback(const std_msgs::EmptyConstPtr &msg);

};

#endif





