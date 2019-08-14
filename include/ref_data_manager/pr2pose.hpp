#ifndef _PR2POSE_HPP_
#define _PR2POSE_HPP_
#include<iostream>
#include<vector>
#include<sstream>
#include<iomanip>
#include<fstream>

#include<ros/ros.h>
#include<std_msgs/Empty.h>
#include<nav_msgs/Odometry.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

class PR2Pose{	

	private:
		ros::Publisher pr_num_vis_pub;

		ros::Subscriber odom_sub;
		ros::Subscriber flag_sub;
		
		geometry_msgs::Point now_pose;
		std::vector <geometry_msgs::Point> pr_poses;

		std::string file_dir, file_dir2, file_dir3, file_name, file_ext;
		std::string output_txtfile;
		std::ofstream writing_file;

		visualization_msgs::MarkerArray m_array;

		visualization_msgs::Marker text_vis(const int num);
	
	public:
		PR2Pose(ros::NodeHandle n,ros::NodeHandle priv_nh);
		~PR2Pose();	
		void odomCallback(const nav_msgs::OdometryConstPtr &msg);
		void flagCallback(const std_msgs::EmptyConstPtr &msg);

};

#endif




