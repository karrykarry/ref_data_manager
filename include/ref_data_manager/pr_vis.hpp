#ifndef _PR_VIS_HPP_
#define _PR_VIS_HPP_
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


using namespace std;

class PR_vis{

	private:
		ros::Publisher pr_num_vis_pub;

		ros::Subscriber odom_sub;
		ros::Subscriber flag_sub;
		
		geometry_msgs::Point now_pose;
		vector <geometry_msgs::Point> pr_poses;

		string output_txtfile;
		std::ofstream writing_file;

		visualization_msgs::MarkerArray m_array;

		visualization_msgs::Marker text_vis(const int num);
	
	public:
		PR_vis(ros::NodeHandle n,ros::NodeHandle priv_nh);
		~PR_vis();	
		void odomCallback(const nav_msgs::OdometryConstPtr &msg);
		void flagCallback(const std_msgs::EmptyConstPtr &msg);

};

#endif




