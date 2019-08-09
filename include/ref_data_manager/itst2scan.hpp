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
// #include<visualization_msgs/Marker.h>
// #include<visualization_msgs/MarkerArray.h>


using namespace std;

class Itst2scan{

	private:

		ros::Subscriber best_num_sub;

		string SAVE_PATH, W_DIR;
	
	public:
		Itst2scan(ros::NodeHandle n,ros::NodeHandle priv_nh);
		~Itst2scan();

		void itstcallback(const std_msgs::Int32ConstPtr &msg);
		// void flagCallback(const std_msgs::EmptyConstPtr &msg);

};

#endif





