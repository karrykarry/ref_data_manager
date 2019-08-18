#ifndef _ITST2DATA_PUB_HPP_
#define _ITST2DATA_PUB_HPP_ 
#include<iostream>
#include<vector>
#include<sstream>
#include<iomanip>
#include<fstream>

#include<ros/ros.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Float64MultiArray.h>
// #include<nav_msgs/Odometry.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>


class Itst2data_pub{

	private:
		ros::Publisher pr_num_vis_pub;
		ros::Publisher more_estimate_num_pub;
		ros::Publisher most_estimate_num_pub;
		
		ros::Subscriber best_score_sub;
		ros::Subscriber all_score_sub;

		std::string file_dir, file_dir2, file_dir3, file_name, file_ext;
		int num_candidate;

		std::vector <geometry_msgs::Point> pr_poses;

		visualization_msgs::Marker make_vis_marker(const double now_x, const double now_y, const double now_z, const int num, const int id_num);

		std::vector<std::string> split(const std::string &str, char sep);
		
		void pr_list_pub();
		
		void color_change(visualization_msgs::Marker& color, double r_val, double g_val, double b_val, double a_val);
		void best_num_vis(const int num);

		template<typename T> T MIN(T val_1, T val_2);

	public:
		Itst2data_pub(ros::NodeHandle n,ros::NodeHandle priv_nh);
		~Itst2data_pub();

		void bestscorecallback(const std_msgs::Int32ConstPtr &msg);
		void allscorecallback(const std_msgs::Float64MultiArrayConstPtr &msg);

};


template<typename T>
T Itst2data_pub::MIN(T val_1, T val_2){
	return val_1 < val_2 ? val_1 : val_2;
}





#endif





