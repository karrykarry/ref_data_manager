#ifndef _TEST_PC_RUN_HPP_ 
#define _TEST_PC_RUN_HPP_

#include<iostream>
#include<vector>
#include<sstream>
#include<iomanip>
#include<fstream>

#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>

#include<tf/transform_datatypes.h>
#include<tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>


class Test_pc_run{

	private:
		ros::Publisher pc_pub;
		ros::Subscriber pose_sub;

		std::string file_dir, file_dir2, pc_file_dir, odom_file_dir, odom_list;
		std::string pc_file_name;
		
		std::vector <geometry_msgs::Point> pr_poses;
	
		std::vector<std::string> split(const std::string &str, char sep);
		void pr_list_pub();

	public:
		Test_pc_run(ros::NodeHandle n,ros::NodeHandle priv_nh);
		~Test_pc_run();

		bool callback_flag;
		void pc_publisher(const int num);
		void poseCallback(const geometry_msgs::PoseConstPtr &msg);

};

#endif







