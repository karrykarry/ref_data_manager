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

		int file_count;
		int pose_true_cnt, yaw_true_cnt, all_true_cnt;

		bool diff_pose(const geometry_msgs::Pose est_pose);
		bool diff_yaw(const double yaw1, const double yaw2);
	
	public:
		Test_pc_run(ros::NodeHandle n,ros::NodeHandle priv_nh);
		~Test_pc_run();

		void pc_publisher(const int num);
		void poseCallback(const geometry_msgs::PoseConstPtr &msg);

		double deg2rad(const double yaw_){
			double yaw = yaw_;
			while(yaw >= M_PI) yaw -= 2.0*M_PI;
			while(yaw <= -M_PI) yaw += 2.0*M_PI;
			return yaw;
		}
};

#endif







