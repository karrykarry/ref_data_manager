#ifndef _TEST_PC_RUN_HPP_ 
#define _TEST_PC_RUN_HPP_

#include<iostream>
#include<vector>
#include<sstream>
#include<iomanip>
#include<fstream>

#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Empty.h>

#include<tf/transform_datatypes.h>
#include<tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>


class Test_pc_run{

	private:
		ros::Publisher pc_pub;
		ros::Publisher score_pub;
		ros::Publisher test_pc_pub;
		ros::Publisher recover_pub;
		
		ros::Subscriber pose_sub;
		ros::Subscriber flag_sub;

		std::string file_dir, file_dir2, pc_file_dir, odom_file_dir, odom_list;
		std::string pc_file_name;
		bool IS_DATASET;
		
		std::vector <geometry_msgs::Point> pr_poses;
	
		std::vector<std::string> split(const std::string &str, char sep);
		void pr_list_pub();


		std::ofstream writing_file;
		std::vector<double> diff_x;
		std::vector<double> diff_y;
		std::vector<double> diff_theta;



		std_msgs::Int32 score;


		int file_count;
		int pose_true_cnt, yaw_true_cnt, all_true_cnt;
		bool nx_flag;

		bool diff_pose(const geometry_msgs::Pose est_pose);
		bool diff_yaw(const double yaw1, const double yaw2);
	
	public:
		Test_pc_run(ros::NodeHandle n,ros::NodeHandle priv_nh);
		~Test_pc_run();

		void pc_publisher(ros::Publisher pub, const int num);
		void poseCallback(const geometry_msgs::PoseConstPtr &msg);
		void flagCallback(const std_msgs::BoolConstPtr &msg);

		double deg2rad(const double yaw_){
			double yaw = yaw_;
			while(yaw >= M_PI) yaw -= 2.0*M_PI;
			while(yaw <= -M_PI) yaw += 2.0*M_PI;
			return yaw;
		}
};

#endif







