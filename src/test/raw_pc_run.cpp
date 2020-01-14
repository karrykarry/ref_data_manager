/*
 *
 * 逆周りの検証用のソースコード
 * 検証するのが 
 * use_sim_time trueを使っているからその使用になっている
 *
 * ~/data/bagfiles/infant$ rosbag play 2018-09-09-16-33-11.bag --clock -s 15 
 * でvelodyne_pointsを使わない用にする
 *
 * nodeの情報は~/Pictures/ros_catkin_ws/ref_data_manager/sample/~
 * を参照
 *
 * pcd のデータのみ
 * ~/Pictures/ros_catkin_ws/test/~にある
 *
 *
 *
 */

#include<iostream>
#include<vector>
#include<sstream>
#include<iomanip>
#include<fstream>

#include<ros/ros.h>
#include<std_msgs/Bool.h>
#include<sensor_msgs/PointCloud2.h>
#include<nav_msgs/Odometry.h>

#include<tf/transform_datatypes.h>
#include<tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>


class Test_pc_run{

	private:
		ros::Publisher pc_pub;
		ros::Publisher test_pc_pub;
		ros::Publisher odom_pub;
		ros::Subscriber pose_sub;
		ros::Subscriber pc_sub;
	
		int file_count;
		ros::Time now_time;

		sensor_msgs::PointCloud2 now_pc;
		sensor_msgs::PointCloud2 buffer_pc;

	public:
		Test_pc_run(ros::NodeHandle n,ros::NodeHandle priv_nh);

		void poseCallback(const geometry_msgs::PoseConstPtr &msgs);	
		void pcCallback(const sensor_msgs::PointCloud2ConstPtr &msgs);	

};


Test_pc_run::Test_pc_run(ros::NodeHandle n, ros::NodeHandle private_nh_)
{
	pc_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);
	test_pc_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points/test", 10);
	odom_pub = n.advertise<nav_msgs::Odometry>("/estimate_pose/odometry", 10);
	
	pose_sub = n.subscribe<geometry_msgs::Pose>("/map2context_result", 1, &Test_pc_run::poseCallback, this);
	
	pc_sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_", 1, &Test_pc_run::pcCallback, this);

}


void 
Test_pc_run::poseCallback(const geometry_msgs::PoseConstPtr &msgs){
	nav_msgs::Odometry odom;
	odom.header.stamp = now_time;
	odom.header.frame_id = "/map";
	odom.pose.pose = *msgs;
	odom_pub.publish(odom);
	
	buffer_pc.header.frame_id = "/context_estimate";
	test_pc_pub.publish(buffer_pc);

	now_pc.header.frame_id = "/context_estimate";
	pc_pub.publish(now_pc);
	
	buffer_pc = now_pc;
}



void 
Test_pc_run::pcCallback(const sensor_msgs::PointCloud2ConstPtr &msgs){
	static bool flag = false;

	now_pc = *msgs;

	if(!flag){
		buffer_pc = now_pc;
		pc_pub.publish(now_pc);
	}

	flag = true;
}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "test_pc_run");
	ros::NodeHandle n;
	ros::NodeHandle private_nh_("~");
    ROS_INFO("\033[1;32m---->\033[0mtest_pc_run Started.");
	
	Test_pc_run test_pc_run(n, private_nh_);

	ros::spin();

	return (0);
}



