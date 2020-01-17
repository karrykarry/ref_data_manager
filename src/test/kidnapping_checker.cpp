/*
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


class KidnappingChecker{

	private:
		ros::Publisher pose_pub;
		ros::Subscriber ekf_sub;
		ros::Subscriber global_sub;


	public:
		KidnappingChecker(ros::NodeHandle n,ros::NodeHandle priv_nh);

		void ndtCallback(const nav_msgs::OdometryConstPtr& msg);
		void globallocalizationCallback(const geometry_msgs::PoseConstPtr& msg);
};


KidnappingChecker::KidnappingChecker(ros::NodeHandle n, ros::NodeHandle private_nh_)
{
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
	
	ekf_sub = n.subscribe("/EKF/result", 1, &KidnappingChecker::ndtCallback, this);
	global_sub = n.subscribe("/map2context_result", 1, &KidnappingChecker::globallocalizationCallback, this);

}




void 
KidnappingChecker::ndtCallback(const nav_msgs::OdometryConstPtr& msg)
{
}


void 
KidnappingChecker::globallocalizationCallback(const geometry_msgs::PoseConstPtr& msg)
{
	geometry_msgs::PoseStamped pose_;

	pose_.header.frame_id="/map";
	pose_.pose = *msg;

	pose_pub.publish(pose_);
}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "test_pc_save");
	ros::NodeHandle n;
	ros::NodeHandle private_nh_("~");
    ROS_INFO("\033[1;32m---->\033[0mtest_pc_save Started.");
	
	KidnappingChecker kidnappingchecker(n, private_nh_);
	
	ros::spin();
	return (0);
}




