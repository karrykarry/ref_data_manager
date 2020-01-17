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

		std::string pc_file_name;
	
		int file_count;
		ros::Time now_time;

	public:
		Test_pc_run(ros::NodeHandle n,ros::NodeHandle priv_nh);

		void pc_publisher(const int num, const ros::Time now_time);
};


Test_pc_run::Test_pc_run(ros::NodeHandle n, ros::NodeHandle private_nh_):
	file_count(0), now_time(0)
{
	// pc_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_obstacles/itst", 10);
	pc_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points/slam", 10);

	pc_file_name = "/home/amsl/Pictures/ros_catkin_ws/ref_data_manager/sample/pcd";
}



void
Test_pc_run::pc_publisher(const int num, const ros::Time now_time_){
	
	std::ostringstream oss;
	oss << std::setfill( '0' ) << std::setw( 3 ) << num;

	std::string pcd_name; 
	pcd_name = pc_file_name + "/" + oss.str() + ".pcd";

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_IN (new pcl::PointCloud<pcl::PointXYZI>);
	if( pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_name,*cloud_IN) == -1 ){
		std::cerr << "pcd load error !!"<<std::endl;;
		exit(1);
	}
	
	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(*cloud_IN, pc);

	pc.header.frame_id  = "/velodyne";	
	pc.header.stamp  = now_time_;
	pc_pub.publish(pc);
}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "test_pc_save");
	ros::NodeHandle n;
	ros::NodeHandle private_nh_("~");
    ROS_INFO("\033[1;32m---->\033[0mtest_pc_save Started.");
	
	Test_pc_run test_pc_run(n, private_nh_);
	
	int count=0;
    ros::Rate loop_rate(2);
	while(ros::ok()){
		static bool flag = false;
		if(flag){
			std::cout<<count<<std::endl;
			test_pc_run.pc_publisher(count, ros::Time(0));

			count++;
		}

		flag =true;
		ros::spinOnce();
		loop_rate.sleep();
	}

	return (0);
}



