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
		ros::Subscriber pose_sub;
		ros::Subscriber flag_sub;

		std::string file_dir, file_dir2;
		std::string pc_file_name;
	
		int file_count;
		ros::Time now_time;

	public:
		Test_pc_run(ros::NodeHandle n,ros::NodeHandle priv_nh);

		void pc_publisher(ros::Publisher pub, const int num, const ros::Time now_time);
		void poseCallback(const geometry_msgs::PoseConstPtr &msg);	
		void flagCallback(const std_msgs::BoolConstPtr &msg);

};


Test_pc_run::Test_pc_run(ros::NodeHandle n, ros::NodeHandle private_nh_):
	file_count(0), now_time(0)
{
	pc_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);
	test_pc_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points/test", 10);
	
	pose_sub = n.subscribe<geometry_msgs::Pose>("/map2context_result", 1, &Test_pc_run::poseCallback, this);
	flag_sub = n.subscribe<std_msgs::Bool>("/next_pcd", 1, &Test_pc_run::flagCallback, this);

	private_nh_.param("TEST/FILE_DIR", file_dir, {"/home/amsl/Pictures/ros_catkin_ws/test/lidar_only"});
	private_nh_.param("TEST/FILE_DIR2", file_dir2, {"/0925"});

	
	pc_file_name = file_dir;
	pc_file_name += file_dir2;

}



void
Test_pc_run::pc_publisher(ros::Publisher pub,const int num, const ros::Time now_time_){
	
	std::ostringstream oss;
	oss << std::setfill( '0' ) << std::setw( 4 ) << num;

	std::string pcd_name; 
	pcd_name = pc_file_name + "/" + oss.str() + ".pcd";

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_IN (new pcl::PointCloud<pcl::PointXYZI>);
	if( pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_name,*cloud_IN) == -1 ){
		std::cerr << "pcd load error !!"<<std::endl;;
		exit(1);
	}
	
	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(*cloud_IN, pc);

	pc.header.frame_id  = "/context_estimate";	
	pc.header.stamp  = now_time_;
	pub.publish(pc);
}

void 
Test_pc_run::poseCallback(const geometry_msgs::PoseConstPtr &msg){

	std::cout << "File_Number: " << file_count << std::endl;
	std::cout<<now_time<<std::endl;

	pc_publisher(test_pc_pub, file_count ,now_time);
	pc_publisher(pc_pub, file_count, now_time);
	
	file_count++;
	now_time += ros::Duration(0.5);
}

void 
Test_pc_run::flagCallback(const std_msgs::BoolConstPtr &msg){
	std::cout << "File_Number: " << file_count << std::endl;
	std::cout<<now_time<<std::endl;
	
	pc_publisher(test_pc_pub, file_count ,now_time);
	pc_publisher(pc_pub, file_count, now_time);

	file_count++;
	now_time += ros::Duration(0.5);
}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "test_pc_save");
	ros::NodeHandle n;
	ros::NodeHandle private_nh_("~");
    ROS_INFO("\033[1;32m---->\033[0mtest_pc_save Started.");
	
	Test_pc_run test_pc_run(n, private_nh_);

	ros::spin();

	return (0);
}


