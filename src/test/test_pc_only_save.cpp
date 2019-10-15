/*
 * 評価用にpose とその時のvelodyneのデータをpcdにして保存しておくやつ
 *
 *
 *
 */

#include<stdio.h>
#include<iostream>
#include<ros/ros.h>
#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/PointCloud2.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_broadcaster.h>

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>


class Test_pc_save{
	private:
		ros::Subscriber pc_sub;
		ros::Subscriber odo_sub;
		
		std::string file_dir, file_dir2, pc_file_dir, odom_file_dir, odom_list;
		std::string pc_file_name, pc_time_name;
		std::ofstream writing_file;
		int SKIP_TIME;
		bool odom_flag;

		std::vector<double> times;

		pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;
		
	public:
		Test_pc_save(ros::NodeHandle n, ros::NodeHandle private_nh_);
		
		void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& input);
};


Test_pc_save::Test_pc_save(ros::NodeHandle n, ros::NodeHandle private_nh_):
	input_cloud(new pcl::PointCloud<pcl::PointXYZI>)
{	
	pc_sub = n.subscribe("/velodyne_points", 1, &Test_pc_save::lidarCallback, this);
	
	private_nh_.param("TEST/FILE_DIR", file_dir, {"/home/amsl/Pictures/ros_catkin_ws/test/lidar_only"});
	private_nh_.param("TEST/FILE_DIR2", file_dir2, {"/0925"});
	private_nh_.param("SKIP_TIME", SKIP_TIME, {10});
	
	pc_file_name = file_dir;
	pc_file_name += file_dir2;
	pc_time_name = pc_file_name;


	pc_time_name += "/time.txt";

	writing_file.open(pc_time_name, std::ios::out);
}



void 
Test_pc_save::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	static int count = 0;
	count++;
	if(count%SKIP_TIME == 0){
		pcl::fromROSMsg (*input, *input_cloud);

		static int pcd_num = 0;

		std::ostringstream oss;
		oss << std::setfill( '0' ) << std::setw( 4 ) << pcd_num++;

		std::string pcd_name; 

		pcd_name = pc_file_name + "/" + oss.str() + ".pcd";

		std::cout << pcd_name <<std::endl;

		pcl::io::savePCDFile(pcd_name, *input_cloud);

		// times.push_back((double)input->header.stamp.nsec*1.0e-9 + input->header.stamp.sec);
		writing_file <<  input->header.stamp << std::endl;
		
		count = 0;
	}
	

}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "test_pc_save");
	ros::NodeHandle n;
	ros::NodeHandle private_nh_("~");
    ROS_INFO("\033[1;32m---->\033[0mtest_pc_save Started.");
	
	Test_pc_save test_pc_save(n, private_nh_);

	ros::spin();

	return (0);
}


