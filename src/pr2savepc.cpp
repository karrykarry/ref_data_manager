//
// 引数にくっつけたいpcdを取る
//

#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


class PC_save{
	private:
		ros::Subscriber pc_sub;
		std::string file_dir, file_dir2, file_dir3;
		std::string file_name;

		pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;
		
	public:
		PC_save(ros::NodeHandle n, ros::NodeHandle private_nh_);
		
		void lidar_Callback(const sensor_msgs::PointCloud2ConstPtr& input);
};


PC_save::PC_save(ros::NodeHandle n, ros::NodeHandle private_nh_):
	input_cloud(new pcl::PointCloud<pcl::PointXYZI>)
{	
	pc_sub = n.subscribe("/velodyne_points", 1, &PC_save::lidar_Callback, this);
	
	private_nh_.param("PCD_INFO/FILE_DIR", file_dir, {"/home/amsl/Pictures/ros_catkin_ws/ref_data_manager"});
	private_nh_.param("PCD_INFO/FILE_DIR2", file_dir2, {"/sample"});
	private_nh_.param("PCD_INFO/FILE_DIR3", file_dir3, {"/pcd"});
	file_name = file_dir;
	file_name += file_dir2;
	file_name += file_dir3;
}



void 
PC_save::lidar_Callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::fromROSMsg (*input, *input_cloud);
	
	static int pcd_num = 0;
	
	std::ostringstream oss;
	oss << std::setfill( '0' ) << std::setw( 3 ) << pcd_num++;
	
	std::string pcd_name; 
	
	pcd_name = file_name + "/" + oss.str() + ".pcd";
	
	std::cout << pcd_name <<std::endl;
	
	pcl::io::savePCDFile(pcd_name, *input_cloud);

}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "pc2savepc");
	ros::NodeHandle n;
	ros::NodeHandle private_nh_("~");
    ROS_INFO("\033[1;32m---->\033[0m pc2savepc Started.");
	
	PC_save pc_save(n, private_nh_);

	ros::spin();

	return (0);
}
