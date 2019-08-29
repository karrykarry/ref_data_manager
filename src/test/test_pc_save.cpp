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
		std::string pc_file_name, odom_file_name;
		std::ofstream writing_file;
		int SKIP_TIME;
		bool odom_flag;

		std::vector <geometry_msgs::Pose> buffer_odom_poses;
		geometry_msgs::Pose buffer_odom;

		pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;
		
	public:
		Test_pc_save(ros::NodeHandle n, ros::NodeHandle private_nh_);
		~Test_pc_save();
		
		void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& input);
		void odomCallback(const nav_msgs::OdometryConstPtr& input);
};


Test_pc_save::Test_pc_save(ros::NodeHandle n, ros::NodeHandle private_nh_):
	odom_flag(false),
	input_cloud(new pcl::PointCloud<pcl::PointXYZI>)
{	
	pc_sub = n.subscribe("/velodyne_points", 1, &Test_pc_save::lidarCallback, this);
	odo_sub = n.subscribe("/odometer", 1, &Test_pc_save::odomCallback, this);
	
	private_nh_.param("TEST/FILE_DIR", file_dir, {"/home/amsl/Pictures/ros_catkin_ws/ref_data_manager"});
	private_nh_.param("TEST/FILE_DIR2", file_dir2, {"/sample"});
	private_nh_.param("TEST/PC_FILE_DIR", pc_file_dir, {"/test_pcd"});
	private_nh_.param("TEST/ODOM_FILE_DIR", odom_file_dir, {"/test_pose"});
	private_nh_.param("TEST/ODOM_LIST", odom_list, {"/list.txt"});
	private_nh_.param("SKIP_TIME", SKIP_TIME, {4});
	pc_file_name = file_dir;
	pc_file_name += file_dir2;
	pc_file_name += pc_file_dir;
	
	odom_file_name = file_dir;
	odom_file_name += file_dir2;
	odom_file_name += odom_file_dir;
	odom_file_name += odom_list;

	writing_file.open(odom_file_name, std::ios::out);
}

Test_pc_save::~Test_pc_save(){

	std::cout << "\033[1;31m save --> " << odom_file_name << "\033[0m" << std::endl;

	// for(auto pr_pose : buffer_odom_poses){
	// 	writing_file << pr_pose.position.x <<"," << pr_pose.position.y <<"," << pr_pose.position.z
	// 		<<","<< pr_pose.orientation.x <<"," << pr_pose.orientation.y <<"," << pr_pose.orientation.z
	// 		<<","<< pr_pose.orientation.w << std::endl;
	// }


	for(auto pr_pose : buffer_odom_poses){
		double roll, pitch, yaw;	
		tf::Quaternion q;
		quaternionMsgToTF(pr_pose.orientation,q);
		tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
			
		writing_file << pr_pose.position.x <<"," << pr_pose.position.y <<"," << pr_pose.position.z
			<<","<< roll <<"," << pitch <<"," << yaw << std::endl;
	}
}


void 
Test_pc_save::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	if(odom_flag){
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

			buffer_odom_poses.push_back(buffer_odom);

			count = 0;
		}
	}

}

void 
Test_pc_save::odomCallback(const nav_msgs::OdometryConstPtr& input)
{
	buffer_odom = input->pose.pose;
	odom_flag = true;
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

