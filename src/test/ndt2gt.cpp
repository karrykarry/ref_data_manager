/*
 *
 * NDTのposeとpcdをicpにかけてしっかりしたのにする
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>


class Test_pc_run{

	private:
		ros::Publisher before_pub;
		ros::Publisher after_pub;
		ros::Publisher map_pub;
		pcl::PointCloud<pcl::PointXYZI>::Ptr low_map_cloud;

		std::vector<geometry_msgs::Pose> pr_poses;
		
		std::vector<geometry_msgs::Point> after_poses;

		std::string tgt_file_name, src_file_name, src_pose_file_name;

		std::ofstream writing_file;

		Eigen::Matrix4f map_icp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, geometry_msgs::Pose odo);
		std::vector<std::string> split(const std::string &str, char sep);
		void pr_list_pub();
		void calc_rpy(Eigen::Matrix4f ans, double &yaw);

	public:
		Test_pc_run(ros::NodeHandle n,ros::NodeHandle priv_nh);
		~Test_pc_run();
		void map_read(std::string filename);

		void pc_publisher(const int num, const ros::Time now_time);
};


Test_pc_run::Test_pc_run(ros::NodeHandle n, ros::NodeHandle private_nh_):
	low_map_cloud(new pcl::PointCloud<pcl::PointXYZI>)
{
	before_pub = n.advertise<sensor_msgs::PointCloud2>("/transform_pcd_before", 1);
	after_pub = n.advertise<sensor_msgs::PointCloud2>("/transform_pcd", 1);
	
	map_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_pcd", 1, true);

	private_nh_.param("TGT_FILE_NAME", tgt_file_name, {"/home/amsl/Pictures/ros_catkin_ws/ref_data_manager/sample/perfect_tkb_2018-09-15-14-08_v3.pcd"});
	
	private_nh_.param("SRC_FILE_NAME", src_file_name, {"/home/amsl/Pictures/ros_catkin_ws/ref_data_manager/sample/test_pcd"});
	private_nh_.param("SRC_POSE_FILE_NAME", src_pose_file_name, {"/home/amsl/Pictures/ros_catkin_ws/ref_data_manager/sample/test_pose"});

	pr_list_pub();
	map_read(tgt_file_name);


	writing_file.open(src_pose_file_name + "/icp_list.txt" , std::ios::out);
	std::cout<<"Let's go"<<std::endl;
}

Test_pc_run::~Test_pc_run(){

	std::cout << "\033[1;31m save --> " << src_pose_file_name + "/icp_list.txt" << "\033[0m" << std::endl;

	for(auto after_pose: after_poses){
		writing_file << after_pose.x <<"," << after_pose.y <<"," << 0.0 << "," 
			<< 0.0 <<"," << 0.0 <<"," << after_pose.z << std::endl;
	}

}


std::vector<std::string> 
Test_pc_run::split(const std::string &str, char sep)
{
	std::vector<std::string> v;
	std::stringstream ss(str);
	std::string buffer;
	while( getline(ss, buffer, sep) ) {
		v.push_back(buffer);
	}
	return v;
}


void
Test_pc_run::pr_list_pub(){
	std::string PR_list_filename = src_pose_file_name + "/list.txt"; 
	

	std::ifstream reading_file;
	std::string reading_line_buffer;

	reading_file.open(PR_list_filename, std::ios::in);

	if(reading_file.fail()) {
		this->~Test_pc_run();
		std::cerr<<"\033[1;32m File could not be opened "<< PR_list_filename << "\033[0m"<<std::endl;
		exit(1);
	}

	std::vector<std::string> v;	
	std::cout<<"------read "<<PR_list_filename<< "--------"<<std::endl;
	
	while (!reading_file.eof())
	{
		// read by line
		std::getline(reading_file, reading_line_buffer);
		if(!reading_line_buffer.empty()){
			v = split(reading_line_buffer,',');
			std::cout<<atof(v[0].c_str())<<","<<atof(v[1].c_str())<<","<<atof(v[2].c_str())<<","<<atof(v[5].c_str())<<std::endl;
		
			geometry_msgs::Pose pr_pose;

			pr_pose.position.x = atof(v[0].c_str());
			pr_pose.position.y = atof(v[1].c_str());
			pr_pose.position.z = atof(v[2].c_str());
			
			//roll:3 pitch:4 yaw:5
			pr_pose.orientation.z = atof(v[5].c_str());	//yaw
			
			pr_poses.push_back(pr_pose);

		}
	}
	std::cout<<"\033[1;31minput --> pr_pose \033[0m"<<std::endl;
}




void
Test_pc_run::map_read(std::string filename){


    if (pcl::io::loadPCDFile<pcl::PointXYZI> (filename, *low_map_cloud) == -1)  
		PCL_ERROR ("事前地図ないよ \n");
	else 
		std::cout<<"\x1b[32m"<<"読み込んだファイル："<<filename<<"\x1b[m\r"<<std::endl;
	
	sensor_msgs::PointCloud2 vis_map;
	pcl::toROSMsg(*low_map_cloud , vis_map);           

	vis_map.header.stamp = ros::Time(0); //laserのframe_id
	vis_map.header.frame_id = "/map";

	map_pub.publish(vis_map);
	sleep(1.0);
	std::cout<<"\x1b[32m"<<"map read finish"<<filename<<"\x1b[m\r"<<std::endl;
}




Eigen::Matrix4f 
Test_pc_run::map_icp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tgt, 
					 pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src, 
					 pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, 
					 geometry_msgs::Pose odo)
{
	pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
	icp.setMaxCorrespondenceDistance(1.0);
	icp.setMaximumIterations(100);
	icp.setTransformationEpsilon(1e-8);
	icp.setEuclideanFitnessEpsilon(1e-8);

	/*------ Voxel Grid ------*/
	pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_src (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_tgt (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::VoxelGrid<pcl::PointXYZI> vg;
	vg.setLeafSize(0.3,0.3,0.3);
	vg.setInputCloud(cloud_src);
	vg.filter(*filtered_cloud_src);
	vg.setInputCloud(cloud_tgt);
	vg.filter(*filtered_cloud_tgt);


	Eigen::AngleAxisf init_rotation (odo.orientation.z , Eigen::Vector3f::UnitZ ());
	Eigen::Translation3f init_translation (odo.position.x, odo.position.y, odo.position.z);

	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();


	icp.setInputTarget(filtered_cloud_tgt);
	icp.setInputSource(filtered_cloud_src);
    icp.align (*cloud, init_guess);
	
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
	pcl::transformPointCloud (*cloud_src, *transformed_cloud, init_guess);
	
	sensor_msgs::PointCloud2 pc_;
	pcl::toROSMsg(*transformed_cloud, pc_);

	pc_.header.frame_id  = "/map";	
	pc_.header.stamp  = ros::Time(0);
	before_pub.publish(pc_);


	return icp.getFinalTransformation();
}


void 
Test_pc_run::calc_rpy(Eigen::Matrix4f ans, double &yaw){
	double roll, pitch;
	tf::Matrix3x3 mat_l;
	mat_l.setValue(static_cast<double>(ans(0, 0)), static_cast<double>(ans(0, 1)), static_cast<double>(ans(0, 2)),
			static_cast<double>(ans(1, 0)), static_cast<double>(ans(1, 1)), static_cast<double>(ans(1, 2)),
			static_cast<double>(ans(2, 0)), static_cast<double>(ans(2, 1)), static_cast<double>(ans(2, 2)));

	mat_l.getRPY(roll, pitch, yaw, 1);
}


void
Test_pc_run::pc_publisher(const int num, const ros::Time now_time_){
	
	std::ostringstream oss;
	oss << std::setfill( '0' ) << std::setw( 4 ) << num;

	std::string pcd_name; 
	pcd_name = src_file_name + "/" + oss.str() + ".pcd";

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_IN (new pcl::PointCloud<pcl::PointXYZI>);
	if( pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_name,*cloud_IN) == -1 ){
		std::cerr << "pcd load error !!"<<std::endl;;
		this->~Test_pc_run();
		exit(1);
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_OUT (new pcl::PointCloud<pcl::PointXYZI>);
	Eigen::Matrix4f answer = map_icp(low_map_cloud, cloud_IN,  cloud_OUT, pr_poses[num]);
	
	double ans_yaw;

	calc_rpy(answer, ans_yaw);

	std::cout<<"-------------------------------------"<<std::endl;
	std::cout << pr_poses[num].position.x << "," << pr_poses[num].position.y << "," << 
		pr_poses[num].orientation.z << std::endl;
	std::cout<<std::endl;	
	std::cout << answer(0, 3) << "," << answer(1, 3) << "," << ans_yaw <<std::endl;
	std::cout<<"-------------------------------------"<<std::endl;

	geometry_msgs::Point aft_;
	aft_.x = answer(0, 3);
	aft_.y = answer(1, 3);
	aft_.z = ans_yaw;

	after_poses.push_back(aft_);

	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(*cloud_OUT, pc);

	pc.header.frame_id  = "/map";	
	pc.header.stamp  = ros::Time(0);
	after_pub.publish(pc);
}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "pub_pcd");
	ros::NodeHandle n;
	ros::NodeHandle private_nh_("~");
    ROS_INFO("\033[1;32m---->\033[0mpub_pcd Started.");
	
	Test_pc_run test_pc_run(n, private_nh_);
	
	int count=0;
    ros::Rate loop_rate(10);
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




