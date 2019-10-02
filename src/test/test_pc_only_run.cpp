/*
 *
 * 検証用のソースコード
 * 検証するのが 
 * use_sim_time trueを使っているからその使用になっている
 *
 * ~/data/bagfiles/infant$ rosbag play 2018-09-09-16-33-11.bag --clock -s 15 
 * でvelodyne_pointsを使わない用にする
 *
 */

#include"test_pc_run.hpp"

Test_pc_run::Test_pc_run(ros::NodeHandle n, ros::NodeHandle private_nh_):
	file_count(0),
	pose_true_cnt(0), yaw_true_cnt(0), all_true_cnt(0)
{
	pc_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);
	test_pc_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points/test", 10);
	
	pose_sub = n.subscribe<geometry_msgs::Pose>("/map2context_result", 1, &Test_pc_run::poseCallback, this);

	private_nh_.param("TEST/FILE_DIR", file_dir, {"/home/amsl/Pictures/ros_catkin_ws/ref_data_manager"});
	private_nh_.param("TEST/FILE_DIR2", file_dir2, {"/sample"});
	private_nh_.param("TEST/PC_FILE_DIR", pc_file_dir, {"/test_pcd"});
	private_nh_.param("TEST/ODOM_FILE_DIR", odom_file_dir, {"/test_pose"});
	private_nh_.param("TEST/ODOM_LIST", odom_list, {"/list.txt"});

	pr_list_pub();
	
	pc_file_name = file_dir;
	pc_file_name += file_dir2;
	pc_file_name += pc_file_dir;



	writing_file.open("/home/amsl/m2_result/context.csv", std::ios::out);

}



void
Test_pc_run::pc_publisher(ros::Publisher pub,const int num){
	std::ostringstream oss;
	oss << std::setfill( '0' ) << std::setw( 4 ) << num;

	std::string pcd_name; 
	pcd_name = pc_file_name + "/" + oss.str() + ".pcd";

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_IN (new pcl::PointCloud<pcl::PointXYZI>);
	if( pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_name,*cloud_IN) == -1 ){
		std::cerr << "pcd load error !!"<<std::endl;;
		this->~Test_pc_run();
		exit(1);
	}
	
	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(*cloud_IN, pc);

	pc.header.frame_id  = "/context_estimate";	
	pc.header.stamp  = ros::Time(0);
	pub.publish(pc);

}

void 
Test_pc_run::poseCallback(const geometry_msgs::PoseConstPtr &msg){

	static bool is_start = false;
	if(is_start){
		std::cout << "File_Number: " << file_count << std::endl;

		bool pose_flag = diff_pose(*msg);

		double est_yaw, odo_yaw;
		est_yaw = deg2rad(msg->orientation.z);
		odo_yaw = deg2rad(pr_poses[file_count].z);

		std::cout<<"estimate(yaw): "<< est_yaw <<"[deg]"<<"  odometry(yaw): "<< odo_yaw <<"[deg]"<<std::endl;
		bool yaw_flag = diff_yaw(est_yaw, odo_yaw);

		if(pose_flag){
			std::cout<<"pose : OK, "<<std::flush;
			pose_true_cnt++;
		}
		else std::cout<<"pose : NG, "<<std::flush;

		if(yaw_flag){
			std::cout<<"yaw : OK"<<std::endl;
			yaw_true_cnt++;
		}

		else std::cout<<"yaw : NG"<<std::endl;

		if(pose_flag && yaw_flag) all_true_cnt++;
	
		pc_publisher(test_pc_pub, file_count);

		file_count++;
		std::cout<<std::endl;
	}
	pc_publisher(pc_pub, file_count);
	is_start = true;
}


