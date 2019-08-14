/*
 * PRの情報を受けてpose情報を保存していくソース
 *
 */
#include"pr2pose.hpp"

PR2Pose::PR2Pose(ros::NodeHandle n,ros::NodeHandle private_nh_)
{	
	// pr_num_vis_pub = n.advertise<visualization_msgs::Marker>("/pr/num/vis", 10);
	pr_num_vis_pub = n.advertise<visualization_msgs::MarkerArray>("/pr/num/vis", 10);
	
	odom_sub = n.subscribe<nav_msgs::Odometry>("/odometer", 1, &PR2Pose::odomCallback, this);
	flag_sub = n.subscribe<std_msgs::Empty>("/pr/saveflag", 1, &PR2Pose::flagCallback, this);

	private_nh_.param("PR_INFO/FILE_DIR", file_dir, {"/home/amsl/Pictures/ros_catkin_ws/ref_data_manager"});
	private_nh_.param("PR_INFO/FILE_DIR2", file_dir2, {"/sample"});
	private_nh_.param("PR_INFO/FILE_DIR3", file_dir3, {"/pose"});
	private_nh_.param("PR_INFO/FILE_NAME", file_name, {"/list"});
	private_nh_.param("PR_INFO/FILE_EXT", file_ext, {".txt"});

	output_txtfile = file_dir;
	output_txtfile.append(file_dir2);
	output_txtfile.append(file_dir3);
	output_txtfile.append(file_name);
	output_txtfile.append(file_ext);
	
	std::cout << "\033[1;31m output --> " << output_txtfile << "\033[0m" << std::endl;

	writing_file.open(output_txtfile, std::ios::out);
}

PR2Pose::~PR2Pose(){

	std::cout << "\033[1;31m save --> " << output_txtfile << "\033[0m" << std::endl;

	for(auto pr_pose : pr_poses){
		writing_file << pr_pose.x <<"," << pr_pose.y <<"," << pr_pose.z << std::endl;
		std::cout << pr_pose.x <<"," << pr_pose.y <<"," << pr_pose.z << std::endl;
	}
}

visualization_msgs::Marker
PR2Pose::text_vis(const int num){

	visualization_msgs::Marker m;
	// m.header.stamp = ros::Time::now(); 
	m.header.stamp = ros::Time(0); 
	m.header.frame_id = "/map";
	m.ns = "histogram";
	m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	m.action = visualization_msgs::Marker::ADD;
	m.lifetime = ros::Duration(0);
	// $B7A(B
	m.color.r = 1.0;
	m.color.g = 1.0;
	m.color.b = 1.0;
	m.color.a = 1.0; 

	m.pose.orientation.x = 0.0;
	m.pose.orientation.y = 0.0;
	m.pose.orientation.z = 0.0;
	m.pose.orientation.w = 1.0;
	m.pose.position.x = now_pose.x;
	m.pose.position.y = now_pose.y;	
	m.pose.position.z = 0.0;

	m.scale.z = 5.0; 

	m.id = num;
	m.text =  std::to_string(num).substr(0,4);

	return m;
}



//mapでの位置を知るためにsub
void 
PR2Pose::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
	now_pose =msg->pose.pose.position;
}


//PRすることを知るためにsub
void 
PR2Pose::flagCallback(const std_msgs::EmptyConstPtr &msg)
{
	static int num = 0;
	pr_poses.push_back((now_pose));
	
	m_array.markers.push_back(text_vis(num));
	pr_num_vis_pub.publish(m_array);
	num++;
}
