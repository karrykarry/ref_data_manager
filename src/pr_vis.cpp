
#include"pr_vis.hpp"

PR_vis::PR_vis(ros::NodeHandle n,ros::NodeHandle private_nh_):
	output_txtfile("/home/amsl/Pictures/ros_catkin_ws/scan_context/save_pr/list.txt")
{
	
	// pr_num_vis_pub = n.advertise<visualization_msgs::Marker>("/pr/num/vis", 10);
	pr_num_vis_pub = n.advertise<visualization_msgs::MarkerArray>("/pr/num/vis", 10);
	
	odom_sub = n.subscribe<nav_msgs::Odometry>("/odometer", 1, &PR_vis::odomCallback, this);
	flag_sub = n.subscribe<std_msgs::Empty>("/pr/saveflag", 1, &PR_vis::flagCallback, this);

	writing_file.open(output_txtfile, std::ios::out);
}

PR_vis::~PR_vis(){

    cout << "\033[1;31m save pr_save_pose \033[0m" << endl;

	for(auto pr_pose : pr_poses){
		writing_file << pr_pose.x <<"," << pr_pose.y <<"," << pr_pose.z <<endl;
		cout << pr_pose.x <<"," << pr_pose.y <<"," << pr_pose.z <<endl;
	}
}

visualization_msgs::Marker
PR_vis::text_vis(const int num){

	visualization_msgs::Marker m;
	// m.header.stamp = ros::Time::now(); 
	m.header.stamp = ros::Time(0); 
	m.header.frame_id = "/map";
	m.ns = "histogram";
	m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	m.action = visualization_msgs::Marker::ADD;
	m.lifetime = ros::Duration(0);
	// 形
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
	m.text =  to_string(num).substr(0,4);

	return m;
}




void 
PR_vis::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
	now_pose =msg->pose.pose.position;
}


void 
PR_vis::flagCallback(const std_msgs::EmptyConstPtr &msg)
{
	static int num = 0;
	pr_poses.push_back((now_pose));
	
	m_array.markers.push_back(text_vis(num));
	pr_num_vis_pub.publish(m_array);
	num++;
}
