/*
 *itstでnodeの判定したら、すべてをpubする管理ソース
 （そこでのscan_contextをrefにするソース）
 *
 */
#include"itst2scan.hpp"

Itst2data_pub::Itst2data_pub(ros::NodeHandle n,ros::NodeHandle private_nh_)
{

	estimate_num_pub = n.advertise<visualization_msgs::Marker>("/pr/estimate_num", 10);
	pr_num_vis_pub = n.advertise<visualization_msgs::MarkerArray>("/pr/num/vis", 10, true);
	
	best_num_sub = n.subscribe<std_msgs::Int32>("/score/best", 1, &Itst2data_pub::itstcallback, this);
	
	
	private_nh_.param("PR_INFO/FILE_DIR", file_dir, {"/home/amsl/Pictures/ros_catkin_ws/ref_data_manager"});
	private_nh_.param("PR_INFO/FILE_DIR2", file_dir2, {"/sample"});
	private_nh_.param("PR_INFO/FILE_DIR3", file_dir3, {"/pose"});
	private_nh_.param("PR_INFO/FILE_NAME", file_name, {"/list"});
	private_nh_.param("PR_INFO/FILE_EXT", file_ext, {".txt"});

	pr_list_pub();

}

Itst2data_pub::~Itst2data_pub(){
}


visualization_msgs::Marker
Itst2data_pub::make_vis_marker(const double now_x,const double now_y,const double now_z, const int num, const int id_num){

	visualization_msgs::Marker m;
	m.header.stamp = ros::Time(0); 
	m.header.frame_id = "/map";
	m.ns = "histogram";
	m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	m.action = visualization_msgs::Marker::ADD;
	m.lifetime = ros::Duration(0);
	// 形
	m.color.r = 1.0;
	m.color.g = 0.0;
	m.color.b = 0.0;
	m.color.a = 1.0; 

	m.pose.orientation.x = 0.0;
	m.pose.orientation.y = 0.0;
	m.pose.orientation.z = 0.0;
	m.pose.orientation.w = 1.0;
	m.pose.position.x = now_x;
	m.pose.position.y = now_y;	
	m.pose.position.z = now_z;

	m.scale.z = 5.0; 

	m.id = id_num;
	m.text =  std::to_string(num).substr(0,4);

	return m;
}



std::vector<std::string> 
Itst2data_pub::split(const std::string &str, char sep)
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
Itst2data_pub::pr_list_pub(){
	std::string PR_list_filename = file_dir;		//poseが入っているpath及びファイル
	PR_list_filename.append(file_dir2);
	PR_list_filename.append(file_dir3);
	PR_list_filename.append(file_name);
	PR_list_filename.append(file_ext);
	

	std::ifstream reading_file;
	std::string reading_line_buffer;

	reading_file.open(PR_list_filename, std::ios::in);
	
	if (reading_file.fail()) {
		std::cout<<"\033[1;32m File could not be opened \033[0m"<<std::endl;
		exit(1);
	}
	
	std::vector<std::string> v;
		
	visualization_msgs::MarkerArray m_array;
	std::cout<<"------read "<<PR_list_filename<< "--------"<<std::endl;
	
	static int num = 0;
	while (!reading_file.eof())
	{
		// read by line
		std::getline(reading_file, reading_line_buffer);
		if(!reading_line_buffer.empty()){
			v = split(reading_line_buffer,',');
			std::cout<<atof(v[0].c_str())<<","<<atof(v[1].c_str())<<","<<atof(v[2].c_str())<<std::endl;
			m_array.markers.push_back(make_vis_marker( atof(v[0].c_str()), atof(v[1].c_str()), atof(v[2].c_str()) , num, num));
			num++;
        
			{
				geometry_msgs::Point pr_pose;
				pr_pose.x = atof(v[0].c_str());
				pr_pose.y = atof(v[1].c_str());
				pr_pose.z = atof(v[2].c_str());
        
				pr_poses.push_back(pr_pose);
			}
		}
	}
	pr_num_vis_pub.publish(m_array);
	std::cout<<"------finish "<<PR_list_filename<< "--------"<<std::endl;
}


void
Itst2data_pub::num_vis(const int num){
	
	visualization_msgs::Marker est_num;	//estimate_num
	est_num =  make_vis_marker(pr_poses[num].x, pr_poses[num].y, pr_poses[num].z, num, 0);
	
	//color change
	est_num.color.r = 1.0;
	est_num.color.g = 1.0;
	est_num.color.b = 1.0;
	est_num.color.a = 1.0; 
	
	estimate_num_pub.publish(est_num);
}



void
Itst2data_pub::itstcallback(const std_msgs::Int32ConstPtr &msg){
	num_vis(msg->data);	

}
