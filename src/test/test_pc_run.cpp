/*
 *
 * 検証用のソースコード
 * 検証するのが 
 * use_sim_time trueを使っているからその使用になっている
 *
 * ~/data/bagfiles/infant$ rosbag play 2018-09-09-16-33-11.bag --clock -s 15 
 * でvelodyne_pointsを使わない用にする
 *
 * pf_scoreの画像のdataset用を集めるソースコードも兼ねている
 * IS_DATASETの場所が変更点
 */

#include"test_pc_run.hpp"

Test_pc_run::Test_pc_run(ros::NodeHandle n, ros::NodeHandle private_nh_):
	file_count(-1),
	pose_true_cnt(0), yaw_true_cnt(0), all_true_cnt(0),
	nx_flag(true)
{
	pc_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);
	score_pub = n.advertise<std_msgs::Int32>("/score/process", 10);
	test_pc_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points/test", 10);
	recover_pub = n.advertise<std_msgs::Empty>("/process", 10);
	
	pose_sub = n.subscribe<geometry_msgs::Pose>("/map2context_result", 1, &Test_pc_run::poseCallback, this);
	flag_sub = n.subscribe<std_msgs::Bool>("/next_pcd", 1, &Test_pc_run::flagCallback, this);

	private_nh_.param("TEST/FILE_DIR", file_dir, {"/home/amsl/Pictures/ros_catkin_ws/ref_data_manager"});
	private_nh_.param("TEST/FILE_DIR2", file_dir2, {"/sample"});
	private_nh_.param("TEST/PC_FILE_DIR", pc_file_dir, {"/test_pcd"});
	private_nh_.param("TEST/ODOM_FILE_DIR", odom_file_dir, {"/test_pose"});
	private_nh_.param("TEST/ODOM_LIST", odom_list, {"/list.txt"});
	
	private_nh_.param("IS_DATASET", IS_DATASET, {false});

	pr_list_pub();
	
	pc_file_name = file_dir;
	pc_file_name += file_dir2;
	pc_file_name += pc_file_dir;


	writing_file.open("/home/amsl/m2_result/context.csv", std::ios::out);
 	ROS_INFO_STREAM("\033[1;32mAre we in dataset mode? :" << IS_DATASET<<"\033[0m");

}

Test_pc_run::~Test_pc_run(){
	std::cout << "\033[1;31m"<<
		"Result pose_ok:" << pose_true_cnt << 
		", yaw_ok:" << yaw_true_cnt << 
		", all_ok:" << all_true_cnt << 
		", file_sum:" << file_count <<"\033[0m" << std::endl; 

		
	std::cout<<"----save now----"<<std::endl;
	for(auto xx : diff_x){
		writing_file << xx <<"," <<std::flush;
	}
	writing_file <<std::endl;
	for(auto yy : diff_y){
		writing_file << yy <<"," <<std::flush;
	}
	writing_file <<std::endl;
	for(auto theta : diff_theta){
		writing_file << theta <<"," <<std::flush;
	}

	std::cout<<std::endl;
	std::cout<<std::endl;
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
	std::string PR_list_filename = file_dir;		//poseが入っているpath及びファイル
	PR_list_filename.append(file_dir2);
	PR_list_filename.append(odom_file_dir);
	PR_list_filename.append(odom_list);
	

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
			// std::cout<<atof(v[0].c_str())<<","<<atof(v[1].c_str())<<","<<atof(v[2].c_str())<<std::endl;
			std::cout<<atof(v[0].c_str())<<","<<atof(v[1].c_str())<<","<<atof(v[5].c_str())<<std::endl;

			geometry_msgs::Point pr_pose;
			pr_pose.x = atof(v[0].c_str());
			pr_pose.y = atof(v[1].c_str());
			
			// z:2 roll:3 pitch:4 yaw:5
			pr_pose.z = atof(v[5].c_str());	//yaw

			pr_poses.push_back(pr_pose);
		}
	}
	std::cout<<"\033[1;31minput --> pr_pose \033[0m"<<std::endl;
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


bool
Test_pc_run::diff_pose(const geometry_msgs::Pose est_pose){
	bool result_flag = false;

	std::cout << 
		"estimate(x, y): " << est_pose.position.x << "," << est_pose.position.y << 
		
		"  odometry(x, y): " << pr_poses[file_count].x << "," << pr_poses[file_count].y <<std::endl;
	

	if( (fabs(est_pose.position.x - pr_poses[file_count].x) < 0.5) 
			&& (fabs(est_pose.position.y - pr_poses[file_count].y) < 0.5) ){
		result_flag = true;
		score.data = 0;
	}

	else{
		std::cout<<"\033[1;31m"<<std::flush;
		
		if( (fabs(est_pose.position.x - pr_poses[file_count].x) < 5) 
				&& (fabs(est_pose.position.y - pr_poses[file_count].y) < 5) ){
			score.data = 1;
		}
		else score.data = 2;
	}
		
	std::cout<< "Diffrence(x, y):" << est_pose.position.x - pr_poses[file_count].x << ","
		<< est_pose.position.y - pr_poses[file_count].y << "\033[0m" << std::endl;

	if(!IS_DATASET){
		diff_x.push_back(fabs(est_pose.position.x - pr_poses[file_count].x));
		diff_y.push_back(fabs(est_pose.position.y - pr_poses[file_count].y));
	}

	return result_flag;
}



bool
Test_pc_run::diff_yaw(const double yaw1,const double yaw2){
	bool result_flag = false;
	
	double x_ = cos(yaw2) * cos(yaw1) + sin(yaw2) * sin(yaw1);
	double y_ = (-1) * sin(yaw2) * cos(yaw1) + cos(yaw2) * sin(yaw1);

	if(fabs(atan2(y_, x_) * 180.0 / M_PI) <= 6.0) result_flag = true;
	
	else std::cout<<"\033[1;31m"<<std::flush;
	
	std::cout<<"diff_theta: "<< fabs(atan2(y_, x_) * 180.0 / M_PI) << "[deg]\033[0m" <<std::endl;

	if(!IS_DATASET) diff_theta.push_back(fabs(atan2(y_, x_) * 180.0 / M_PI));
	
	return result_flag;
}


void 
Test_pc_run::poseCallback(const geometry_msgs::PoseConstPtr &msg){

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



	// 	file_count++;
	// 	std::cout<<std::endl;


	// if(nx_flag) pc_publisher(pc_pub, file_count);
	if(!IS_DATASET){
		pc_publisher(pc_pub, file_count);
		file_count++;
		std::cout<<std::endl;
	}
	
	else{
		score_pub.publish(score);
		std::cout<<"save ->"<< score.data <<std::endl;	
		std::cout<<"\033[1;32mrecover pub\033[0m" <<std::endl;
		std_msgs::Empty em;
		recover_pub.publish(em);
	}
	// }
	nx_flag = false;
	std::cout<<std::endl;
}


////// imageをつくるため/////////



void 
Test_pc_run::flagCallback(const std_msgs::BoolConstPtr &msg){
	
	file_count++;
	std::cout<<std::endl;	
	std::cout<<"###########################"<<std::endl;	
	std::cout << "File_Number: " << file_count << "publish" << std::endl;
	pc_publisher(pc_pub, file_count);
	pc_publisher(test_pc_pub, file_count);

	nx_flag = msg->data;

}

