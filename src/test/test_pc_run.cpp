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
	file_count(400),
	pose_true_cnt(0), yaw_true_cnt(0), all_true_cnt(0),
	nx_flag(true)
{
	pc_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);
	score_pub = n.advertise<std_msgs::Int32>("/score/process", 10);
	test_pc_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points/test", 10);
	recover_pub = n.advertise<std_msgs::Empty>("/process", 10);
	gt_pub = n.advertise<visualization_msgs::Marker>("/pose/ground_truth", 10);

	epllipse_sub = n.subscribe("/error_epllipse/self_pose", 1, &Test_pc_run::eplCallback, this);
	pose_sub = n.subscribe<geometry_msgs::Pose>("/map2context_result", 1, &Test_pc_run::poseCallback, this);
	flag_sub = n.subscribe<std_msgs::Bool>("/next_pcd", 1, &Test_pc_run::flagCallback, this);

	private_nh_.param("TEST/FILE_DIR", file_dir, {"/home/amsl/Pictures/ros_catkin_ws/ref_data_manager"});
	private_nh_.param("TEST/FILE_DIR2", file_dir2, {"/sample"});
	private_nh_.param("TEST/PC_FILE_DIR", pc_file_dir, {"/test_pcd"});
	private_nh_.param("TEST/ODOM_FILE_DIR", odom_file_dir, {"/test_pose"});
	private_nh_.param("TEST/ODOM_LIST", odom_list, {"/list.txt"});
	
	private_nh_.param("IS_DATASET", IS_DATASET, {false});
	private_nh_.param("IS_CLEANUP", IS_CLEANUP, {false});

	pr_list_pub();
	
	if(IS_CLEANUP) 
		// miss_checker = new Miss_checker("/home/amsl/m2_result/miss_file-2019-9-25-cnnok.txt");
		miss_checker = new Miss_checker("/home/amsl/m2_result/out_file-2018-9-9-bestonly.txt");

	pc_file_name = file_dir;
	pc_file_name += file_dir2;
	pc_file_name += pc_file_dir;

	writing_file.open("/home/amsl/m2_result/context.csv", std::ios::out);
	writing_missnum.open("/home/amsl/m2_result/miss_file.txt", std::ios::out);
	writing_outnum.open("/home/amsl/m2_result/out_file.txt", std::ios::out);
 	ROS_INFO_STREAM("\033[1;32mAre we in dataset mode? :" << IS_DATASET<<"\033[0m");
 	ROS_INFO_STREAM("\033[1;32mAre we Miss_checker mode? :" << IS_CLEANUP<<"\033[0m");
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
		
	std::cout<<"----miss file save now----"<<std::endl;
	for(auto num_ : miss_file_num){
		writing_missnum << num_ << std::endl;
	}

	std::cout<<"----out file save now----"<<std::endl;
	for(auto num_ : out_file_num){
		writing_outnum << num_ << std::endl;
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
Test_pc_run::ground_truth_pub(){
	visualization_msgs::Marker m;

	m.id 	  = 0;
	m.type   = visualization_msgs::Marker::CYLINDER;
	m.action = visualization_msgs::Marker::ADD;
	m.ns      		   	  = "ground_truth";
	m.header.frame_id 	  = "/map";
	m.pose.position.x    = pr_poses[file_count].position.x;
	m.pose.position.y    = pr_poses[file_count].position.y;
	m.pose.position.z    = pr_poses[file_count].position.z;
	m.pose.orientation.w = 1.0;
	m.scale.x = 0.2; // 0.8
	m.scale.y = 0.2;
	m.scale.z = 1.0; //0.2
	m.color.a = 1.0;
	m.color.r = 0.0;
	m.color.g = 1.0;
	m.color.b = 0.0;

	m.header.stamp = ros::Time(0);

	gt_pub.publish(m);
}



bool
Test_pc_run::diff_pose(const geometry_msgs::Pose est_pose){
	bool result_flag = false;

	std::cout << 
		"estimate(x, y): " << est_pose.position.x << "," << est_pose.position.y << 
		
		"  odometry(x, y): " << pr_poses[file_count].position.x << "," << pr_poses[file_count].position.y <<std::endl;
	

	if( (fabs(est_pose.position.x - pr_poses[file_count].position.x) < 0.5) 
			&& (fabs(est_pose.position.y - pr_poses[file_count].position.y) < 0.5) ){
		result_flag = true;
		score.data = 0;
	}

	else{
		std::cout<<"\033[1;31m"<<std::flush;
		miss_file_num.push_back(file_count);

		if( (fabs(est_pose.position.x - pr_poses[file_count].position.x) < 5) 
				&& (fabs(est_pose.position.y - pr_poses[file_count].position.y) < 5) ){
			score.data = 1;
		}
		else score.data = 2;
	}
		
	std::cout<< "Diffrence(x, y):" << est_pose.position.x - pr_poses[file_count].position.x << ","
		<< est_pose.position.y - pr_poses[file_count].position.y << "\033[0m" << std::endl;

	if(!IS_DATASET){
		diff_x.push_back(fabs(est_pose.position.x - pr_poses[file_count].position.x));
		diff_y.push_back(fabs(est_pose.position.y - pr_poses[file_count].position.y));
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

//誤差楕円は評価の仕方を甘くしているだけ
//分布よりビタイチの精度を求める
void 
Test_pc_run::eplCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg){
	std::cout << "epllipse" << std::endl;

/*

	Eigen::MatrixXf sigma= Eigen::MatrixXf::Zero(2,2);//共分散行列
	Eigen::MatrixXf sigma_uv= Eigen::MatrixXf::Zero(2,2);//分散共分散行列の座標軸の変換するため
	Eigen::Vector2d c_sigma;

	sigma.coeffRef(0,0) = e_xx - ave_x*ave_x; 
	sigma.coeffRef(0,1) = sigma.coeffRef(1,0) = e_xy - ave_x*ave_y; 
	sigma.coeffRef(1,1) = e_yy - ave_y*ave_y; 
////////////////////誤差楕円を書く
//
	sigma_uv.coeffRef(0,0) = sigma.coeffRef(0,0)+sigma.coeffRef(1,1)+
		pow( pow(sigma.coeffRef(0,0)-sigma.coeffRef(1,1), 2)+4*pow(sigma.coeffRef(0,1), 2), 0.5);	
	
	sigma_uv.coeffRef(1,1) = sigma.coeffRef(0,0)+sigma.coeffRef(1,1) -
		pow( pow(sigma.coeffRef(0,0)-sigma.coeffRef(1,1), 2)+4*pow(sigma.coeffRef(0,1), 2), 0.5);

	sigma_uv.coeffRef(0,0) /= 2;
	sigma_uv.coeffRef(1,1) /= 2;

	std::cout<<"sigma_uv:\n"<<sigma_uv<<std::endl;
	c_sigma.x() = pow(MAHALANOBIS_SQRT_DISTANCE*sigma_uv.coeffRef(0,0), 0.5);
	c_sigma.y() = pow(MAHALANOBIS_SQRT_DISTANCE*sigma_uv.coeffRef(1,1), 0.5);
	std::cout<<"c_sigma:\n"<<c_sigma<<std::endl;


	nav_msgs::Odometry epllipse;

	epllipse.header.stamp = ros::Time();
	epllipse.header.frame_id = "/context_estimate";
	epllipse.pose.covariance[0] = c_sigma.x();
	epllipse.pose.covariance[1] = c_sigma.y();
	epllipse.pose.covariance[6] = c_sigma.x();
	epllipse.pose.covariance[7] = c_sigma.y();
	epllipse_pub.publish(epllipse);

	return sigma;


*/


}


//file_count の順番に気をつける
void 
Test_pc_run::poseCallback(const geometry_msgs::PoseConstPtr &msg){

	std::cout << "File_Number: " << file_count << std::endl;

	ground_truth_pub();


	bool pose_flag = diff_pose(*msg);

	double est_yaw, odo_yaw;
	est_yaw = deg2rad(msg->orientation.z);
	odo_yaw = deg2rad(pr_poses[file_count].orientation.z);

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
		file_count++;
		if(IS_CLEANUP) file_count = miss_checker->missfile_num(); 
		std::cout<<"###########################"<<std::endl;	
		std::cout << "File_Number: " << file_count << "publish" << std::endl;
		pc_publisher(pc_pub, file_count);
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
	static bool flag_=false;
	// if(file_count>205) file_count = 195;
	// else if(file_count<194) file_count = 195;
	if(flag_) out_file_num.push_back(file_count);

	file_count++;
	if(IS_CLEANUP) file_count = miss_checker->missfile_num(); 
	std::cout<<std::endl;	
	std::cout<<"↑ out!!!###########################"<<std::endl;	
	std::cout << "File_Number: " << file_count << "publish" << std::endl;
	pc_publisher(pc_pub, file_count);
	pc_publisher(test_pc_pub, file_count);

	nx_flag = msg->data;
	flag_ = true;
}


/////// missの原因を探るため //////////
Test_pc_run::Miss_checker::Miss_checker(std::string input_){
	std::string ref_list = input_;
	std::ifstream reading_file;
	std::string reading_line_buffer;
	
	reading_file.open(ref_list, std::ios::in);
	
	while (!reading_file.eof())
	{
		// read by line
		std::getline(reading_file, reading_line_buffer);
		if(!reading_line_buffer.empty()){
			file_num_list.push_back(atof(reading_line_buffer.c_str()));				
		}
	}
	reading_file.close();

	std::cout<<"########### miss file ###########"<<std::endl;	
	for(auto file_num_ : file_num_list){
		std::cout<<file_num_<<","<<std::flush;
	}
	std::cout<<std::endl;
}

int
Test_pc_run::Miss_checker::missfile_num(){
	static int i = 0;
	static int cnt_=0;
	if(i>=file_num_list.size()) i-= file_num_list.size();
	
	int ans_ = file_num_list[i];
	
	cnt_++;
	// if(cnt_%3==0)
		i++;
	return ans_;
}

