/*
 *itstでnodeの判定したら、すべてをpubする管理ソース
 （そこでのscan_contextをrefにするソース）
 *
 */
#include"itst2data_pub.hpp"

Itst2data_pub::Itst2data_pub(ros::NodeHandle n,ros::NodeHandle private_nh_)
{

	pr_num_pub = n.advertise<visualization_msgs::MarkerArray>("/pr/num/vis", 10, true);
	better_estimate_num_pub = n.advertise<visualization_msgs::MarkerArray>("/pr/better_estimate_num/vis", 10);
	best_estimate_num_pub = n.advertise<visualization_msgs::Marker>("/pr/best_estimate_num", 10);
	
	best_image_path_pub = n.advertise<std_msgs::String>("pr/image/path", 10);	//
	
	
	best_score_sub = n.subscribe<std_msgs::Int32>("/score/best", 1, &Itst2data_pub::bestscorecallback, this);
	better_score_sub = n.subscribe<std_msgs::Int32MultiArray>("/score/better", 1, &Itst2data_pub::betterscorecallback, this);
	all_score_sub = n.subscribe<std_msgs::Float64MultiArray>("/score/vis", 1, &Itst2data_pub::allscorecallback, this);
	
	
	private_nh_.param("PR_INFO/FILE_DIR", file_dir, {"/home/amsl/Pictures/ros_catkin_ws/ref_data_manager"});
	private_nh_.param("PR_INFO/FILE_DIR2", file_dir2, {"/sample"});
	private_nh_.param("PR_INFO/FILE_DIR3", file_dir3, {"/pose"});
	private_nh_.param("PR_INFO/FILE_NAME", file_name, {"/list"});
	private_nh_.param("PR_INFO/FILE_EXT", file_ext, {".txt"});

	private_nh_.param("Number_of_candidate", num_candidate, {5});
	private_nh_.param("IMAGE_FLAG", IMAGE_FLAG, {false});

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
	m.color.g = 1.0;
	m.color.b = 1.0;
	m.color.a = 1.0; 

	m.pose.orientation.x = 0.0;
	m.pose.orientation.y = 0.0;
	m.pose.orientation.z = 0.0;
	m.pose.orientation.w = 1.0;
	m.pose.position.x = now_x;
	m.pose.position.y = now_y;	
	m.pose.position.z = now_z;

	m.scale.z = 6.0; 

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
	pr_num_ = num;
	pr_num_pub.publish(m_array);
	std::cout<<"------finish "<<PR_list_filename<< "--------"<<std::endl;
}


void
Itst2data_pub::color_change(visualization_msgs::Marker& marker, double r_val, double g_val, double b_val, double a_val){

	marker.color.r = r_val;
	marker.color.g = g_val;
	marker.color.b = b_val;
	marker.color.a = a_val; 

}

void
Itst2data_pub::best_num_vis(const int num){
	
	visualization_msgs::Marker est_num;	//estimate_num
	est_num =  make_vis_marker(pr_poses[num].x, pr_poses[num].y, pr_poses[num].z, num, 0);

	color_change(est_num, 1.0, 1.0, 1.0, 1.0);	//white
	
	best_estimate_num_pub.publish(est_num);
}


void
Itst2data_pub::itst2context(const int num){
	const std::string context_dir = "/scan_context/mono/";
	std::ostringstream oss;
	oss << std::setfill( '0' ) << std::setw( 3 ) << num;
	
	std::string output_file;
	output_file = file_dir; 
	output_file += file_dir2; 
	output_file += context_dir; 
	output_file += oss.str() + ".png";

	std_msgs::String image_path;
	image_path.data  = output_file;
	best_image_path_pub.publish(image_path);

	if(IMAGE_FLAG){
		cv::Mat input_img = cv::imread(output_file,0);
		cv::imshow("best_estimate_image.png", input_img);
		cv::waitKey(1);
	}


}




void
Itst2data_pub::bestscorecallback(const std_msgs::Int32ConstPtr &msg){
	
	best_num_vis(msg->data);
	
}

void 
Itst2data_pub::betterscorecallback(const std_msgs::Int32MultiArrayConstPtr &msgs){

	std_msgs::Int32MultiArray better_scores = *msgs;

	better_scores.data.erase(better_scores.data.begin());// 一番scoreが高いもの(best)を削除

	std::vector<bool> flag(pr_num_, false);

	visualization_msgs::MarkerArray better_array;
	visualization_msgs::MarkerArray remain_array;
	visualization_msgs::Marker num_m;

	int cnt_=0;
	for(auto msg : msgs->data){
		int num = msg;
		if(!flag[num]){
			num_m = make_vis_marker(pr_poses[num].x, pr_poses[num].y, pr_poses[num].z, num, cnt_);
			cnt_++;

			color_change(num_m, 1.0, 1.0, 0.0, 1.0);	//red
			better_array.markers.push_back(num_m);
			flag[num]=true;
		}
	}
	
	better_estimate_num_pub.publish(better_array);
	
	// itst2context(msg->data);
	
	cnt_=0;
	for(int i=0;i<=pr_num_;i++){
		if(flag[i]) continue;
		num_m = make_vis_marker(pr_poses[i].x, pr_poses[i].y, pr_poses[i].z, i, cnt_);
		cnt_++;
		
		color_change(num_m, 0.0, 1.0, 0.0, 1.0);	//green
		remain_array.markers.push_back(num_m);
	}


	pr_num_pub.publish(remain_array);

}




//今の所、スコアが同じだと上書きされてしまう
//このcallbackが上位複数のを表示する以外にするのであれば関数化する
void
Itst2data_pub::allscorecallback(const std_msgs::Float64MultiArrayConstPtr &msg){
	
	std::map<double ,int> score2pr_num;	// double score, int pr_num
	
	std::vector<double> scores;	// double score, int pr_num

	int pr_num=0;
	double min_score = 1000;	//適当に大きい数字を入れた
	for(auto score : msg->data){
		double high_sort_score = 1.0/msg->data[pr_num]; //nth_element は小さい順にsortするから逆数を取った
		min_score = MIN_(min_score, high_sort_score);
		scores.push_back(high_sort_score);

		score2pr_num[high_sort_score] = pr_num;
		pr_num++;
	}
	
	std::nth_element(scores.begin(), scores.begin() + num_candidate, scores.end());

	// scores.resize(num_candidate);	//pubの制限

	visualization_msgs::MarkerArray buffer_array;
	visualization_msgs::MarkerArray remain_array;
	visualization_msgs::Marker buffer_m;
	
	int id_num=1;
	
	for(auto score : scores){
		if(score != min_score){
			pr_num = score2pr_num[score]; 	//prの番号を入れる（下のコードが長くなるから）

			buffer_m = make_vis_marker(pr_poses[pr_num].x, pr_poses[pr_num].y, pr_poses[pr_num].z, pr_num, id_num);
			if(id_num < num_candidate){	//scoreが上位なもの
				color_change(buffer_m, 0.0, 1.0, 0.0, 1.0);	//green

				buffer_array.markers.push_back(buffer_m);
			}
			else {						//scoreが下位なもの
				color_change(buffer_m, 1.0, 1.0, 0.0, 1.0);	//red

				remain_array.markers.push_back(buffer_m);
			}
			id_num++;
		}

		// std::cout<<rank<<"位:"<<msg->data[score2pr_num[score]]<<":"<<score2pr_num[score]<<std::endl;
		// rank++;
	}	
	better_estimate_num_pub.publish(buffer_array);
	pr_num_pub.publish(remain_array);

	// std::cout<<std::endl;
}
