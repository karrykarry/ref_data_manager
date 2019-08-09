
#include"itst2scan.hpp"

Itst2scan::Itst2scan(ros::NodeHandle n,ros::NodeHandle private_nh_)
{
	best_num_sub = n.subscribe<std_msgs::Int32>("/score/itst/best", 1, &Itst2scan::itstcallback, this);
	
	
	private_nh_.param("save_path", SAVE_PATH, {"/home/amsl/Picture/ros_catkin_ws/ref_data_manager"});
	private_nh_.param("where_dir", W_DIR, {"/d_kan_around"});	

}

Itst2scan::~Itst2scan(){
}


void
Itst2scan::itstcallback(const std_msgs::Int32ConstPtr &msg){
	

}
