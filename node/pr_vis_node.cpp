/* similarity_score.cpp
 *
 * 2019.07.16
 *
 * author : R.Kusakari
 *
*/ 
#include<ros/ros.h>
#include<pr_vis.hpp>


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pr_vis_node");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");
	// ros::Rate loop(10);

    ROS_INFO("\033[1;32m---->\033[0m pr_vis Started.");
	
	PR_vis pr_vis(n,priv_nh);	

	ros::spin();
 
	return 0;
}

       




