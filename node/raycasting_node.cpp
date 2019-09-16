/* itst2scan.cpp
 *
 * 2019.08.08
 *
 * author : R.Kusakari
 *
*/ 
#include<ros/ros.h>
#include"raycasting.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "raycasting");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

    ROS_INFO("\033[1;32m---->\033[0m Raycasting Started.");
	
	Raycast raycast(n,priv_nh);	

	ros::spin();
 
	return 0;
}

       






