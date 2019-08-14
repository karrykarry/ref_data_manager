/* itst2scan.cpp
 *
 * 2019.08.08
 *
 * author : R.Kusakari
 *
*/ 
#include<ros/ros.h>
#include"itst2scan.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "itst2scan");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

    ROS_INFO("\033[1;32m---->\033[0m itst2data_pub Started.");
	
	Itst2data_pub itst2data_pub(n,priv_nh);	

	ros::spin();
 
	return 0;
}

       





