/* vis_laser.cpp
 *
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>

using namespace std;

class Laser
{
	private:
		ros::Subscriber laser_sub;
		ros::Publisher laser_pub;

		string HEADER_FRAME;

	public:
		Laser(ros::NodeHandle n,ros::NodeHandle priv_nh);
		void laserCallback(const sensor_msgs::PointCloud2Ptr msg);

};

Laser::Laser(ros::NodeHandle n,ros::NodeHandle priv_nh)
{
	priv_nh.param("header_frame",HEADER_FRAME,{"/context_estimate"});

	laser_sub = n.subscribe("velodyne_points", 10, &Laser::laserCallback, this);
	laser_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points/context_res", 10);
}


void
Laser::laserCallback(const sensor_msgs::PointCloud2Ptr msg){
	sensor_msgs::PointCloud2 pc;

	pc = *msg;

	pc.header.stamp =  ros::Time(0);
	pc.header.frame_id = HEADER_FRAME;

	laser_pub.publish(pc);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "change_frame_pc");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	cout<<"-------change_frame_pc--------"<<endl;

	Laser laser(n,priv_nh);

    ros::spin();
	
	return 0;
}


