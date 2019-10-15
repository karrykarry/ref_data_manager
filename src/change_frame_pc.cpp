/* vis_laser.cpp
 *
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>

#include <iostream>

using namespace std;

class Laser
{
	private:
		ros::Subscriber laser_sub;
		ros::Publisher laser_pub;

		string HEADER_FRAME;
	
		sensor_msgs::PointCloud2 buffer_pc;

	public:
		Laser(ros::NodeHandle n,ros::NodeHandle priv_nh);
		void laserCallback(const sensor_msgs::PointCloud2Ptr msg);
		void resultCallback(const geometry_msgs::PoseConstPtr& msg);

};

Laser::Laser(ros::NodeHandle n,ros::NodeHandle priv_nh)
{
	priv_nh.param("header_frame",HEADER_FRAME,{"/context_estimate"});

	laser_sub = n.subscribe("velodyne_points", 10, &Laser::laserCallback, this);
	laser_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points/context_res", 10);
}


void
Laser::laserCallback(const sensor_msgs::PointCloud2Ptr msg){

	buffer_pc = *msg;

}


void 
Laser::resultCallback(const geometry_msgs::PoseConstPtr& msg){

	buffer_pc.header.stamp =  ros::Time(0);
	buffer_pc.header.frame_id = HEADER_FRAME;

	laser_pub.publish(buffer_pc);

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


