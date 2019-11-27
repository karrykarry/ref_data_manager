/*
 * loam から求まるtf をodometryに変換する
 *
*/
#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class Tf2odom{
	private:
		ros::Publisher odom_pub;
		tf::TransformListener listener;
		tf::StampedTransform buffer_transform;

		std::string CHILD_FRAME, PARENT_FRAME;

		nav_msgs::Odometry odo;
		
	public:
		Tf2odom(ros::NodeHandle n, ros::NodeHandle private_nh_);

		void listen_tf();

};

Tf2odom::Tf2odom(ros::NodeHandle n, ros::NodeHandle private_nh_)
{
	odom_pub = n.advertise<nav_msgs::Odometry>("/odometer", 100);
	
	private_nh_.param("PARENT_FRAME", PARENT_FRAME, {"/map"});
	private_nh_.param("CHILD_FRAME", CHILD_FRAME, {"/base_link"});


	odo.header.frame_id = PARENT_FRAME;

}


void
Tf2odom::listen_tf()
{

	ros::Time time_now = ros::Time(0);
	try{
		listener.waitForTransform(PARENT_FRAME, CHILD_FRAME, time_now, ros::Duration(0.5));
		
		listener.lookupTransform(PARENT_FRAME, CHILD_FRAME,  
				time_now, buffer_transform);
		odo.header.stamp = time_now;
		odo.pose.pose.position.x = buffer_transform.getOrigin().x();
		odo.pose.pose.position.y = buffer_transform.getOrigin().y();
		odo.pose.pose.position.z = buffer_transform.getOrigin().z();
		odo.pose.pose.orientation.x = buffer_transform.getRotation().x();
		odo.pose.pose.orientation.y = buffer_transform.getRotation().y();
		odo.pose.pose.orientation.z = buffer_transform.getRotation().z();
		odo.pose.pose.orientation.w = buffer_transform.getRotation().w();

		odom_pub.publish(odo);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "tf2odom");
	ros::NodeHandle n;
	ros::NodeHandle private_nh_("~");
    ROS_INFO("\033[1;32m---->\033[0m tf2odom Started.");
	
	Tf2odom tf2odom(n, private_nh_);

    ros::Rate loop_rate(100);
		
	while(ros::ok()){
		tf2odom.listen_tf();
		ros::spinOnce();
        loop_rate.sleep();
    }

	return (0);
}


