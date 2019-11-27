//
// map座標系で見たvelodyneをpublishする
//

#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


class Trans_pc{
	private:
		ros::Publisher pc_pub;
		ros::Subscriber pc_sub;
		ros::Subscriber odom_sub;

		pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;

		std::string PARENT_FRAME, CHILD_FRAME, VELODYNE_FRAME;
		geometry_msgs::Quaternion buffer_quat;
		
		tf::TransformBroadcaster br;
		tf::Transform transform; 
		bool odom_flag;
		
	public:
		Trans_pc(ros::NodeHandle n, ros::NodeHandle private_nh_);
		void GetRPY(const geometry_msgs::Quaternion &q, double &roll,double &pitch,double &yaw);
		void tf_pub(const geometry_msgs::Point &p,const ros::Time& current_time);
		
		void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& input);
		void odomCallback(const nav_msgs::OdometryConstPtr& msg);
};


Trans_pc::Trans_pc(ros::NodeHandle n, ros::NodeHandle private_nh_):
	input_cloud(new pcl::PointCloud<pcl::PointXYZI>), odom_flag(false)
{	
	pc_pub= n.advertise<sensor_msgs::PointCloud2>("/velodyne_points/trans", 1000);
	pc_sub = n.subscribe("/velodyne_points", 1, &Trans_pc::lidarCallback, this);
	
	odom_sub = n.subscribe("/odometer", 1, &Trans_pc::odomCallback, this);

	private_nh_.param("PARENT_FRAME", PARENT_FRAME, {"/map"});
	private_nh_.param("CHILD_FRAME", CHILD_FRAME, {"/not_rot_baselink"});
	private_nh_.param("VELODYNE_FRAME", VELODYNE_FRAME, {"/not_rot_velodyne"});
}





void 
Trans_pc::GetRPY(const geometry_msgs::Quaternion &q,
    double &roll,double &pitch,double &yaw){
	tf::Quaternion quat(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}


void
Trans_pc::tf_pub(const geometry_msgs::Point &p,const ros::Time& current_time){

	transform.setOrigin( tf::Vector3( p.x, p.y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
    
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, current_time, PARENT_FRAME, CHILD_FRAME));

}

void 
Trans_pc::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
	buffer_quat = msg->pose.pose.orientation;
	tf_pub(msg->pose.pose.position, msg->header.stamp);
	odom_flag = true;
}


void 
Trans_pc::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::fromROSMsg (*input, *input_cloud);
	sensor_msgs::PointCloud2 after_pc;

	if(odom_flag){
		double roll, pitch, yaw;
		GetRPY(buffer_quat, roll, pitch, yaw);

		Eigen::Matrix3f rot;
		rot = Eigen::AngleAxisf(roll*(-1), Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(pitch*(-1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

		Eigen::Translation3f init_translation (0, 0, 0);

		Eigen::Matrix4f transform = (rot * init_translation).matrix ();

		pcl::transformPointCloud (*input_cloud, *transformed_cloud, transform);
		pcl::toROSMsg(*transformed_cloud, after_pc);
	}
	else pcl::toROSMsg(*input_cloud, after_pc);

	after_pc.header.stamp = input->header.stamp;
	// after_pc.header.frame_id = child_frame;
	after_pc.header.frame_id = VELODYNE_FRAME;
	pc_pub.publish(after_pc);
	

}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "trans_pc");
	ros::NodeHandle n;
	ros::NodeHandle private_nh_("~");
    ROS_INFO("\033[1;32m---->\033[0m trans_pc Started.");
	
	Trans_pc trans_pc(n, private_nh_);

	ros::spin();

	return (0);
}

