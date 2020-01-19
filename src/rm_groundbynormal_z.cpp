#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>

using namespace std;


ros::Publisher pc_pub;
double rm_normal;

void sq_lidarCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr rm_cloud (new pcl::PointCloud<pcl::PointXYZINormal>);

    pcl::fromROSMsg (*input, *input_cloud);
	
	for(auto pc : input_cloud->points){
		if(pc.normal_z > rm_normal){
			rm_cloud->push_back(pc);
		}
	}
	
	sensor_msgs::PointCloud2 pc;
    pcl::toROSMsg(*rm_cloud , pc);           
           
	pc.header.frame_id = "/velodyne"; //laserのframe_id
	pc.header.stamp = input->header.stamp; 

	pc_pub.publish(pc);

}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "rm_groundbynormal_z");
    
    ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");
	priv_nh.param("rm_normal", rm_normal, {-0.9});	
	

    pc_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_obstacles", 1);	
    ros::Subscriber lidar_sub = n.subscribe("/velodyne_points", 1, sq_lidarCallback);


	ros::spin();

	return (0);
}

