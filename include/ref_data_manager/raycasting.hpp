#ifndef _RAYCASTING_HPP_
#define _RAYCASTING_HPP_

#include<iostream>
#include<vector>
#include<sstream>
#include<iomanip>
#include<fstream>

#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<nav_msgs/OccupancyGrid.h>

#include<pcl_ros/transforms.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_types_conversion.h>
#include<pcl/point_types.h>



class Raycast{
	
	public:
		Raycast(ros::NodeHandle n,ros::NodeHandle priv_nh);

		void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
		
		typedef pcl::PointXYZI PointXYZI;
		typedef pcl::PointCloud<PointXYZI> CloudXYZI;
		typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;

		int get_index_from_xy(const double x, const double y){
			int _x = floor(x / RESOLUTION + GRID_WIDTH_2 + 0.5);
			int _y = floor(y / RESOLUTION + GRID_WIDTH_2 + 0.5);
			return _y * GRID_WIDTH + _x;
		}

		nav_msgs::OccupancyGrid input_cloud_to_grid_cells(const CloudXYZIPtr&);


	private:
		ros::Publisher grid_pub;
		ros::Subscriber pc_sub;

		double RESOLUTION;
		double WIDTH;
		double WIDTH_2;
		int GRID_WIDTH;
		int GRID_WIDTH_2;
		int GRID_NUM;
		int BEAM_NUM;
		int BUFFER_SIZE;

		static const int UNKNOWN = -1;
		static const int CLEAR = 0;
		static const int OCCUPIED = 100;

    
    	void get_beam_list(const CloudXYZIPtr&, std::vector<double>&);

};

#endif








