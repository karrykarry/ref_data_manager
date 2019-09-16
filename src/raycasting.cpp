/*
 * Ray_castingをしてpfのために
 * 未知・占有・非占有を算出する
 *
 */

#include"raycasting.hpp"

Raycast::Raycast(ros::NodeHandle n, ros::NodeHandle private_nh_)
{
	grid_pub = n.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 1);
	
	pc_sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_obstacles", 1, &Raycast::pcCallback, this);

    private_nh_.param("RESOLUTION", RESOLUTION, {0.25});
    private_nh_.param("WIDTH", WIDTH, {40.0});
    private_nh_.param("BEAM_NUM", BEAM_NUM, {720});
    private_nh_.param("BUFFER_SIZE", BUFFER_SIZE, {40});

    GRID_WIDTH = WIDTH / RESOLUTION;
    GRID_NUM = GRID_WIDTH * GRID_WIDTH;
    WIDTH_2 = WIDTH / 2.0;
    GRID_WIDTH_2 = GRID_WIDTH / 2.0;
}


void 
Raycast::get_beam_list(const CloudXYZIPtr& input_cloud, std::vector<double>& _beam_list)
{
    _beam_list = std::vector<double>(BEAM_NUM, WIDTH);
    const double BEAM_ANGLE_RESOLUTION = 2.0 * M_PI / (double)BEAM_NUM;
    for(const auto& pt: input_cloud->points){
        double distance = sqrt(pt.x * pt.x + pt.y * pt.y);
        if(-WIDTH_2 <= pt.x && pt.x <= WIDTH_2 && -WIDTH_2 <= pt.y && pt.y <= WIDTH_2){
            double angle = atan2(pt.y, pt.x);
            int beam_index = (angle + M_PI) / BEAM_ANGLE_RESOLUTION;
            if(0 <= beam_index && beam_index < BEAM_NUM){
                if(_beam_list[beam_index] > distance){
                    _beam_list[beam_index] = distance;
                }
            }
        }
    }
}


nav_msgs::OccupancyGrid
Raycast::input_cloud_to_grid_cells(const CloudXYZIPtr& input_cloud ){

    std::vector<int> states(GRID_NUM, UNKNOWN);
    std::vector<double> beam_list;
 	get_beam_list(input_cloud, beam_list);
	
	
	nav_msgs::OccupancyGrid grid;
	grid.header = pcl_conversions::fromPCL(input_cloud->header);
	grid.info.resolution = RESOLUTION;
	grid.info.width = GRID_WIDTH;
	grid.info.height = GRID_WIDTH;
	grid.info.origin.position.x = -WIDTH * 0.5;
	grid.info.origin.position.y = -WIDTH * 0.5;
	grid.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);

	for(const auto& pt : input_cloud->points){
		if(-WIDTH_2 <= pt.x && pt.x <= WIDTH_2 && -WIDTH_2 <= pt.y && pt.y <= WIDTH_2){
			int index = get_index_from_xy(pt.x, pt.y);
			if(0 <= index && index < GRID_NUM) states[index] = OCCUPIED;
		}
	}

	const double BEAM_ANGLE_RESOLUTION = 2.0 * M_PI / (double)BEAM_NUM;
	
	for(int j=0;j<BEAM_NUM;j++){
		double angle = j * BEAM_ANGLE_RESOLUTION - M_PI;
		angle = atan2(sin(angle), cos(angle));

		for(double range = 0.0; range<beam_list[j]; range += RESOLUTION){
			
			double x = range * cos(angle);
			double y = range * sin(angle);
			
			if(-WIDTH_2 <= x && x <= WIDTH_2 && -WIDTH_2 <= y && y <= WIDTH_2){
				
				int index = get_index_from_xy(x, y);
				if(0 <= index && index < GRID_NUM) states[index] = CLEAR;
			}
		}
	}

	for(auto state : states){
		if(state == UNKNOWN) grid.data.push_back(UNKNOWN);
		else grid.data.push_back(state);
	}


	return grid;
}


void
Raycast::pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg){

	CloudXYZIPtr obstacles_cloud(new CloudXYZI);
	pcl::fromROSMsg(*msg, *obstacles_cloud);


	nav_msgs::OccupancyGrid grid;
	grid = input_cloud_to_grid_cells(obstacles_cloud);

	grid_pub.publish(grid);

}
