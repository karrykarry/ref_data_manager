/* test_pc_run.cpp
 *
 * 2019.08.29
 *
 * author : R.Kusakari
 *
*/ 
#include<ros/ros.h>
#include"test_pc_run.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "test_pc_run");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

    ROS_INFO("\033[1;32m---->\033[0m test_pc_run Started.");
	
	Test_pc_run test_pc_run(n,priv_nh);	
    
    ros::Rate loop_rate(1);
		
	static int count = 0;
    
	// while(ros::ok()){
	// 	if(test_pc_run.callback_flag){
	// 		test_pc_run.pc_publisher(count);	
	// 		count++;
	// 		// std:: cout << "test" << std::endl;
	// 		// test_pc_run.callback_flag = false;
	// 		// std:: cout << "test" << std::endl;
	// 	}
	// 	ros::spinOnce();
    //     loop_rate.sleep();
    // }

	ros::spin();


    return 0;
}

       







