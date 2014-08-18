#include <ros/ros.h>
#include "ScrollingGarment.h"


// void getListOfPoints(std::vector< geometry_msgs::Point>& waypoints_1,std::vector< geometry_msgs::Point>& waypoints_2){
// 	geometry_msgs::Point blank;

// 	for (int i=0; i < 10; i++){
// 		blank.x = -0.9;
// 		blank.y = 0.4;
// 		waypoints_1.push_back(blank);
// 		blank.y = 0.3;
// 		waypoints_1.push_back(blank);
// 		blank.x = -1.5;
// 		blank.y = 0.4;
// 		waypoints_1.push_back(blank);
// 		blank.x = -0.9;
// 		blank.y = 0.5;
// 		waypoints_1.push_back(blank);

// 		blank.x = -0.9;
// 		blank.y = -0.4;
// 		waypoints_2.push_back(blank);
// 		blank.y = -0.3;
// 		waypoints_2.push_back(blank);
// 		blank.x = -1.5;
// 		blank.y = -0.4;
// 		waypoints_2.push_back(blank);
// 		blank.x = -0.9;
// 		blank.y = -0.3;
// 		waypoints_2.push_back(blank);
// 	}
// }

int main(int argc, char **argv) {
	ros::init(argc, argv, "scrolling_garment_on_the_table");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();


	// std::string frame_id = "ctu_floor";
	// std::string table_frame = "t3_desk";
	// std::vector< geometry_msgs::Point > waypoints_1, waypoints_2;
	// getListOfPoints(waypoints_1, waypoints_2);

	ScrollingGarment sg;
	// sg.moveOverTable(frame_id,	waypoints_1, waypoints_2, table_frame, 10);

	ROS_INFO_STREAM("GOOD BYE");
	ros::spinOnce();
	return 0;
}