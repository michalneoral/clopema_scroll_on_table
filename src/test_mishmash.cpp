#include <ros/ros.h>
#include <clopema_robot/robot_commander.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <boost/math/special_functions/fpclassify.hpp>
#include <moveit/robot_state/conversions.h>
#include "ScrollGarmentForceCB.h"
#include "TrajectoryPublisherScroll.h"
#include <tf_conversions/tf_eigen.h>
#include "ScrollGarment.h"

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
	ros::init(argc, argv, "test_mishmash");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// std::string frame_id = "base_link";
	// std::string table_frame = "t3_desk";
	// std::vector< geometry_msgs::Point > waypoints_1, waypoints_2;
	// getListOfPoints(waypoints_1, waypoints_2);

	// ScrollGarment sg;
	// sg.table_frame_ = table_frame;
	// ROS_INFO_STREAM("Good Bye");
	
	std::cout << "\033[1;33mbold red text\033[0m\n" << std::endl;
		
	// clopema_robot::ClopemaRobotCommander ext("ext");
	// ext.setNamedTarget("ext_minus_90");
	// ext.move();

	// clopema_robot::ClopemaRobotCommander crc("arms");
	// crc.setPoseReferenceFrame("base_link");    
	// geometry_msgs::Pose p1, p2;
	// robot_state::RobotState rs(*crc.getCurrentState());
	// crc.setStartState(rs);

	// p1.position.x = -0.9;
	// p1.position.y = 0.1;
	// p1.position.z = 0.84;
	// p1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/2+M_PI/6, M_PI);
	// ROS_INFO_STREAM(p1.orientation);
	// if (!rs.setFromIK(rs.getJointModelGroup("r1_arm"), p1, "r1_ee")) {
	// 	ROS_WARN_STREAM("Cannot set from IK - first arm");		
	// 	return false;
	// }

	// p2.position.x = -0.9;
	// p2.position.y = -0.1;
	// p2.position.z = 0.84;
	// p2.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/2+M_PI/6, 1.88499);
	// ROS_INFO_STREAM(p2.orientation);
	// if (!rs.setFromIK(rs.getJointModelGroup("r2_arm"), p2, "r2_ee")) {
	// 	ROS_WARN_STREAM("Cannot set from IK - second arm");
	// 	return false;
	// }

	// crc.setJointValueTarget(rs);
	// return crc.move();


							ros::spin();
							return 0;
						}