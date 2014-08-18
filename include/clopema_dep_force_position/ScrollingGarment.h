#ifndef SCROLLINGGARMENT_H
#define SCROLLINGGARMENT_H

#include <ros/ros.h>
// #include <clopema_robot/robot_commander.h>
// #include <eigen_conversions/eigen_msg.h>
// #include <boost/foreach.hpp>
// #include <boost/thread.hpp>
// #include <iostream>
// #include <geometry_msgs/WrenchStamped.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/Point.h>
// #include <boost/math/special_functions/fpclassify.hpp>
// #include <moveit/robot_state/conversions.h>

// //===========================================
// #define HEIGHT -0.010 // [m]
// #define START_STOP_HEIGHT 0.050 // [m]
// //-------------------------------------------
// #define ROLL2DESK_R1 (M_PI)
// #define PITCH2DESK_R1 (M_PI/2-M_PI/6)
// #define YAW2DESK_R1 (M_PI/2-M_PI/4)  //(-M_PI)
// //-------------------------------------------
// #define ROLL2DESK_R2 (M_PI)
// #define PITCH2DESK_R2 (M_PI/2-M_PI/6)
// #define YAW2DESK_R2 (-M_PI/2+M_PI/4) //(-M_PI)
// //-------------------------------------------
// #define ANGLE_NUMBER_STEP 1
// #define STEP 0.01
// #define JUMP_TRESHOLD 0
// //===========================================

class ScrollingGarment {

public:
	ScrollingGarment();

	// void getListOfCollisions(std::vector<std::string>& elinks1,std::vector<std::string>& elinks2, std::string table_frame);

	// void getPositions(std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, std::vector< geometry_msgs::Point >& waypoints_1, std::vector< geometry_msgs::Point >& waypoints_2, double yawR1, double yawR2, double offset);

	// void transformFromFrameToTable(std::string frame_id, std::string table_frame, std::vector<geometry_msgs::Point >& waypoints_1,	std::vector< geometry_msgs::Point >& waypoints_2);

	// void moveOverTable( std::string frame_id, std::vector< geometry_msgs::Point >& waypoints_1, std::vector< geometry_msgs::Point >& waypoints_2, std::string table_frame, double force);

	// bool planPoses(moveit_msgs::RobotTrajectory &trajectories, std::vector<std::string> elinks1, std::vector<std::string> elinks2, std::vector<geometry_msgs::Pose> wp1, std::vector<geometry_msgs::Pose> wp2, bool first_combination);

public:
	// clopema_robot::ClopemaRobotCommander crc_;
};


#endif