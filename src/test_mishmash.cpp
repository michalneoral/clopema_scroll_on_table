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

void getListOfPoints(std::vector< geometry_msgs::Point>& waypoints_1,std::vector< geometry_msgs::Point>& waypoints_2){
	geometry_msgs::Point blank;

	for (int i=0; i < 10; i++){
		blank.x = -0.9;
		blank.y = 0.4;
		waypoints_1.push_back(blank);
		blank.y = 0.3;
		waypoints_1.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.4;
		waypoints_1.push_back(blank);
		blank.x = -0.9;
		blank.y = 0.5;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.4;
		waypoints_2.push_back(blank);
		blank.y = -0.3;
		waypoints_2.push_back(blank);
		blank.x = -1.5;
		blank.y = -0.4;
		waypoints_2.push_back(blank);
		blank.x = -0.9;
		blank.y = -0.3;
		waypoints_2.push_back(blank);
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_mishmash");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	std::string frame_id = "base_link";
	std::string table_frame = "t3_desk";
	std::vector< geometry_msgs::Point > waypoints_1, waypoints_2;
	getListOfPoints(waypoints_1, waypoints_2);

	ScrollGarment sg;
	sg.table_frame_ = table_frame;
	ROS_INFO_STREAM("Good Bye");

	// {
	// 	std::vector<geometry_msgs::Pose> wp1, wp2;
	// 	geometry_msgs::Pose blank;

	// 	blank.position.x = -0.9;
	// 	blank.position.y = 0.4;
	// 	blank.position.z = 0.85;
	// 	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1);
	// 	wp1.push_back(blank);
	// 	blank.position.z = 0.95;
	// 	wp1.push_back(blank);

	// 	blank.position.x = -0.9;
	// 	blank.position.y = -0.4;
	// 	blank.position.z = 0.85;
	// 	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2); 
	// 	wp2.push_back(blank);
	// 	blank.position.z = 0.95;
	// 	wp2.push_back(blank);


	// 	clopema_robot::ClopemaRobotCommander crc("arms");

	// 	crc.setPoseReferenceFrame("base_link");

	// 	robot_state::RobotState rs(*crc.getCurrentState());

	// 	for(int i=0; i<2; i++){

	// 		if (!rs.setFromIK(rs.getJointModelGroup("r1_arm"), wp1[i], "r1_ee")) {
	// 			ROS_WARN_STREAM("Cannot set from IK - first arm");        
	// 		}
	// 		if (!rs.setFromIK(rs.getJointModelGroup("r2_arm"), wp2[i], "r2_ee")) {
	// 			ROS_WARN_STREAM("Cannot set from IK - second arm");        
	// 		}

	// 		crc.setJointValueTarget(rs);
	// 		ROS_INFO_STREAM(crc.getPoseReferenceFrame());
	// 		crc.move();

	// 		ros::Duration(2.0).sleep();

	// 	}

	// 	moveit_msgs::RobotTrajectory trajectories;
	// 	crc.computeCartesianPathDual(wp1, "r1_ee", wp2, "r2_ee", 0.01, 1.2, trajectories, true);
	// 	crc.execute_traj(trajectories.joint_trajectory);
	// }

	ros::spin();
	return 0;
}