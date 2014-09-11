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

// double findPath(trajectory_msgs::JointTrajectory& trajectory){

// }


int main(int argc, char **argv) {
	ros::init(argc, argv, "test_mishmash");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// std::string frame_id = "base_link";
	std::string table_frame = "t3_desk";
	// std::vector< geometry_msgs::Point > waypoints_1, waypoints_2;
	// getListOfPoints(waypoints_1, waypoints_2);

	// ScrollGarment sg;
	// sg.table_frame_ = table_frame;
	// while(ros::ok()){
	// 	sg.showForces();
	// 	ros::Duration(0.1).sleep();
	// }

	// ROS_INFO_STREAM("Good Bye");

	clopema_robot::ClopemaRobotCommander ext("ext");
	ext.setNamedTarget("ext_minus_90");
	ext.move();

	clopema_robot::ClopemaRobotCommander home("arms");
	home.setNamedTarget("home_arms");
	home.move();

	std::vector<std::string> joint_names;
	joint_names.push_back("r1_joint_s");
	joint_names.push_back("r1_joint_l");
	joint_names.push_back("r1_joint_u");
	joint_names.push_back("r1_joint_r");
	joint_names.push_back("r1_joint_b");
	joint_names.push_back("r1_joint_t");
	joint_names.push_back("r2_joint_s");
	joint_names.push_back("r2_joint_l");
	joint_names.push_back("r2_joint_u");
	joint_names.push_back("r2_joint_r");
	joint_names.push_back("r2_joint_b");
	joint_names.push_back("r2_joint_t");
	joint_names.push_back("ext_axis");

	clopema_robot::ClopemaRobotCommander crc("arms");
	crc.setPoseReferenceFrame("base_link");    
	geometry_msgs::Pose p;
	robot_state::RobotState rs(*crc.getCurrentState());
	for(int i=0; i<joint_names.size(); i++){
		std::cout << joint_names[i] << ": "<< rs.getVariablePosition (joint_names[i]) << std::endl; 
	}
	crc.setStartState(rs);

	// p.position.x = -0.9;
	// p.position.y = 0.1;
	// p.position.z = 0.84;
	// p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/2+M_PI/6, M_PI);
	
	// if (!rs.setFromIK(rs.getJointModelGroup("r1_arm"), p1, "r1_ee")) {
	// 	ROS_WARN_STREAM("Cannot set from IK - first arm");		
	// 	return false;
	// }

	// d = crc_.computeCartesianPath(wp1_copy, tip_1, wp2_copy, tip_2, step, JUMP_TRESHOLD, trajectories, false);

	// if(!(fabs(d - 1.0) < 0.001)) {
	// 	sprintf(buffer, "cannot interpolate up. d = %.4f (%.4f).", d, fabs(d - 1.0));
	// 	ROS_WARN_STREAM(buffer);
	// 	// std::cout << "wp1: " << wp1.size() << " wp2: " << wp2.size() << " wp1_copy: " << wp1_copy.size() << " wp2_copy: " << wp2_copy.size() << std::endl;
	// 	return false;
	// } else{
	// 	if(!crc_.check_trajectory(trajectories, elinks1, elinks2)) {
	// 		sprintf(buffer, "cannot interpolate up because of collision");
	// 		ROS_WARN_STREAM(buffer);
	// 		return false;
	// 	}
	// 	else{
	// 		return true;
	// 	}
	// }

	// crc.setJointValueTarget(rs);
	// return crc.move();


	ros::spinOnce();
	return 0;
}