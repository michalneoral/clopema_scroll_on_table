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
#include "ForceCallbackStarter.h"

#define MAX_FORCE_EXP_PROXIMITY 30
#define MIN_HEIGHT 0.775
#define MAX_HEIGHT 1.2
#define R1_X (-0.9)
#define R2_X (-0.9)
#define R1_Y 0.3
#define R2_Y (-0.3)
#define R1_yaw 4.1
#define R2_yaw (2*M_PI - 4.1)
#define PITCH (M_PI/2+M_PI/6)

bool goToPosition(clopema_robot::ClopemaRobotCommander& crc_){
	robot_state::RobotState rs(*crc_.getCurrentState());
	geometry_msgs::Pose blank, p1, p2;

	crc_.setPoseReferenceFrame("base_link");

	blank.position.x = R1_X;
	blank.position.y = R1_Y;
	blank.position.z = MAX_HEIGHT;
	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, PITCH, R1_yaw);
	p1 = blank;

	blank.position.x = R2_X;
	blank.position.y = R2_Y;
	blank.position.z = MAX_HEIGHT;
	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, PITCH, R2_yaw);
	p2 = blank;	
	
	if (!rs.setFromIK(rs.getJointModelGroup("r1_arm"), p1, "r1_ee")) {
		ROS_WARN_STREAM("Cannot set from IK - first arm");
		return false;
	}
	if (!rs.setFromIK(rs.getJointModelGroup("r2_arm"), p2, "r2_ee")) {
		ROS_WARN_STREAM("Cannot set from IK - second arm");
		return false;
	}
	crc_.setJointValueTarget(rs);
	return crc_.move();
}

bool doExperiment(bool down, clopema_robot::ClopemaRobotCommander& crc){
	
	crc.setRobotSpeed(0.02);
	goToPosition(crc);
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_mishmash");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	clopema_robot::ClopemaRobotCommander ext("ext");
	ext.setNamedTarget("ext_minus_90");
	ext.move();
	ros::Duration(0.1).sleep();

	clopema_robot::ClopemaRobotCommander crc("arms");	
	ros::Duration(0.1).sleep();

 	ForceCallbackStarter sg;

	if(!doExperiment(true, crc)){
		return false;
	}


	//ros::spinOnce();
	spinner.stop();
	return 0;
}