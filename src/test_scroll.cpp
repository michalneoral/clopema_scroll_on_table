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
#include <visualization_msgs/Marker.h>
#include <tf_conversions/tf_eigen.h>

#define STEP 0.005
#define JUMP_TRESHOLD 1.50
#define GO_HOME false

// clopema_robot::ClopemaRobotCommander crc_("arms");

void getListOfPoints1(std::vector<geometry_msgs::Pose>& waypoints_1, std::vector<geometry_msgs::Pose>& waypoints_2){
	//-----------------------------------------------------------------------
	geometry_msgs::Pose blank;

	waypoints_1.clear();
	waypoints_2.clear();
	{
		blank.position.x = -1.4;
		blank.position.y = 0.2;
		blank.position.z = 0.79;
		blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, (M_PI/2+M_PI/6),  4.10846);
		waypoints_1.push_back(blank);
		blank.position.x = -0.9;
		blank.position.z = 0.83;
		waypoints_1.push_back(blank);
		
		blank.position.x = -0.9;
		blank.position.y = -0.2;
		blank.position.z = 0.79;
		blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, (M_PI/2+M_PI/6),  3.62519);
		waypoints_2.push_back(blank);
		blank.position.x = -1.3;
		blank.position.z = 0.83;
		waypoints_2.push_back(blank);
	}
}

void getListOfPoints2(std::vector<geometry_msgs::Pose>& waypoints_1, std::vector<geometry_msgs::Pose>& waypoints_2){
	//-----------------------------------------------------------------------
	geometry_msgs::Pose blank;

	waypoints_1.clear();
	waypoints_2.clear();
	{
		blank.position.x = -1.0;
		blank.position.y = 0.0;
		blank.position.z = 0.79;
		blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, (M_PI/2+M_PI/6),  4.10836);
		waypoints_1.push_back(blank);
		blank.position.x = -1.0;
		blank.position.y = 0.13;
		blank.position.z = 0.83;
		waypoints_1.push_back(blank);
		
		blank.position.x = -1.0;
		blank.position.y = -0.07;
		blank.position.z = 0.79;
		blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, (M_PI/2+M_PI/6),  1.2084);
		waypoints_2.push_back(blank);
		blank.position.x = -1.0;
		blank.position.y = -0.07;
		blank.position.z = 0.83;
		waypoints_2.push_back(blank);
	}
}

void getListOfCollisions(std::vector<std::string>& elinks1,std::vector<std::string>& elinks2, std::string table_frame){
	//-------------------------------------------------------------------------------------------
	elinks1.push_back("r1_gripper");
	elinks1.push_back("r1_ee");
	elinks1.push_back("r1_ee_par");
	elinks1.push_back("r1_gg");
	elinks1.push_back("r2_ps");
	elinks1.push_back("r2_gripper");
	elinks1.push_back("r2_ee");
	elinks1.push_back("r2_ee_par");
	elinks1.push_back("r2_gg");
	for(int i=0; i<elinks1.size(); i++){
		elinks2.push_back(table_frame);
	}
}

bool planPoses(moveit_msgs::RobotTrajectory &trajectories, const std::vector<std::string> elinks1, const std::vector<std::string> elinks2, const std::vector<geometry_msgs::Pose>& wp1, const std::vector<geometry_msgs::Pose>& wp2, bool current_state, clopema_robot::ClopemaRobotCommander& crc_){
	//------------------------------------------------------------------------

	double d;
	std::vector<geometry_msgs::Pose> wp1_copy = wp1, wp2_copy = wp2;
	crc_.setStartStateToCurrentState();
	if (!current_state) {
		robot_state::RobotState rs(*crc_.getCurrentState());
		if(!rs.setFromIK (rs.getJointModelGroup("r1_arm"), wp1_copy.front(), "r1_ee")){
			ROS_ERROR("Error - setFromIK 1 ");
			return false;
		}
		if(!rs.setFromIK (rs.getJointModelGroup("r2_arm"), wp2_copy.front(), "r2_ee")){
			ROS_ERROR("Error - setFromIK 2 ");
			return false;
		}
		wp1_copy.erase(wp1_copy.begin());
		wp2_copy.erase(wp2_copy.begin());
		crc_.setStartState(rs);
	}
	

	d = crc_.computeCartesianPathDual(wp1_copy, "r1_ee", wp2_copy, "r2_ee", STEP, JUMP_TRESHOLD, trajectories, false);

	if(!(fabs(d - 1.0) < 0.001)) {
		ROS_WARN_STREAM("cannot interpolate up. d < 1");
		std::cout << d << std::endl;
		return false;
	} else if(!crc_.check_trajectory(trajectories, elinks1, elinks2)) {
		ROS_WARN_STREAM("cannot interpolate up because of collision");
		return false;
	}
	ROS_INFO_STREAM("interpolate OK!");
	return true;
}

bool goHome(bool go){
	if(!go){
		return true;
	}
	ros::Duration(0.1).sleep();
	clopema_robot::ClopemaRobotCommander arms("arms");
	arms.setNamedTarget("home_arms");
	return arms.move();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "testing_scroll");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	clopema_robot::ClopemaRobotCommander ext("ext");
	ext.setNamedTarget("ext_minus_90");
	ext.move();
	ros::Duration(0.1).sleep();

	clopema_robot::ClopemaRobotCommander crc("arms");

	std::vector<std::string> elinks1;
	std::vector<std::string> elinks2;
	getListOfCollisions(elinks1, elinks2, "t3_desk");

	goHome(true);

	std::vector<geometry_msgs::Pose> wp1, wp2;
	moveit_msgs::RobotTrajectory trajectory;
	
	getListOfPoints1(wp1, wp2);	
	planPoses(trajectory, elinks1, elinks2, wp1, wp2, false, crc);
	wp1.erase(wp1.end());
	wp2.erase(wp2.end());
	if(planPoses(trajectory, elinks1, elinks2, wp1, wp2, true, crc)){
		crc.execute_traj(trajectory.joint_trajectory);
	}

	
	goHome(GO_HOME);


	getListOfPoints2(wp1, wp2);
	planPoses(trajectory, elinks1, elinks2, wp1, wp2, false, crc);
	wp1.erase(wp1.end());
	wp2.erase(wp2.end());
	if(planPoses(trajectory, elinks1, elinks2, wp1, wp2, true, crc)){
		crc.execute_traj(trajectory.joint_trajectory);
	}
	
	std::cout << "\033[1;33mEnd\033[0m" << std::endl; //]]
	ros::spinOnce();
	return 0;
}