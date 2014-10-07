#ifndef FORCECALLBACKSTARTER_H
#define FORCECALLBACKSTARTER_H

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
#include "ScrollGarmentForceCB.h"
#include "ScrollGarmentSinglePathConfig.h"
#include "TrajectoryPublisherScroll.h"
#include <tf_conversions/tf_eigen.h>


class ForceCallbackStarter {

public:
	ForceCallbackStarter();

	void cb_joint(const sensor_msgs::JointState& msg);

	// bool joints_to_point(const sensor_msgs::JointState& js, trajectory_msgs::JointTrajectoryPoint& p);

	// std::vector<std::string> get_joint_names();

	// bool get_current_point(trajectory_msgs::JointTrajectoryPoint& p);

	// void add_current_state_to_trajectory(trajectory_msgs::JointTrajectory& trajectory);

	


public:
	// std::string table_frame_;
	// ScrollGarmentForceCB WrenchR1_;
	// ScrollGarmentForceCB WrenchR2_;	
	clopema_robot::ClopemaRobotCommander crc_;


private:
	boost::mutex mutex_joints_;
	sensor_msgs::JointState joints_;
	ros::NodeHandle node_;
	ros::Subscriber sub_joints_;

	// boost::mutex mutex_msg_;

	// Eigen::Affine3d link_r1_force_sensor_;
	// Eigen::Affine3d link_r2_force_sensor_;

	// TrajectoryPublisherScroll pub_traj_;
	// ros::Publisher pub_marker_;
};

#endif