#ifndef SCROLLGARMENTSINGLEPATHCONFIG_H
#define SCROLLGARMENTSINGLEPATHCONFIG_H

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

class ScrollGarmentSinglePathConfig {

public:
	ScrollGarmentSinglePathConfig();

	double getPercentage();

	void changeWaypoint1(geometry_msgs::Point p1, geometry_msgs::Point p2);

	void changeWaypoint2(geometry_msgs::Point p1, geometry_msgs::Point p2);

	lowerTrajPercentage_ = 0;
	upperTrajPercentage_ = 0;

public:
	// moveit_msgs::RobotTrajectory lowerTraj_;
	// moveit_msgs::RobotTrajectory upperTraj_;
	
	double lowerTrajPercentage_;
	double upperTrajPercentage_;
	double yawR1_;
	double yawR2_;
	bool configuration_;

	std::vector< geometry_msgs::Point > short_waypoints_1_;
	std::vector< geometry_msgs::Point > short_waypoints_2_;

};


#endif