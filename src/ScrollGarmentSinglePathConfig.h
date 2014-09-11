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

#define OFFSET 0.001

class ScrollGarmentSinglePathConfig {

public:
	ScrollGarmentSinglePathConfig();

	ScrollGarmentSinglePathConfig(const ScrollGarmentSinglePathConfig&){
	};

	ScrollGarmentSinglePathConfig& operator=(const ScrollGarmentSinglePathConfig& a){
		lowerTrajPercentage_ = a.lowerTrajPercentage_;
		upperTrajPercentage_ = a.upperTrajPercentage_;
		centerTrajPercentage_ = a.centerTrajPercentage_;
		centerTrajSteps_ = a.centerTrajSteps_;
		yawR1_ = a.yawR1_;
		yawR2_ = a.yawR2_;
		configuration_ = a.configuration_;
		short_waypoints_1_ = a.short_waypoints_1_;
		short_waypoints_2_ = a.short_waypoints_2_;
	};

	bool isEqual(const ScrollGarmentSinglePathConfig& a);

	double getPercentage();

	double getLPer();

	double getUPer();

	void changeWaypoint1(geometry_msgs::Point p1, geometry_msgs::Point p2);

	void changeWaypoint2(geometry_msgs::Point p1, geometry_msgs::Point p2);

public:
	// moveit_msgs::RobotTrajectory lowerTraj_;
	// moveit_msgs::RobotTrajectory upperTraj_;
	
	double lowerTrajPercentage_;
	double upperTrajPercentage_;
	double centerTrajPercentage_;
	int centerTrajSteps_;

	double yawR1_;
	double yawR2_;
	bool configuration_;

	std::vector< geometry_msgs::Point > short_waypoints_1_;
	std::vector< geometry_msgs::Point > short_waypoints_2_;

};

#endif