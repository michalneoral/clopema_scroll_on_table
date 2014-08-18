#ifndef TRAJECTORYPUBLISHERSCROLL_H
#define TRAJECTORYPUBLISHERSCROLL_H

#include <ros/ros.h>
#include <clopema_robot/robot_commander.h>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/thread.hpp>


class TrajectoryPublisherScroll {
public:
	TrajectoryPublisherScroll();
	// void execute(const trajectory_msgs::JointTrajectory& trajectory);
	void controle(const trajectory_msgs::JointTrajectory& trajectory);

public:
	// static ros::NodeHandle n_;
	ros::Publisher pub_traj_;
	ros::Publisher pub_rosbag_;
	ros::NodeHandle node_;
};
#endif