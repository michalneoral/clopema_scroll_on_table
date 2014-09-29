#ifndef SCROLLGARMENTFORCECB_H
#define SCROLLGARMENTFORCECB_H

#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/math/special_functions/fpclassify.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <clopema_robot/robot_commander.h>

#define ABSOLUTE_MAX_FORCE 50.0 // [N]
#define ABSOLUTE_MAX_FORCE_RAW 60 // [N]
#define ABSOLUTE_MAX_TORQUE_RAW 20 // [Nm]
#define RELATIVE_MAX_FORCE 0.15 // [-]

class ScrollGarmentForceCB {

public:
	ScrollGarmentForceCB();
	void cb_force(const geometry_msgs::WrenchStamped& msg);
	void startSub(std::string name);
	void computeRotateForce();
	double getForce();
	void force_info_stream(Eigen::Vector3d force, Eigen::Vector3d torque);
	void setRotMat(Eigen::Affine3d rot);
	void setInitialize();
	void setLenght();
	bool isForceOk(double force);
	void showForces();
	void emergencyStop();

public:
	std::string force_topic_name_;
	std::string ee_name_;
	std::string sensor_name_;

	boost::mutex mutex_wrench_;
	boost::mutex mutex_computedForce_;

	double computedForce_;

	boost::mutex mutex_forces_;
	Eigen::Vector3d forces_;
	boost::mutex mutex_torques_;
	Eigen::Vector3d torques_;
	Eigen::Vector3d init_forces_;
	Eigen::Vector3d init_torques_;

	ros::Subscriber sub_force_;
	ros::NodeHandle node_;

	boost::mutex mutex_rotateMatrix_;
	Eigen::Affine3d rotateMatrix_;

	double lenght_;

	bool ready_;

	boost::mutex mutex_stop_on_;
	bool stop_on_;
};

#endif