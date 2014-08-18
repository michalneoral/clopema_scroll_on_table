#ifndef SCROLLGARMENT_H
#define SCROLLGARMENT_H

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
// #include <controller_manager.h>



//===========================================
#define HEIGHT (-0.010)// + 0.78) // [m]
#define OVER_TABLE_HEIGHT (0.010)// + 0.78) // [m]
#define START_STOP_HEIGHT (0.050)// + 0.78) // [m]
#define EXTREME_HEIGHT (-0.020)// + 0.78) // [m]
//-------------------------------------------
#define ROLL2DESK_R1 0 //(M_PI)
#define PITCH2DESK_R1 (M_PI/2+M_PI/6) //(M_PI/2-M_PI/6)
#define YAW2DESK_R1  (2*M_PI-3*M_PI/4)//(-M_PI/2)
//-------------------------------------------
#define ROLL2DESK_R2 0 //(M_PI)
#define PITCH2DESK_R2 (M_PI/2+M_PI/6) //(M_PI/2-M_PI/6)
#define YAW2DESK_R2  (3*M_PI/4)//(-M_PI/2)
//-------------------------------------------
#define ANGLE_NUMBER_STEP 10
#define STEP 0.01 // [m]
#define STEP2TABLE 0.001 // [m]
#define JUMP_TRESHOLD 0
//-------------------------------------------
#define SLEEP 1.0 // [s]
#define SLEEP_BEFORE_EXEC 0.1 // [s]
#define SLEEP_AFTER_EXEC 0.1 // [s]
#define FORCE_CONST  0.00015 //0.00035 // [m/N]
//-------------------------------------------
#define X 0
#define Y 1
#define Z 2
#define BASE_TARGET "base_link"
//===========================================

class ScrollGarment {

public:
	ScrollGarment();
	void getListOfCollisions(std::vector<std::string>& elinks1,std::vector<std::string>& elinks2, std::string table_frame);

	void getTestPositions(std::string table_frame , std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, const std::vector< geometry_msgs::Point >& waypoints_1, const	std::vector< geometry_msgs::Point >& waypoints_2, double yawR1, double yawR2, double offset);

	bool testTrajectory(std::string table_frame , double& yawR1, double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, bool& conf);

	void transformFromFrameToTable(std::string frame_id, std::string table_frame, std::vector<geometry_msgs::Point >& waypoints_1, std::vector< geometry_msgs::Point >& waypoints_2);

	geometry_msgs::Quaternion transformQuaternion(std::string table_frame,double roll, double pitch, double yaw);

	bool moveOverTable( std::string frame_id, std::vector< geometry_msgs::Point >& waypoints_1, std::vector< geometry_msgs::Point >& waypoints_2, std::string table_frame, double force);

	bool planPoses(moveit_msgs::RobotTrajectory &trajectories, const std::vector<std::string> elinks1, const std::vector<std::string> elinks2, const std::vector<geometry_msgs::Pose>& wp1, const std::vector<geometry_msgs::Pose>& wp2, const bool first_combination, double step);

	bool goToPosition(std::string table_frame , const double& yawR1, const double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, const bool& conf, bool start);

	int pressOnTheTable(std::string table_frame , const double& yawR1, const double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, bool conf, double force);

	bool scrollOverTable(std::string table_frame , const double& yawR1, const double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, bool conf, double force);

	std::vector<std::string> get_joint_names();

	void add_current_state_to_trajectory(trajectory_msgs::JointTrajectory& trajectory);

	bool get_current_point(trajectory_msgs::JointTrajectoryPoint& p);

	bool joints_to_point(const sensor_msgs::JointState& js, trajectory_msgs::JointTrajectoryPoint& p);

	void cb_joint(const sensor_msgs::JointState& msg);



public:
	clopema_robot::ClopemaRobotCommander crc_;

	std::string table_frame_;

	ScrollGarmentForceCB WrenchR1_;
	ScrollGarmentForceCB WrenchR2_;	

	boost::mutex mutex_joints_;
	sensor_msgs::JointState joints_;

	ros::Subscriber sub_joints_;
	ros::NodeHandle node_;

	boost::mutex mutex_msg_;

	Eigen::Affine3d link_r1_force_sensor_;
	Eigen::Affine3d link_r2_force_sensor_;

	TrajectoryPublisherScroll pub_traj_;

	boost::mutex mutex_z_r1_;
	double z_r1_;
	boost::mutex mutex_z_r2_;
	double z_r2_;
};


#endif