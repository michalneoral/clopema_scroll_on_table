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
#include <visualization_msgs/Marker.h>
#include "ScrollGarmentForceCB.h"
#include "ScrollGarmentSinglePathConfig.h"
#include "TrajectoryPublisherScroll.h"
#include <tf_conversions/tf_eigen.h>

//===========================================
// #define HEIGHT (-0.010)// + 0.78) // [m]
#define TEST_TRAJECTORY_HEIGHT_MAX (0.000) // [m]
#define TEST_TRAJECTORY_HEIGHT_MIN (-0.030) // [m]
// #define TEST_TRAJECTORY_HEIGHT_MAX (0.090) // [m]
// #define TEST_TRAJECTORY_HEIGHT_MIN (0.030) // [m]
#define OVER_TABLE_HEIGHT (0.010)// + 0.78) // [m]
#define START_STOP_HEIGHT (0.060)// + 0.78) // [m]
#define PROBABLY_HEIGHT (-0.010)// [m]
// #define OVER_TABLE_HEIGHT (0.08)// + 0.78) // [m]
// #define START_STOP_HEIGHT (0.15)// + 0.78) // [m]
#define EXTREME_HEIGHT (-0.020)// + 0.78) // [m]
// ------------------------------------------
#define ADD_HEIGHT_TEST 0.0//(0.10) [m]
//-------------------------------------------
#define ROLL2DESK_R1 0 // [rad]
#define PITCH2DESK_R1 (M_PI/2+M_PI/6) // [rad]
#define YAW2DESK_R1 0 // [rad]
//-------------------------------------------
#define ROLL2DESK_R2 0 // [rad]
#define PITCH2DESK_R2 (M_PI/2+M_PI/6) // [rad]
#define YAW2DESK_R2 0 // [rad]
//-------------------------------------------
#define ANGLE_NUMBER_STEP 13 // [-]
#define MIN_ANGLE_DIFF (-M_PI/2-0.05) // [rad]
#define MAX_ANGLE_DIFF (M_PI/2+0.3) // [rad]
#define STEP 0.005 // [m]
#define STEP_START_STOP 0.05 // [m]
#define TEST_STEP 0.25 // [m]
#define STEP2TABLE 0.001 // [m]
#define JUMP_TRESHOLD 1.50//0.0// [dont know]
#define JUMP_TRESHOLD_NONE 0.0// [dont know]
//-------------------------------------------
#define MIN_TRAJ_2_EXECUTE 0.1 // [-]
#define MIN_POINTS_OF_TRAJ 30 // [-]
#define MAX_TRAJ_2_MAKE_ENDPOINT 0.9 // of found trajectory (not of all) [-]
#define MAX_SUB_TRAJ 0.5 // [-]
//-------------------------------------------
#define SLEEP 1.0 // [s]
#define SLEEP_BEFORE_EXEC 0.1 // [s]
#define SLEEP_AFTER_EXEC 0.1 // [s]
#define FORCE_CONST  0.00035 //0.00035 // [m/N]
#define FORCE_SET_MIN 1.0 // [N]
#define FORCE_SET_MAX 50.0 // [N]
//-------------------------------------------
#define X 0 // [-]
#define Y 1 // [-]
#define Z 2 // [-]
#define BASE_TARGET "base_link"
#define START true
#define STOP false
//-------------------------------------------
#define TESTING_MODE true
#define MINIMAL_DISTANCE 0.0005
//===========================================

class ScrollGarment {

public:
	ScrollGarment();

	bool moveOverTablePiecewise(	std::string frame_id,	std::vector< geometry_msgs::Point >& waypoints_1,	std::vector< geometry_msgs::Point >& waypoints_2,	std::string table_frame,	double force);

	bool moveOverTable( std::string frame_id, std::vector< geometry_msgs::Point >& waypoints_1, std::vector< geometry_msgs::Point >& waypoints_2, std::string table_frame, double force);

	bool testWeight(	std::string frame_id,	std::vector< geometry_msgs::Point >& waypoints_1,
		std::vector< geometry_msgs::Point >& waypoints_2,	std::string table_frame,	double force);

	void showForces();

// private:
	void getListOfCollisions(std::vector<std::string>& elinks1,std::vector<std::string>& elinks2, std::string table_frame);

	int pressOnTheTable(std::string table_frame , const double& yawR1, const double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, bool conf, double force);

	void getTestPositions(std::string table_frame , std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, const std::vector< geometry_msgs::Point >& waypoints_1, const	std::vector< geometry_msgs::Point >& waypoints_2, double yawR1, double yawR2, double offset);

	bool testTrajectory(std::string table_frame , double& yawR1, double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, bool& conf);

	bool findBiggestTrajectory(std::string table_frame , double& yawR1, double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, bool& conf, double& percent_of_path);

	void transformFromFrameToTable(std::string frame_id, std::string table_frame, std::vector<geometry_msgs::Point >& waypoints_1, std::vector< geometry_msgs::Point >& waypoints_2);

	geometry_msgs::Quaternion transformQuaternion(std::string table_frame,double roll, double pitch, double yaw);

	bool planPoses(moveit_msgs::RobotTrajectory &trajectories, const std::vector<std::string> elinks1, const std::vector<std::string> elinks2, const std::vector<geometry_msgs::Pose>& wp1, const std::vector<geometry_msgs::Pose>& wp2, const bool first_combination, double step, bool current_state);

	bool planPoses(moveit_msgs::RobotTrajectory &trajectories, const std::vector<std::string> elinks1, const std::vector<std::string> elinks2, const std::vector<geometry_msgs::Pose>& wp1, const std::vector<geometry_msgs::Pose>& wp2, const bool first_combination, double step, double jump_treshold, bool current_state);

	bool planPoses(moveit_msgs::RobotTrajectory &trajectories, const std::vector<std::string> elinks1, const std::vector<std::string> elinks2, const std::vector<geometry_msgs::Pose>& wp1, const std::vector<geometry_msgs::Pose>& wp2, const bool first_combination, double step, bool current_state, double& percent_of_path, bool& isBiggest);

	bool startStopPosition(std::string table_frame , const double& yawR1, const double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, const bool& conf, bool start);

	bool scrollOverTable(std::string table_frame , const double& yawR1, const double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, bool conf, double force);

	std::vector<std::string> get_joint_names();

	void add_current_state_to_trajectory(trajectory_msgs::JointTrajectory& trajectory);

	bool get_current_point(trajectory_msgs::JointTrajectoryPoint& p);

	bool joints_to_point(const sensor_msgs::JointState& js, trajectory_msgs::JointTrajectoryPoint& p);

	void cb_joint(const sensor_msgs::JointState& msg);

	void showMarkers(std::string table_frame, const std::vector<geometry_msgs::Pose>& wp1, const std::vector<geometry_msgs::Pose>& wp2);

	bool testWaypointsPosition(const std::vector<geometry_msgs::Point> waypoints_1, const std::vector<geometry_msgs::Point> waypoints_2);

	void getLowerPositions(std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, ScrollGarmentSinglePathConfig& singlePathConfig);

	void getUpperPositions(std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, ScrollGarmentSinglePathConfig& singlePathConfig);

	void getCombinePositions(std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, ScrollGarmentSinglePathConfig& singlePathConfig);

	void getClearCentralPositions(std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, ScrollGarmentSinglePathConfig& singlePathConfig);

	void getPositions(std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, const std::vector< geometry_msgs::Point >& waypoints_1, const	std::vector< geometry_msgs::Point >& waypoints_2, double yawR1, double yawR2, const std::vector<double> height, bool startStop);
	
	double testPathConfig(moveit_msgs::RobotTrajectory &trajectories, const std::vector<std::string> elinks1, const std::vector<std::string> elinks2, const std::vector<geometry_msgs::Pose>& wp1, const std::vector<geometry_msgs::Pose>& wp2, const bool first_combination, double step, bool current_state, const double percent_of_path, bool& isBiggest);

	bool findBiggestTrajectory(ScrollGarmentSinglePathConfig& currentPathConfig, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2);

	bool startStopPosition(ScrollGarmentSinglePathConfig& singlePathConfig, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, bool start);

	int pressOnTheTable(ScrollGarmentSinglePathConfig& singlePathConfig, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, double force);

	bool scrollOverTable(ScrollGarmentSinglePathConfig& singlePathConfig, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, double force);

	double getLenght(const std::vector<geometry_msgs::Point>& wp, int i, bool fromCur2End);

	double getLenght(const std::vector<geometry_msgs::Point>& wp);

	void getNamesConfiguration(bool comb, std::string& tip_1, std::string& tip_2, std::string& group_1, std::string& group_2);

	void emerStopOn(bool on);

public:
	std::string table_frame_;
	clopema_robot::ClopemaRobotCommander crc_;

private:
	
	double sub_traj_;

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
	ros::Publisher pub_marker_;

	boost::mutex mutex_z_r1_;
	double z_r1_;
	boost::mutex mutex_z_r2_;
	double z_r2_;

	boost::mutex mutex_cur_yawR1_;
	double cur_yawR1_;
	boost::mutex mutex_cur_yawR2_;
	double cur_yawR2_;

	boost::mutex mutex_ready_g_;
	bool ready_g_;

	boost::mutex mutex_ext_axis_yaw_;
	double ext_axis_yaw_;
};


#endif