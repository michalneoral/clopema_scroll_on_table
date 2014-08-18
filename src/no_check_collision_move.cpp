#include <ros/ros.h>
#include <clopema_robot/robot_commander.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <boost/math/special_functions/fpclassify.hpp>

#include <clopema_robot/GraspAndFold.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>


#define HEIGHT 0.78
#define START_STOP_HEIGHT 0.83 

#define ROLL2DESK_R1 (M_PI)
#define PITCH2DESK_R1 (M_PI/2-M_PI/6)
#define YAW2DESK_R1 (-M_PI) //(M_PI/2-M_PI/4)

#define ROLL2DESK_R2 (M_PI)
#define PITCH2DESK_R2 (M_PI/2-M_PI/6)
#define YAW2DESK_R2 (-M_PI) //(-M_PI/2+M_PI/4)

#define ANGLE_NUMBER_STEP 8

#define STEP 0.01
#define JUMP_TRESHOLD 0

boost::shared_ptr<clopema_robot::ClopemaRobotCommander> crc;



bool planPoses(moveit_msgs::RobotTrajectory& trajectories, std::vector<std::string> elinks1, std::vector<std::string> elinks2, std::vector<geometry_msgs::Pose> wp1, std::vector<geometry_msgs::Pose> wp2, bool first_combination);
void moveOverTable(	std::string frame_id,	std::vector< geometry_msgs::Point >& waypoints_1,	std::vector< geometry_msgs::Point >& waypoints_2,	std::string table_frame,	double force);

void getListOfPoints(std::vector< geometry_msgs::Point>& waypoints_1,std::vector< geometry_msgs::Point>& waypoints_2){
	geometry_msgs::Point blank;

	blank.x = -0.9;
	blank.y = 0.4;
	waypoints_1.push_back(blank);
	blank.y = 0.3;
	waypoints_1.push_back(blank);
	blank.x = -1.5;
	blank.y = 0.4;
	waypoints_1.push_back(blank);
	blank.x = -0.9;
	blank.y = 0.5;
	waypoints_1.push_back(blank);

	blank.x = -0.9;
	blank.y = -0.4;
	waypoints_2.push_back(blank);
	blank.y = -0.3;
	waypoints_2.push_back(blank);
	blank.x = -1.5;
	blank.y = -0.4;
	waypoints_2.push_back(blank);
	blank.x = -0.9;
	blank.y = -0.3;
	waypoints_2.push_back(blank);
}

void getListOfCollisions(std::vector<std::string>& elinks1,std::vector<std::string>& elinks2, std::string table_frame){
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

void getPositions(std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, std::vector< geometry_msgs::Point >& waypoints_1,
	std::vector< geometry_msgs::Point >& waypoints_2, double yawR1, double yawR2){
	geometry_msgs::Pose blank;

	wp1.erase(wp1.begin(),wp1.end());
	wp2.erase(wp2.begin(),wp2.end());

	if (waypoints_1.size() != waypoints_2.size()){
		ROS_INFO_STREAM("Waypoints have different number of points.");
	}

	for (int i = 0; i < waypoints_1.size(); i++){
		blank.position = waypoints_1[i];
		blank.position.z = HEIGHT;
		blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
		wp1.push_back(blank);

		blank.position = waypoints_2[i];
		blank.position.z = HEIGHT;
		blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
		wp2.push_back(blank);
	}
}

void moveOverTable(	std::string frame_id,	std::vector< geometry_msgs::Point >& waypoints_1,
	std::vector< geometry_msgs::Point >& waypoints_2,	std::string table_frame,	double force) 
{ //-------------------------------------------------------------------------------------------
	std::vector<std::string> elinks1, elinks2;
	getListOfCollisions(elinks1, elinks2, table_frame);
	moveit_msgs::RobotTrajectory final_trajectory;
	std::vector<geometry_msgs::Pose> wp1, wp2;


	for (int i=0; i< ANGLE_NUMBER_STEP; i++) {
		for (int j=0; j< ANGLE_NUMBER_STEP; j++){
			getPositions(wp1, wp2, waypoints_1, waypoints_2, i*(2*M_PI/ANGLE_NUMBER_STEP), j*(2*M_PI/ANGLE_NUMBER_STEP));
			moveit_msgs::RobotTrajectory trajectories;
			if(!planPoses(trajectories, elinks1, elinks2, wp1, wp2, true)){
				if(planPoses(trajectories, elinks1, elinks2, wp1, wp2, false)){
					final_trajectory = trajectories;
				}
			} else{
				final_trajectory = trajectories;
			}
		}
	}

	if (final_trajectory.joint_trajectory.points.size() > 0){
		if(crc->execute_traj(final_trajectory.joint_trajectory)) {
			ROS_INFO_STREAM(final_trajectory.joint_trajectory.points.size()); 
			ROS_INFO_STREAM("Execution is OK");
		}else{
			ROS_WARN_STREAM("Execution ERROR");
		}
	}else{
		ROS_WARN_STREAM("Execution ERROR, trajectory has no points");
	}


	//TODO

}

bool planPoses(moveit_msgs::RobotTrajectory &trajectories, std::vector<std::string> elinks1, std::vector<std::string> elinks2, std::vector<geometry_msgs::Pose> wp1, std::vector<geometry_msgs::Pose> wp2, bool first_combination){

	char buffer [100];


	std::string tip_1 = "r1_ee", tip_2 = "r2_ee", conf = "Configuration 1: ";
	if(!first_combination) {
		tip_1 = "r2_ee", tip_2 = "r1_ee", conf = "Configuration 2: ";
	}

	double d;
	ROS_INFO_STREAM(conf);
	d = crc->computeCartesianPathDual(wp1, tip_1, wp2, tip_2, STEP, JUMP_TRESHOLD, trajectories, false);
	if(!(fabs(d - 1.0) < 0.001)) {
		sprintf(buffer, "cannot interpolate up because fraction = %.4f", d);
		ROS_WARN_STREAM(buffer);
		return false;
	} else{
		if(!crc->check_trajectory(trajectories, elinks1, elinks2)) {
			sprintf(buffer, "cannot interpolate up because of collision");
			ROS_WARN_STREAM(buffer);
			return false;
		}
		else{
			ROS_INFO_STREAM(trajectories.joint_trajectory.points.size());
			return true;
		}
	}
	return false;
}



int main(int argc, char **argv) {
	ros::init(argc, argv, "no_check_collision_move");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	crc.reset(new clopema_robot::ClopemaRobotCommander("arms"));

	std::string frame_id = "ctu_floor";
	std::string table_frame = "t3_desk";

	crc->setPoseReferenceFrame("ctu_floor");
	robot_state::RobotState rs(*crc->getCurrentState());

	std::vector< geometry_msgs::Point > waypoints_1, waypoints_2;
	getListOfPoints(waypoints_1, waypoints_2);

	moveOverTable(frame_id,	waypoints_1, waypoints_2, table_frame, 10);

	ROS_INFO_STREAM("GOOD BYE");
	ros::spinOnce();
	return 0;
}