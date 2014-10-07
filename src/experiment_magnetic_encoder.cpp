#include <ros/ros.h>
#include "ScrollGarment.h"

#define MIN_HEIGHT 0.77
#define MAX_HEIGHT 0.85
#define R1_X (-0.9)
#define R2_X (-0.9)
#define R1_Y 0.3
#define R2_Y (-0.3)
#define R1_yaw 4.1
#define R2_yaw (2*M_PI - 4.1)
#define PITCH (M_PI/2+M_PI/6)

bool doExperiment(clopema_robot::ClopemaRobotCommander& crc_, ScrollGarment& sg_, double speed, bool down){
	
	crc_.setRobotSpeed(speed);
	robot_state::RobotState rs(*crc_.getCurrentState());
	std::vector<geometry_msgs::Pose> wp1, wp2;
	geometry_msgs::Pose blank;
	crc_.setPoseReferenceFrame("base_link");

	double height = MAX_HEIGHT;
	if (down){
		height = MIN_HEIGHT;
	}

	blank.position.x = R1_X;
	blank.position.y = R1_Y;
	blank.position.z = height;
	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, PITCH, R1_yaw);
	wp1.push_back(blank);

	blank.position.x = R2_X;
	blank.position.y = R2_Y;
	blank.position.z = height;
	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, PITCH, R2_yaw);
	wp2.push_back(blank);	

	std::vector<std::string> elinks1,elinks2;
	sg_.getListOfCollisions(elinks1, elinks2, sg_.table_frame_);

	double d;
	moveit_msgs::RobotTrajectory traj;
	d = crc_.computeCartesianPathDual(wp1, "r1_ee", wp2, "r2_ee", 0.001, 1.5, traj, false);
	traj.joint_trajectory.points.erase(traj.joint_trajectory.points.begin());
	sg_.add_current_state_to_trajectory(traj.joint_trajectory);

	if(!(fabs(d - 1.0) < 0.001)) {
		ROS_WARN_STREAM("cannot interpolate up.");
		return false;
	} else{
		if(!crc_.check_trajectory(traj, elinks1, elinks2)) {
			ROS_WARN_STREAM("cannot interpolate up because of collision");
			return false;
		}
	}
	return crc_.execute_traj(traj.joint_trajectory);
}

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


int main(int argc, char **argv) {
	ros::init(argc, argv, "experiment_magnetic_encoder");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	clopema_robot::ClopemaRobotCommander ext("ext");
	ext.setNamedTarget("ext_minus_90");
	ext.move();
	ros::Duration(0.1).sleep();

	int open = 1;
	if (argc >= 2) {
		open = std::stoi(argv[1]);
	}

	clopema_robot::ClopemaRobotCommander crc("arms");
	if (open){
		crc.setGripperState(clopema_robot::ACTION_R1_GRIPPER, clopema_robot::GRIPPER_OPEN, true);
		crc.setGripperState(clopema_robot::ACTION_R2_GRIPPER, clopema_robot::GRIPPER_OPEN, true);	
	}
	else{
		crc.setGripperState(clopema_robot::ACTION_R1_GRIPPER, clopema_robot::GRIPPER_CLOSE, true);
		crc.setGripperState(clopema_robot::ACTION_R2_GRIPPER, clopema_robot::GRIPPER_CLOSE, true);	
	}
	crc.move();
	ros::Duration(0.1).sleep();

	std::string table_frame = "t3_desk";
	ScrollGarment sg;
	sg.table_frame_ = table_frame;


	ros::Duration(5).sleep();

	// sg.emerStopOn(false);

	crc.setRobotSpeed(0.2);

	if(!goToPosition(crc)){
		return false;
	}

	for(int i=1; i<=10; i++){
		if(!doExperiment(crc, sg, 0.02*i, true)){
			return false;
		}
		ros::Duration(2.0).sleep();
		if(!doExperiment(crc, sg, 0.02*i, false)){
			return false;
		}
		ros::Duration(2.0).sleep();
	}

	ROS_INFO_STREAM("GOOD BYE");
	ros::spinOnce();
	return 0;
}