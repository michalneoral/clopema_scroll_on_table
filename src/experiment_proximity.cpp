#include <ros/ros.h>
#include "ScrollGarment.h"

bool goDown(ScrollGarment& sg){
	
	clopema_robot::ClopemaRobotCommander crc("arms");
	robot_state::RobotState rs(*crc.getCurrentState());

	std::vector<geometry_msgs::Pose> wp1, wp2;
	geometry_msgs::Pose blank, p1, p2;

	crc.setPoseReferenceFrame("base_link");

	blank.position.x = -0.9;
	blank.position.y = 0.3;
	blank.position.z = 0.77;
	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, (M_PI/2+M_PI/6), 4.1);
	wp1.push_back(blank);
	blank.position.z = 1.2;
	p1 = blank;
	if (!rs.setFromIK(rs.getJointModelGroup("r1_arm"), p1, "r1_ee")) {
		ROS_WARN_STREAM("Cannot set from IK - first arm");
		return false;
	}

	blank.position.x = -0.9;
	blank.position.y = -0.3;
	blank.position.z = 0.77;
	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, (M_PI/2+M_PI/6), 2*M_PI - 4.1);
	wp2.push_back(blank);
	blank.position.z = 1.2;	
	p2 = blank;	
	if (!rs.setFromIK(rs.getJointModelGroup("r2_arm"), p2, "r2_ee")) {
		ROS_WARN_STREAM("Cannot set from IK - second arm");
		return false;
	}


	crc.setRobotSpeed(0.02);

	crc.setJointValueTarget(rs);
	if(!crc.move()){
		return false;
	}

	std::vector<std::string> elinks1,elinks2;
	sg.getListOfCollisions(elinks1, elinks2, sg.table_frame_);

	std::cout << p1 << std::endl << "-----" << std::endl << p2 << std::endl << std::endl;

	double d;
	moveit_msgs::RobotTrajectory traj;
	d = crc.computeCartesianPathDual(wp1, "r1_ee", wp2, "r2_ee", 0.001, 1.5, traj, false);
	// d = crc.computeCartesianPathDual(wp1, "r1_ee", wp1, "r1_ee", 0.001, 1.5, traj, false);


	std::cout << wp1.front() << std::endl << "-----" << std::endl << wp2.front() << std::endl << std::endl;

	if(!(fabs(d - 1.0) < 0.001)) {
		ROS_WARN_STREAM("cannot interpolate up.");
		return false;
	} else{
		if(!crc.check_trajectory(traj, elinks1, elinks2)) {
			ROS_WARN_STREAM("cannot interpolate up because of collision");
			return false;
		}
	}
	return crc.execute_traj(traj.joint_trajectory);
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "experiment_proximity");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	double force = 40;
	
	clopema_robot::ClopemaRobotCommander ext("ext");
	ext.setNamedTarget("ext_minus_90");
	ext.move();
	ros::Duration(0.1).sleep();

	clopema_robot::ClopemaRobotCommander crc("arms");
	crc.setGripperState(clopema_robot::ACTION_R1_GRIPPER, clopema_robot::GRIPPER_OPEN);
	crc.setGripperState(clopema_robot::ACTION_R2_GRIPPER, clopema_robot::GRIPPER_OPEN, true);
	crc.move();

	std::string table_frame = "t3_desk";
	ScrollGarment sg;
	sg.table_frame_ = table_frame;
	goDown(sg);

	ROS_INFO_STREAM("GOOD BYE");
	ros::spinOnce();
	return 0;
}