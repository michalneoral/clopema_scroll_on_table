#include <ros/ros.h>
#include "ScrollGarment.h"

#define MIN_HEIGHT 0.775

bool doExperiment(ScrollGarment& sg, bool down){
	
	clopema_robot::ClopemaRobotCommander crc("arms");
	crc.setRobotSpeed(0.02);
	robot_state::RobotState rs(*crc.getCurrentState());

	std::vector<geometry_msgs::Pose> wp1, wp2;
	geometry_msgs::Pose blank, p1, p2;

	crc.setPoseReferenceFrame("base_link");

	blank.position.x = -0.9;
	blank.position.y = 0.3;
	blank.position.z = MIN_HEIGHT;
	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, (M_PI/2+M_PI/6), 4.1);
	wp1.push_back(blank);
	blank.position.z = 1.2;
	p1 = blank;

	blank.position.x = -0.9;
	blank.position.y = -0.3;
	blank.position.z = MIN_HEIGHT;
	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, (M_PI/2+M_PI/6), 2*M_PI - 4.1);
	wp2.push_back(blank);
	blank.position.z = 1.2;	
	p2 = blank;	

	if(down){
		if (!rs.setFromIK(rs.getJointModelGroup("r1_arm"), p1, "r1_ee")) {
			ROS_WARN_STREAM("Cannot set from IK - first arm");
			return false;
		}
		if (!rs.setFromIK(rs.getJointModelGroup("r2_arm"), p2, "r2_ee")) {
			ROS_WARN_STREAM("Cannot set from IK - second arm");
			return false;
		}

		crc.setJointValueTarget(rs);
		if(!crc.move()){
			return false;
		}
	} else {
		wp1.clear();
		wp2.clear();
		wp1.push_back(p1);
		wp2.push_back(p2);
	}

	std::vector<std::string> elinks1,elinks2;
	sg.getListOfCollisions(elinks1, elinks2, sg.table_frame_);

	double d;
	moveit_msgs::RobotTrajectory traj;
	d = crc.computeCartesianPathDual(wp1, "r1_ee", wp2, "r2_ee", 0.001, 1.5, traj, false);

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

bool doExperimentPiecewice(ScrollGarment& sg){
	
	clopema_robot::ClopemaRobotCommander crc("arms");
	crc.setRobotSpeed(0.02);
	robot_state::RobotState rs(*crc.getCurrentState());

	std::vector<geometry_msgs::Pose> wp1, wp2;
	geometry_msgs::Pose blank, p1, p2;

	crc.setPoseReferenceFrame("base_link");

	std::vector<double> heights;
	heights.push_back(1.1);
	heights.push_back(1.05);
	heights.push_back(1.0);
	heights.push_back(0.95);
	heights.push_back(0.90);
	heights.push_back(0.85);
	heights.push_back(0.80);
	heights.push_back(0.79);
	heights.push_back(0.785);


	blank.position.x = -0.9;
	blank.position.y = 0.3;
	blank.position.z = 1.2;
	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, (M_PI/2+M_PI/6), 4.1);
	p1 = blank;

	blank.position.x = -0.9;
	blank.position.y = -0.3;
	blank.position.z = 1.2;
	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, (M_PI/2+M_PI/6), 2*M_PI - 4.1);
	p2 = blank;

	if (!rs.setFromIK(rs.getJointModelGroup("r1_arm"), p1, "r1_ee")) {
		ROS_WARN_STREAM("Cannot set from IK - first arm");
		return false;
	}
	if (!rs.setFromIK(rs.getJointModelGroup("r2_arm"), p2, "r2_ee")) {
		ROS_WARN_STREAM("Cannot set from IK - second arm");
		return false;
	}
	crc.setJointValueTarget(rs);
	if(!crc.move()){
		return false;
	}

	for (int i = 0 ; i < heights.size() ; i++)
	{
		wp1.clear();
		wp2.clear();

		crc.setStartState(*crc.getCurrentState());

		blank.position.x = -0.9;
		blank.position.y = 0.3;
		blank.position.z = heights[i];
		blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, (M_PI/2+M_PI/6), 4.1);
		wp1.push_back(blank);

		blank.position.x = -0.9;
		blank.position.y = -0.3;
		blank.position.z = heights[i];
		blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, (M_PI/2+M_PI/6), 2*M_PI - 4.1);
		wp2.push_back(blank);

		std::vector<std::string> elinks1,elinks2;
		sg.getListOfCollisions(elinks1, elinks2, sg.table_frame_);

		double d;
		moveit_msgs::RobotTrajectory traj;
		d = crc.computeCartesianPathDual(wp1, "r1_ee", wp2, "r2_ee", 0.001, 1.5, traj, false);

		traj.joint_trajectory.points.erase(traj.joint_trajectory.points.begin());
		sg.add_current_state_to_trajectory(traj.joint_trajectory);


		if(!(fabs(d - 1.0) < 0.001)) {
			ROS_WARN_STREAM("cannot interpolate up.");
			return false;
		} else{
			if(!crc.check_trajectory(traj, elinks1, elinks2)) {
				ROS_WARN_STREAM("cannot interpolate up because of collision");
				return false;
			}
		}
		for (int j=0; j<traj.joint_trajectory.points.size(); j++){
		}
		if(!crc.execute_traj(traj.joint_trajectory)){
			std::cout << "Cannot execute traj" << std::endl;
			return false;
		}
		ros::Duration(5.0).sleep();
	}
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "experiment_proximity");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	clopema_robot::ClopemaRobotCommander ext("ext");
	ext.setNamedTarget("ext_minus_90");
	ext.move();
	ros::Duration(0.1).sleep();

	clopema_robot::ClopemaRobotCommander crc("arms");
	crc.setGripperState(clopema_robot::ACTION_R1_GRIPPER, clopema_robot::GRIPPER_OPEN, true);
	crc.setGripperState(clopema_robot::ACTION_R2_GRIPPER, clopema_robot::GRIPPER_CLOSE, true);
	crc.move();
	ros::Duration(0.1).sleep();

	std::string table_frame = "t3_desk";
	ScrollGarment sg;
	sg.table_frame_ = table_frame;

	sg.emerStopOn(true);
	if(!doExperiment(sg, true)){
		return false;
	}

	std::cout << "ITS DOWN" << std::endl;

	sg.emerStopOn(false);
	ros::Duration(5).sleep();

	if(!doExperiment(sg, false)){
		return false;
	}

	sg.emerStopOn(true);
	ros::Duration(5).sleep();

	if(!doExperimentPiecewice(sg)){
		return false;
	}

	sg.emerStopOn(false);
	ros::Duration(5).sleep();

	if(!doExperiment(sg, false)){
		return false;
	}	
	ros::Duration(0.1).sleep();

	crc.setNamedTarget("home_arms");
	crc.move();
	ros::Duration(0.1).sleep();

	crc.setNamedTarget("ext_home");
	crc.move();
	ros::Duration(0.1).sleep();
	
	ROS_INFO_STREAM("GOOD BYE");
	ros::spinOnce();
	return 0;
}