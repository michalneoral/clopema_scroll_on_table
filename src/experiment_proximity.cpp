#include <ros/ros.h>
#include <clopema_motoros/IsRobotReady.h>
#include "ForceCallbackStarter.h"
#include "ScrollGarment.h"

#define MAX_FORCE_EXP_PROXIMITY 30
#define MIN_HEIGHT 0.775
#define MAX_HEIGHT 1.2
#define R1_X (-0.9)
#define R2_X (-0.9)
#define R1_Y 0.3
#define R2_Y (-0.3)
#define R1_yaw 4.1
#define R2_yaw (2*M_PI - 4.1)
#define PITCH (M_PI/2+M_PI/6)

bool open_gripper_ = true;

/** \brief Return true if robot is moving -- read the state from the joint trajectory action */
bool is_robot_moving() {
	clopema_motoros::IsRobotReady srv;
	if(!ros::service::call("/joint_trajectory_action/is_robot_ready", srv)) {
		ROS_ERROR_STREAM("cannot call service is robot ready");
		return true;
	}

	bool result = !srv.response.res;
	return result;
}

void getListOfCollisions(std::vector<std::string>& elinks1,std::vector<std::string>& elinks2, std::string table_frame){
	//-------------------------------------------------------------------------------------------
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
	// elinks1.push_back("r1_ee");
	// elinks2.push_back("r2_ee");
	// elinks1.push_back("r1_ee_par");
	// elinks2.push_back("r2_ee_par");
	// elinks1.push_back("r1_gripper");
	// elinks2.push_back("r2_gripper");
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

bool doExperiment(ForceCallbackStarter& sg, bool down, clopema_robot::ClopemaRobotCommander& crc){
	
	crc.setRobotSpeed(0.02);

	// SETTING POSITION	
	crc.setPoseReferenceFrame("base_link");
	std::vector<geometry_msgs::Pose> wp1, wp2;
	geometry_msgs::Pose blank, p1, p2;

	
	blank.position.x = -0.9;
	blank.position.y = 0.3;
	blank.position.z = MIN_HEIGHT;
	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, (M_PI/2+M_PI/6), 4.1);
	wp1.push_back(blank);
	blank.position.z = 1.2;
	p1 = blank;
	std::cout << p1 << std::endl;

	blank.position.x = -0.9;
	blank.position.y = -0.3;
	blank.position.z = MIN_HEIGHT;
	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, (M_PI/2+M_PI/6), 2*M_PI - 4.1);
	wp2.push_back(blank);
	blank.position.z = 1.2;	
	p2 = blank;	

	if(down){
		goToPosition(crc);
	} else {
		wp1.clear();
		wp2.clear();
		wp1.push_back(p1);
		wp2.push_back(p2);
	}


	std::vector<std::string> elinks1,elinks2;
	getListOfCollisions(elinks1, elinks2, "t3_desk");

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
		clopema_robot::ClopemaRobotCommander::Plan plan;
		plan.trajectory_ = traj;
		
		if(!crc.asyncExecute(plan)){
			std::cout << "Cannot execute traj" << std::endl;
			return false;
		}
		while(is_robot_moving()) {
			ros::Duration(0.0005).sleep();
			std::cout << "robot is moving... R1: " << sg.WrenchR1_.isForceOk(30) << " R2: " << sg.WrenchR2_.isForceOk(30) << " forceR1: " << sg.WrenchR1_.getForce() << " forceR2" << sg.WrenchR2_.getForce() <<  std::endl;
			if( sg.WrenchR1_.getForce() > 30 || sg.WrenchR2_.getForce() > 30 ) {
				crc.stop();
				return true;
			}			
		}
	}
	return true;
}



bool doExperimentPiecewice(ForceCallbackStarter& sg, clopema_robot::ClopemaRobotCommander& crc){
	
	crc.setRobotSpeed(0.02);
	robot_state::RobotState rs(*crc.getCurrentState());

	if(open_gripper_){
		crc.setGripperState(clopema_robot::ACTION_R1_GRIPPER, clopema_robot::GRIPPER_OPEN, true);
		crc.setGripperState(clopema_robot::ACTION_R2_GRIPPER, clopema_robot::GRIPPER_OPEN, true);
	} else {
		crc.setGripperState(clopema_robot::ACTION_R1_GRIPPER, clopema_robot::GRIPPER_CLOSE, true);
		crc.setGripperState(clopema_robot::ACTION_R2_GRIPPER, clopema_robot::GRIPPER_CLOSE, true);	
	}

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
	heights.push_back(0.78);
	heights.push_back(0.77);
	heights.push_back(0.76);
	heights.push_back(0.75);
	heights.push_back(0.74);
	heights.push_back(0.73);


	p1.position.x = -0.9;
	p1.position.y = 0.3;
	p1.position.z = 1.2;
	p1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, (M_PI/2+M_PI/6), 4.1);
	
	p2.position.x = -0.9;
	p2.position.y = -0.3;
	p2.position.z = 1.2;


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
		getListOfCollisions(elinks1, elinks2, "t3_desk");

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
		// if(!crc.execute_traj(traj.joint_trajectory)){
		// 	std::cout << "Cannot execute traj" << std::endl;
		// 	return false;
		// }

		clopema_robot::ClopemaRobotCommander::Plan plan;
		plan.trajectory_ = traj;
		
		if(!crc.asyncExecute(plan)){
			std::cout << "Cannot execute traj" << std::endl;
			return false;
		}
		while(is_robot_moving()) {
			ros::Duration(0.0005).sleep();
			std::cout << "robot is moving... R1: " << sg.WrenchR1_.isForceOk(MAX_FORCE_EXP_PROXIMITY) << " R2: " << sg.WrenchR2_.isForceOk(MAX_FORCE_EXP_PROXIMITY) << " forceR1: " << sg.WrenchR1_.getForce() << " forceR2" << sg.WrenchR2_.getForce() <<  std::endl;
			if( sg.WrenchR1_.getForce() > MAX_FORCE_EXP_PROXIMITY || sg.WrenchR2_.getForce() > MAX_FORCE_EXP_PROXIMITY ) {
				crc.stop();
				return true;
			}			
		}
	}
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "experiment_proximity");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	if (argc > 1){
		open_gripper_ = std::stoi(argv[1]);
	}

	clopema_robot::ClopemaRobotCommander ext("ext");
	ext.setNamedTarget("ext_minus_90");
	ext.move();
	ros::Duration(0.1).sleep();

	clopema_robot::ClopemaRobotCommander crc("arms");
	
	ForceCallbackStarter sg;
	// ScrollGarment sg;
	ros::Duration(0.1).sleep();

	// sg.emerStopOn(true);
	if(!doExperiment(sg, true, crc)){
		return false;
	}

	std::cout << "ITS DOWN" << std::endl;

	// sg.emerStopOn(false);
	ros::Duration(5).sleep();

	if(!doExperiment(sg, false, crc)){
		return false;
	}

	// sg.emerStopOn(true);
	ros::Duration(5).sleep();

	if(!doExperimentPiecewice(sg, crc)){
		return false;
	}

	// sg.emerStopOn(false);
	ros::Duration(5).sleep();

	if(!doExperiment(sg, false, crc)){
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