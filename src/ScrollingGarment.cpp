#include "ScrollingGarment.h"

ScrollingGarment::ScrollingGarment(){
// ScrollingGarment::ScrollingGarment(): crc_("arms"){

}


// void ScrollingGarment::getListOfCollisions(std::vector<std::string>& elinks1,std::vector<std::string>& elinks2, std::string table_frame){
// 	//-------------------------------------------------------------------------------------------
// 	elinks1.push_back("r1_gripper");
// 	elinks1.push_back("r1_ee");
// 	elinks1.push_back("r1_ee_par");
// 	elinks1.push_back("r1_gg");
// 	elinks1.push_back("r2_ps");
// 	elinks1.push_back("r2_gripper");
// 	elinks1.push_back("r2_ee");
// 	elinks1.push_back("r2_ee_par");
// 	elinks1.push_back("r2_gg");
// 	for(int i=0; i<elinks1.size(); i++){
// 		elinks2.push_back(table_frame);
// 	}
// }

// void ScrollingGarment::getPositions(std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, std::vector< geometry_msgs::Point >& waypoints_1,
// 	std::vector< geometry_msgs::Point >& waypoints_2, double yawR1, double yawR2, double offset) { //-------------------------------------------------------------------------------------------

// 	geometry_msgs::Pose blank;
// 	int i = 0;

// 	wp1.erase(wp1.begin(),wp1.end());
// 	wp2.erase(wp2.begin(),wp2.end());

// 	if (waypoints_1.size() != waypoints_2.size()){
// 		ROS_INFO_STREAM("Waypoints have different number of points.");
// 	}

// 	blank.position = waypoints_1[0];
// 	blank.position.z = START_STOP_HEIGHT;
// 	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
// 	wp1.push_back(blank);
// 	blank.position = waypoints_2[0];
// 	blank.position.z = START_STOP_HEIGHT;
// 	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
// 	wp2.push_back(blank);

// 	for (i = 0; i < waypoints_1.size(); i++){
// 		blank.position = waypoints_1[i];
// 		blank.position.z = HEIGHT;
// 		blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
// 		wp1.push_back(blank);

// 		blank.position = waypoints_2[i];
// 		blank.position.z = HEIGHT;
// 		blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
// 		wp2.push_back(blank);

// 		ROS_INFO_STREAM(blank);
// 	}

// 	blank.position = waypoints_1[i-1];
// 	blank.position.z = START_STOP_HEIGHT;
// 	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
// 	wp1.push_back(blank);
// 	blank.position = waypoints_2[i-1];
// 	blank.position.z = START_STOP_HEIGHT;
// 	blank.orientation = tf::createQuaternionMsgFromRollPitchYaw(ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
// 	wp2.push_back(blank);
// }

// void ScrollingGarment::transformFromFrameToTable(std::string frame_id, std::string table_frame, 
// 	std::vector<geometry_msgs::Point >& waypoints_1,
// 	std::vector< geometry_msgs::Point >& waypoints_2){
// 	//-----------------------------------------------------------------------------------------
// 	robot_state::RobotState rs(*crc->getCurrentState());
// 	Eigen::Affine3d e, e_table, e_target, e_tmp;

// 	e_table = rs.getFrameTransform(table_frame);
// 	e_target = rs.getFrameTransform(frame_id);

// 	geometry_msgs::Pose pose_tmp;

// 	for (int i = 0 ; i < waypoints_1.size(); i++){
// 		pose_tmp.position = waypoints_1[i];
// 		tf::poseMsgToEigen(pose_tmp, e);
// 		e_tmp = e_table.inverse() * e_target * e;
// 		waypoints_1[i].x = e_tmp(0,3);
// 		waypoints_1[i].y = e_tmp(1,3);
// 		waypoints_1[i].z = e_tmp(2,3);
// 	}

// 	for (int i = 0 ; i < waypoints_2.size(); i++){
// 		pose_tmp.position = waypoints_2[i];
// 		tf::poseMsgToEigen(pose_tmp, e);
// 		e_tmp = e_table.inverse() * e_target * e;
// 		waypoints_2[i].x = e_tmp(0,3);
// 		waypoints_2[i].y = e_tmp(1,3);
// 		waypoints_2[i].z = e_tmp(2,3);
// 	}

// }

// void ScrollingGarment::moveOverTable(	std::string frame_id,	std::vector< geometry_msgs::Point >& waypoints_1,
// 	std::vector< geometry_msgs::Point >& waypoints_2,	std::string table_frame,	double force) { 
// 	//-------------------------------------------------------------------------------------------

// 	crc->setPoseReferenceFrame(table_frame);

// 	std::vector<std::string> elinks1, elinks2;
// 	getListOfCollisions(elinks1, elinks2, table_frame);
// 	moveit_msgs::RobotTrajectory final_trajectory;
// 	std::vector<geometry_msgs::Pose> wp1, wp2;
// 	double yawR1, yawR2, offset = 0;


// 	transformFromFrameToTable(frame_id, table_frame, waypoints_1, waypoints_2);


// 	for (int i=0; i< ANGLE_NUMBER_STEP; i++) {
// 		for (int j=0; j< ANGLE_NUMBER_STEP; j++){
			
// 			getPositions(wp1, wp2, waypoints_1, waypoints_2, i*(2*M_PI/ANGLE_NUMBER_STEP), j*(2*M_PI/ANGLE_NUMBER_STEP), offset);
			
// 			moveit_msgs::RobotTrajectory trajectories;
// 			if(!planPoses(trajectories, elinks1, elinks2, wp1, wp2, true)){
// 				if(planPoses(trajectories, elinks1, elinks2, wp1, wp2, false)){
// 					final_trajectory = trajectories;
// 					yawR1 = i*(2*M_PI/ANGLE_NUMBER_STEP);
// 					yawR2 = j*(2*M_PI/ANGLE_NUMBER_STEP);
// 				}
// 			} else{
// 				final_trajectory = trajectories;
// 				yawR1 = i*(2*M_PI/ANGLE_NUMBER_STEP);
// 				yawR2 = j*(2*M_PI/ANGLE_NUMBER_STEP);
// 			}
// 		}
// 	}

// 	if (final_trajectory.joint_trajectory.points.size() > 0){
// 		if(crc->execute_traj(final_trajectory.joint_trajectory)) {
// 			ROS_INFO_STREAM(final_trajectory.joint_trajectory.points.size()); 
// 			ROS_INFO_STREAM("Execution is OK");
// 		}else{
// 			ROS_WARN_STREAM("Execution ERROR");
// 		}
// 	}else{
// 		ROS_WARN_STREAM("Execution ERROR, trajectory has no points");
// 	}


// 	//TODO

// }

// bool ScrollingGarment::planPoses(moveit_msgs::RobotTrajectory &trajectories, std::vector<std::string> elinks1, std::vector<std::string> elinks2, std::vector<geometry_msgs::Pose> wp1, std::vector<geometry_msgs::Pose> wp2, bool first_combination){

// 	char buffer [100];


// 	std::string tip_1 = "r1_ee", tip_2 = "r2_ee", conf = "Configuration 1: ";
// 	if(!first_combination) {
// 		tip_1 = "r2_ee", tip_2 = "r1_ee", conf = "Configuration 2: ";
// 	}

// 	double d;
// 	ROS_INFO_STREAM(conf);
// 	d = crc->computeCartesianPathDual(wp1, tip_1, wp2, tip_2, STEP, JUMP_TRESHOLD, trajectories, false);
// 	if(!(fabs(d - 1.0) < 0.001)) {
// 		sprintf(buffer, "cannot interpolate up because fraction = %.4f", d);
// 		ROS_WARN_STREAM(buffer);
// 		return false;
// 	} else{
// 		if(!crc->check_trajectory(trajectories, elinks1, elinks2)) {
// 			sprintf(buffer, "cannot interpolate up because of collision");
// 			ROS_WARN_STREAM(buffer);
// 			return false;
// 		}
// 		else{
// 			ROS_INFO_STREAM(trajectories.joint_trajectory.points.size());
// 			return true;
// 		}
// 	}
// 	return false;
// }
