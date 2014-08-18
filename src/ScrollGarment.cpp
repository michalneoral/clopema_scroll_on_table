#include "ScrollGarment.h"

ScrollGarment::ScrollGarment(): crc_("arms"){

	WrenchR1_.startSub("r1");
	WrenchR2_.startSub("r2");
	sub_joints_ = node_.subscribe("/joint_states", 1, &ScrollGarment::cb_joint, this);
	
	WrenchR1_.setInitialize();
	WrenchR2_.setInitialize();	

}

void ScrollGarment::getListOfCollisions(std::vector<std::string>& elinks1,std::vector<std::string>& elinks2, std::string table_frame){
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
}

double randomOffset(){
	double min=-0.03;
	double max=0.03;
	return (max-min)*((double)std::rand()/RAND_MAX)+min;
}

void ScrollGarment::getTestPositions(std::string table_frame , std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, const std::vector< geometry_msgs::Point >& waypoints_1, const	std::vector< geometry_msgs::Point >& waypoints_2, double yawR1, double yawR2, double offset) { //-------------------------------------------------------------------------------------------

	geometry_msgs::Pose blank;
	int i = 0;

	wp1.erase(wp1.begin(),wp1.end());
	wp2.erase(wp2.begin(),wp2.end());

	if (waypoints_1.size() != waypoints_2.size()){
		ROS_ERROR("Waypoints have different number of points.");
	}

	blank.position = waypoints_1[0];
	blank.position.z = START_STOP_HEIGHT;
	blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
	wp1.push_back(blank);
	blank.position = waypoints_2[0];
	blank.position.z = START_STOP_HEIGHT;
	blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
	wp2.push_back(blank);

	for (i = 0; i < waypoints_1.size(); i++){
		blank.position = waypoints_1[i];
		blank.position.z = HEIGHT + randomOffset() + offset;
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
		wp1.push_back(blank);
		
		blank.position = waypoints_2[i];
		blank.position.z = HEIGHT + randomOffset() + offset;
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
		wp2.push_back(blank);
	}

	blank.position = waypoints_1[i-1];
	blank.position.z = START_STOP_HEIGHT;
	blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
	wp1.push_back(blank);
	blank.position = waypoints_2[i-1];
	blank.position.z = START_STOP_HEIGHT;
	blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
	wp2.push_back(blank);
}

bool ScrollGarment::testTrajectory(std::string table_frame , double& yawR1, double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, bool& conf){
//-------------------------------------------------------------------------------------------

	std::vector<geometry_msgs::Pose> wp1, wp2;
	double offset = 0;
	int angle_number_step = ANGLE_NUMBER_STEP;

	for (int i=0; i< angle_number_step; i++) {
		for (int j=0; j< angle_number_step; j++){
			
			getTestPositions(table_frame, wp1, wp2, waypoints_1, waypoints_2, i*(2*M_PI/angle_number_step), j*(2*M_PI/angle_number_step), offset);
			
			moveit_msgs::RobotTrajectory trajectories;
			if(!planPoses(trajectories, elinks1, elinks2, wp1, wp2, true, STEP)){
				if(planPoses(trajectories, elinks1, elinks2, wp1, wp2, false, STEP)){
					yawR1 = i*(2*M_PI/angle_number_step);
					yawR2 = j*(2*M_PI/angle_number_step);
					conf = false;					
					return true;
				}
			} else{				
				yawR1 = i*(2*M_PI/angle_number_step);
				yawR2 = j*(2*M_PI/angle_number_step);
				conf = true;
				return true;
			}
		}
	}
	return false;
}

void ScrollGarment::transformFromFrameToTable(std::string frame_id, std::string table_frame, std::vector<geometry_msgs::Point >& waypoints_1, std::vector< geometry_msgs::Point >& waypoints_2){
	//-----------------------------------------------------------------------------------------
	robot_state::RobotState rs(*crc_.getCurrentState());
	Eigen::Affine3d e, e_table, e_target, e_tmp;

	e_table = rs.getFrameTransform(table_frame);
	e_target = rs.getFrameTransform(frame_id);

	geometry_msgs::Pose pose_tmp;

	for (int i = 0 ; i < waypoints_1.size(); i++){
		pose_tmp.position = waypoints_1[i];
		tf::poseMsgToEigen(pose_tmp, e);
		e_tmp = e_table.inverse() * e_target * e;
		waypoints_1[i].x = e_tmp(0,3);
		waypoints_1[i].y = e_tmp(1,3);
		waypoints_1[i].z = e_tmp(2,3);
	}

	for (int i = 0 ; i < waypoints_2.size(); i++){
		pose_tmp.position = waypoints_2[i];
		tf::poseMsgToEigen(pose_tmp, e);
		e_tmp = e_table.inverse() * e_target * e;
		waypoints_2[i].x = e_tmp(0,3);
		waypoints_2[i].y = e_tmp(1,3);
		waypoints_2[i].z = e_tmp(2,3);
	}

}

geometry_msgs::Quaternion ScrollGarment::transformQuaternion(std::string table_frame,double roll, double pitch, double yaw) {
	//--------------------------------------------------------------------
	
	geometry_msgs::Pose pose_tmp;
	pose_tmp.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

	robot_state::RobotState rs(*crc_.getCurrentState());
	Eigen::Affine3d e, e_table, e_target, e_tmp;

	e_table = rs.getFrameTransform(table_frame);
	e_target = rs.getFrameTransform(BASE_TARGET);

	tf::poseMsgToEigen(pose_tmp, e);
	e_tmp = e_table.inverse() * e_target * e;
	
	tf::poseEigenToMsg(e_tmp, pose_tmp);
	return pose_tmp.orientation;
}

// void complete_trajectory(const robot_state::RobotState& start_state, trajectory_msgs::JointTrajectory& trajectory) {
// 	trajectory_msgs::JointTrajectory oldTrajectory = trajectory;

// 	std::vector<std::string> joint_names_;
// 	std::string robotPreffix = "r1_";
// 	joint_names_.push_back(robotPreffix + "joint_s");
// 	joint_names_.push_back(robotPreffix + "joint_l");
// 	joint_names_.push_back(robotPreffix + "joint_u");
// 	joint_names_.push_back(robotPreffix + "joint_r");
// 	joint_names_.push_back(robotPreffix + "joint_b");
// 	joint_names_.push_back(robotPreffix + "joint_t");
// 	robotPreffix = "r2_";
// 	joint_names_.push_back(robotPreffix + "joint_s");
// 	joint_names_.push_back(robotPreffix + "joint_l");
// 	joint_names_.push_back(robotPreffix + "joint_u");
// 	joint_names_.push_back(robotPreffix + "joint_r");
// 	joint_names_.push_back(robotPreffix + "joint_b");
// 	joint_names_.push_back(robotPreffix + "joint_t");
// 	joint_names_.push_back("ext_axis");

// 	trajectory.joint_names.clear();
// 	trajectory.points.clear();

// 	trajectory_msgs::JointTrajectoryPoint p;
// 	BOOST_FOREACH(const std::string & jn, joint_names_) {
// 		trajectory.joint_names.push_back(jn);
// 		p.positions.push_back(start_state.getVariablePosition(jn));
// 		p.velocities.push_back(0.0);
// 		p.accelerations.push_back(0.0);
// 		p.effort.push_back(0.0);
// 	}


// 	BOOST_FOREACH(const trajectory_msgs::JointTrajectoryPoint & point, oldTrajectory.points) {
// 		for(unsigned int i = 0 ; i < oldTrajectory.joint_names.size(); ++i) {
// 			std::string jn = oldTrajectory.joint_names[i];
// 			std::vector<std::string>::iterator it;
// 			it = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), jn);
// 			unsigned int ind = std::distance(trajectory.joint_names.begin(), it);
// 			if (ind != 0){
// 				if(i == oldTrajectory.joint_names.size()-1){
// 					p.positions[ind] = start_state.getVariablePosition(jn);
// 				} else {
// 					p.positions[ind] = point.positions[i];
// 				}
// 			} else {
// 				p.time_from_start = point.time_from_start;
// 				trajectory.points.push_back(p);
// 			}
// 		}

// 		p.time_from_start = point.time_from_start;
// 		trajectory.points.push_back(p);
// 	}
// }



int ScrollGarment::pressOnTheTable(std::string table_frame , const double& yawR1, const double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, bool conf, double force) {	//----------------------------------------------------------------------------

	//return 0 - trajectory fault
	//return 1 - trajectory ok
	//return 2 - force ok, you can continue

	std::vector<geometry_msgs::Pose> wp1, wp2;
	moveit_msgs::RobotTrajectory trajectory;
	geometry_msgs::Pose blank;
	bool isOk = false;	
	// Prepare poses 
	{

		blank.position = waypoints_1[0];		
		if(WrenchR1_.isForceOk(force)){
			isOk = true;
			mutex_z_r1_.lock();
			std::cout << "R1 - OK, old z = " << z_r1_ << " force: " << WrenchR1_.getForce() << std::endl << std::endl;
			mutex_z_r1_.unlock();
		}else{
			isOk = false;
			mutex_z_r1_.lock();
			blank.position.z = z_r1_ + (WrenchR1_.getForce() - force) * FORCE_CONST;
			std::cout << "R1 - NOT OK, old z = " << z_r1_ << " new z = " << blank.position.z << " force: " << WrenchR1_.getForce() << std::endl << std::endl;
			mutex_z_r1_.unlock();				
		}		
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
		wp1.push_back(blank);

		blank.position = waypoints_2[0];		
		if(!WrenchR1_.isForceOk(force)){			
			isOk = false;
			mutex_z_r2_.lock();
			blank.position.z = z_r2_ + (WrenchR2_.getForce() - force) * FORCE_CONST;
			std::cout << "R2 - NOT OK, old z = " << z_r2_ << " new z = " << blank.position.z << " force: " << WrenchR2_.getForce() << std::endl << std::endl;
			mutex_z_r2_.unlock();
		}		
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
		wp2.push_back(blank);
	}

	if(isOk){ 
		return 2;
	} else if (planPoses(trajectory, elinks1, elinks2, wp1, wp2, conf, STEP)){
		trajectory.joint_trajectory.points.erase(trajectory.joint_trajectory.points.begin());
		add_current_state_to_trajectory(trajectory.joint_trajectory);
		if(crc_.execute_traj(trajectory.joint_trajectory)){
			return 1;
		} else {
			return 0;
		}
	}else{
		return 0;
	}
}

bool ScrollGarment::scrollOverTable(std::string table_frame , const double& yawR1, const double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, bool conf, double force) {	//----------------------------------------------------------------------------

	std::vector<geometry_msgs::Pose> wp1, wp2;
	geometry_msgs::Pose blank;
	
	
	for (int i = 1; i < waypoints_1.size(); i++){
		bool next = false;
		while(!next) 
		{
			moveit_msgs::RobotTrajectory trajectory;
			wp1.clear();
			wp2.clear();


			blank.position = waypoints_1[i];
			mutex_z_r1_.lock();
			blank.position.z = z_r1_ + (WrenchR1_.getForce() - force) * FORCE_CONST;
			mutex_z_r1_.unlock();
			blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
			wp1.push_back(blank);

			blank.position = waypoints_2[i];
			mutex_z_r2_.lock();
			blank.position.z = z_r2_ + (WrenchR2_.getForce() - force) * FORCE_CONST;
			mutex_z_r2_.unlock();
			blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
			wp2.push_back(blank);


			if(planPoses(trajectory, elinks1, elinks2, wp1, wp2, conf, STEP)){
				trajectory.joint_trajectory.points.erase(trajectory.joint_trajectory.points.begin());
				if (trajectory.joint_trajectory.points.size() <= 4){
					next = true;
					add_current_state_to_trajectory(trajectory.joint_trajectory);
					if(!crc_.execute_traj(trajectory.joint_trajectory)){
						ROS_ERROR("Cannot move.");
						return 0;
					}
				} else {
					trajectory.joint_trajectory.points.erase(trajectory.joint_trajectory.points.begin()+4,trajectory.joint_trajectory.points.end());
					ROS_INFO_STREAM(trajectory.joint_trajectory.points.size());
					add_current_state_to_trajectory(trajectory.joint_trajectory);
					if(!crc_.execute_traj(trajectory.joint_trajectory)){
						ROS_ERROR("Cannot move.");
						return 0;
					}
				}
			}else{
				ROS_ERROR("Trajectory wasn't found.");
				return 0;
			}			
		}
	}
	return 1;
}


bool ScrollGarment::goToPosition(std::string table_frame , const double& yawR1, const double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, const bool& conf, bool start) {
	//------------------------------------------------------------------------
	std::vector<geometry_msgs::Pose> wp1, wp2;
	moveit_msgs::RobotTrajectory trajectory;
	geometry_msgs::Pose blank;
	// Prepare poses 
	if(start){
		blank.position = waypoints_1[0];
		blank.position.z = START_STOP_HEIGHT;
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
		wp1.push_back(blank);
		blank.position.z = OVER_TABLE_HEIGHT;
		wp1.push_back(blank);

		blank.position = waypoints_2[0];
		blank.position.z = START_STOP_HEIGHT;
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
		
		wp2.push_back(blank);
		blank.position.z = OVER_TABLE_HEIGHT;
		wp2.push_back(blank);
	} else {
		blank.position = waypoints_1.back();
		blank.position.z = START_STOP_HEIGHT;
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
		wp1.push_back(blank);

		blank.position = waypoints_2.back();
		blank.position.z = START_STOP_HEIGHT;
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
		wp2.push_back(blank);
	}

	if(planPoses(trajectory, elinks1, elinks2, wp1, wp2, conf, STEP)){
		trajectory.joint_trajectory.points.erase(trajectory.joint_trajectory.points.begin());
		add_current_state_to_trajectory(trajectory.joint_trajectory);
		pub_traj_.controle(trajectory.joint_trajectory);
		return crc_.execute_traj(trajectory.joint_trajectory);
	}else{
		ROS_ERROR("Cannot interpolate");
		return false;
	}
}

bool ScrollGarment::moveOverTable(	std::string frame_id,	std::vector< geometry_msgs::Point >& waypoints_1,
	std::vector< geometry_msgs::Point >& waypoints_2,	std::string table_frame,	double force) { 
	//-------------------------------------------------------------------------------------------
	table_frame_ = table_frame;
	crc_.setPoseReferenceFrame(table_frame);	

	std::vector<std::string> elinks1, elinks2;
	getListOfCollisions(elinks1, elinks2, table_frame);
	moveit_msgs::RobotTrajectory final_trajectory, splitted_traj;
	trajectory_msgs::JointTrajectory part_trajectory;
	
	double yawR1=0, yawR2=0;
	bool conf;

	transformFromFrameToTable(frame_id, table_frame, waypoints_1, waypoints_2);
// ==================================== ↑↑↑ GOOD ↑↑↑ ====================================

	if(testTrajectory(table_frame, yawR1, yawR2, waypoints_1, waypoints_2, elinks1, elinks2, conf)){

		crc_.execute_traj(crc_.gripper_trajectory(clopema_robot::GRIPPER_OPEN));
		

		if( !goToPosition(table_frame, yawR1, yawR2, waypoints_1, waypoints_2, elinks1, elinks2, conf, true) ){
			ROS_ERROR("Cannot execution move to the start position.");
			crc_.setServoPowerOff();
			return false;
		}

		ros::Duration(SLEEP_AFTER_EXEC).sleep();

		WrenchR1_.setInitialize();
		WrenchR2_.setInitialize();



		while(true){
			int statueOfPress = pressOnTheTable(table_frame, yawR1, yawR2, waypoints_1, waypoints_2, elinks1, elinks2, conf, force);
			if( statueOfPress == 0){
				ROS_ERROR("Cannot press on the table.");
				crc_.setServoPowerOff();
				return false;
			} else if (statueOfPress == 2){
				break;
			}
		}
		
		ros::Duration(SLEEP_AFTER_EXEC).sleep();

		if(!scrollOverTable(table_frame, yawR1, yawR2, waypoints_1, waypoints_2, elinks1, elinks2, conf, force)){
			ROS_ERROR("Cannot scroll.");
			crc_.setServoPowerOff();
			return false;
		}

		if( !goToPosition(table_frame, yawR1, yawR2, waypoints_1, waypoints_2, elinks1, elinks2, conf, false) ){
			ROS_ERROR("Cannot execution move to the end position.");
			crc_.setServoPowerOff();
			return false;
		}


	}else{
		ROS_ERROR("Trajectory wasn't found.");
		crc_.setServoPowerOff();
		return false;
	}
	crc_.setServoPowerOff();
	return true;


}

bool ScrollGarment::planPoses(moveit_msgs::RobotTrajectory &trajectories, const std::vector<std::string> elinks1, const std::vector<std::string> elinks2, const std::vector<geometry_msgs::Pose>& wp1, const std::vector<geometry_msgs::Pose>& wp2, const bool first_combination, double step){
	//------------------------------------------------------------------------

	char buffer [100];


	std::string tip_1 = "r1_ee", tip_2 = "r2_ee", conf = "Configuration 1: ";
	if(!first_combination) {
		tip_1 = "r2_ee", tip_2 = "r1_ee", conf = "Configuration 2: ";
	}

	double d;
	
	robot_state::RobotState rs(*crc_.getCurrentState());
	crc_.setStartState(rs);

	d = crc_.computeCartesianPathDual(wp1, tip_1, wp2, tip_2, step, JUMP_TRESHOLD, trajectories, false);


	if(!(fabs(d - 1.0) < 0.001)) {
		sprintf(buffer, "cannot interpolate up because fraction = %.4f", d);
		ROS_WARN_STREAM(buffer);
		return false;
	} else{
		if(!crc_.check_trajectory(trajectories, elinks1, elinks2)) {
			sprintf(buffer, "cannot interpolate up because of collision");
			ROS_WARN_STREAM(buffer);
			return false;
		}
		else{
			std::cout << "Trajectory points: " << trajectories.joint_trajectory.points.size() << std::endl << std::endl;
			return true;
		}
	}
	return false;
}

bool ScrollGarment::joints_to_point(const sensor_msgs::JointState& js, trajectory_msgs::JointTrajectoryPoint& p)  {	
	std::vector<std::string> jns = get_joint_names();

	for(auto jn : jns) {
		auto pos = find(js.name.begin(), js.name.end(), jn);
		if(pos == js.name.end())
			return false;		
		auto d = distance(js.name.begin(), pos);
		p.positions.push_back(js.position[d]);
		p.velocities.push_back(0.0);
		p.accelerations.push_back(0.0);
		p.effort.push_back(0.0);
	}
	return true;
}

std::vector<std::string> ScrollGarment::get_joint_names()  {
	std::vector<std::string> joint_names;
	joint_names.push_back("r1_joint_s");
	joint_names.push_back("r1_joint_l");
	joint_names.push_back("r1_joint_u");
	joint_names.push_back("r1_joint_r");
	joint_names.push_back("r1_joint_b");
	joint_names.push_back("r1_joint_t");
	joint_names.push_back("r2_joint_s");
	joint_names.push_back("r2_joint_l");
	joint_names.push_back("r2_joint_u");
	joint_names.push_back("r2_joint_r");
	joint_names.push_back("r2_joint_b");
	joint_names.push_back("r2_joint_t");
	joint_names.push_back("ext_axis");
	return joint_names;
}

bool ScrollGarment::get_current_point(trajectory_msgs::JointTrajectoryPoint& p) {
	bool rtn = true;
	mutex_joints_.lock();
	rtn = joints_to_point(joints_, p);
	mutex_joints_.unlock();
	return rtn;
}

void ScrollGarment::add_current_state_to_trajectory(trajectory_msgs::JointTrajectory& trajectory){
	trajectory_msgs::JointTrajectory oldTrajectory = trajectory;
	trajectory_msgs::JointTrajectoryPoint p;

	get_current_point(p);
	std::vector<std::string> joint_names = get_joint_names();

	trajectory.joint_names.clear();
	trajectory.points.clear();

	// trajectory.points.push_back(p);
	

	robot_state::RobotState rs(*crc_.getCurrentState());
	crc_.setStartState(rs);

	BOOST_FOREACH(const std::string & jn, joint_names) {
		trajectory.joint_names.push_back(jn);
	}

	BOOST_FOREACH(const trajectory_msgs::JointTrajectoryPoint & point, oldTrajectory.points) {
		for(unsigned int i = 0 ; i < oldTrajectory.joint_names.size(); ++i) {
			std::string jn = oldTrajectory.joint_names[i];
			std::vector<std::string>::iterator it;
			it = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), jn);
			unsigned int ind = std::distance(trajectory.joint_names.begin(), it);
			p.positions[ind] = point.positions[i];
		}

		p.time_from_start = point.time_from_start;
		trajectory.points.push_back(p);		
	}	
}


void ScrollGarment::cb_joint(const sensor_msgs::JointState& msg) {
	trajectory_msgs::JointTrajectoryPoint p;
	if(!joints_to_point(msg, p))
		return;
	mutex_joints_.lock();
	joints_ = msg;
	mutex_joints_.unlock();


	robot_state::RobotState rs_link(*crc_.getCurrentState());
	mutex_msg_.lock();
	rs_link.setVariableValues(msg);
	mutex_msg_.unlock();

	link_r1_force_sensor_ = rs_link.getGlobalLinkTransform ("r1_force_sensor").inverse();
	WrenchR1_.setRotMat(link_r1_force_sensor_);

	link_r2_force_sensor_ = rs_link.getGlobalLinkTransform ("r2_force_sensor").inverse();
	WrenchR2_.setRotMat(link_r2_force_sensor_);

	Eigen::Affine3d e_target, e_tmp, e_r1_ee, e_r2_ee, e_table;

	e_table = rs_link.getFrameTransform(table_frame_);
	e_r1_ee = rs_link.getFrameTransform("r1_ee");
	e_r2_ee = rs_link.getFrameTransform("r2_ee");

	e_tmp = e_table.inverse() * e_r1_ee;
	z_r1_ = e_tmp(2,3);
	e_tmp = e_table.inverse() * e_r2_ee;
	z_r2_ = e_tmp(2,3);

}
