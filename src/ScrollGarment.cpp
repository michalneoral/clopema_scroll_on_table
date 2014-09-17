#include "ScrollGarment.h"

ScrollGarment::ScrollGarment(): crc_("arms"){

	sub_joints_ = node_.subscribe("/joint_states", 1, &ScrollGarment::cb_joint, this);
	pub_marker_ = node_.advertise<geometry_msgs::PoseArray>("points_traj_scroll_garment", 0);
	WrenchR1_.startSub("r1");
	WrenchR2_.startSub("r2");
	ready_g_ = false;
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
	// elinks1.push_back("r1_ee");
	// elinks2.push_back("r2_ee");
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

	/*Test positions for arm R1*/
	{
		blank.position = waypoints_1.front();
		blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
		wp1.push_back(blank);

		// blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST-0.005;
		// wp1.push_back(blank);

		for (i = 0; i < waypoints_1.size(); i++){
			blank.position = waypoints_1[i];
			blank.position.z = TEST_TRAJECTORY_HEIGHT_MIN + ADD_HEIGHT_TEST + offset;
			wp1.push_back(blank);
		}

		blank.position = waypoints_1.back();
		blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;
		wp1.push_back(blank);

		for (i = 0; i < waypoints_1.size(); i++){
			blank.position = waypoints_1[i];
			blank.position.z = TEST_TRAJECTORY_HEIGHT_MAX + ADD_HEIGHT_TEST + offset;
			wp1.push_back(blank);
		}
		blank.position = waypoints_1.back();
		blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;
		wp1.push_back(blank);
	}
	/*Test positions for arm R2*/
	{	
		blank.position = waypoints_2.front();
		blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
		wp2.push_back(blank);

		// blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST-0.005;
		// wp2.push_back(blank);

		for (i = 0; i < waypoints_2.size(); i++){
			blank.position = waypoints_2[i];
			blank.position.z = TEST_TRAJECTORY_HEIGHT_MIN + ADD_HEIGHT_TEST + offset;
			wp2.push_back(blank);
		}

		blank.position = waypoints_2.back();
		blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;
		wp2.push_back(blank);
		
		for (i = 0; i < waypoints_2.size(); i++){
			blank.position = waypoints_2[i];
			blank.position.z = TEST_TRAJECTORY_HEIGHT_MAX + ADD_HEIGHT_TEST + offset;
			wp2.push_back(blank);
		}

		blank.position = waypoints_2.back();
		blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;
		wp2.push_back(blank);
	}	
}

void ScrollGarment::showMarkers(std::string table_frame, const std::vector<geometry_msgs::Pose>& wp1, const std::vector<geometry_msgs::Pose>& wp2) {
	//---------------------------------------------------------------------------------------
	geometry_msgs::PoseArray allposes;
	geometry_msgs::Pose pose;
	allposes.header.frame_id = table_frame;
	allposes.header.stamp = ros::Time();

	// pose.orientation.x = 0.0;
	// pose.orientation.y = 0.0;
	// pose.orientation.z = 0.0;
	// pose.orientation.w = 1.0;		

	for(int i = 0; i < wp1.size(); i++){
		pose = wp1[i];			
		allposes.poses.push_back(pose);
		pose = wp2[i];
		allposes.poses.push_back(pose);		
	}
	pub_marker_.publish( allposes );
}

double modulus(double a, double b)
{
	if(a <= 0){
		return modulus(a+b,b);
	}else{
		int result = static_cast<int>( a / b );
		return a - static_cast<double>( result ) * b;
	}
}

double mod2pi(double a){
	return modulus(a, 2*M_PI);
}

bool ScrollGarment::testTrajectory(std::string table_frame , double& yawR1, double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, bool& conf){
//-------------------------------------------------------------------------------------------

	bool ready = false;
	double current_yawR1=0, current_yawR2=0;

	while(!ready){
		mutex_ready_g_.lock();
		ready = ready_g_;
		mutex_ready_g_.unlock();

		mutex_cur_yawR1_.lock();
		current_yawR1 = cur_yawR1_;
		mutex_cur_yawR1_.unlock();

		mutex_cur_yawR2_.lock();
		current_yawR2 = cur_yawR2_;
		mutex_cur_yawR2_.unlock();
	}

	std::vector<geometry_msgs::Pose> wp1, wp2;
	double offset = 0;
	double angle_number_step = ANGLE_NUMBER_STEP;

	double ext_axis_yaw;
	mutex_ext_axis_yaw_.lock();
	ext_axis_yaw = ext_axis_yaw_;
	mutex_ext_axis_yaw_.unlock();

	
	for (int i=0; i< angle_number_step; i++) {
		yawR1 = mod2pi(2*M_PI + current_yawR1 + i*(2*M_PI/angle_number_step));
		// if( mod2pi(mod2pi(yawR1-ext_axis_yaw) - MAX_ANGLE_DIFF) < MAX_ANGLE_DIFF){// || mod2pi(mod2pi(yawR1-ext_axis_yaw) - MIN_ANGLE_DIFF) < MIN_ANGLE_DIFF){
		// 	std::cout << "c1";
		// 	// continue;
		// }

		for (int j=0; j< angle_number_step; j++) {			
			yawR2 = mod2pi(2*M_PI + current_yawR2 + j*(2*M_PI/angle_number_step));
			// if( mod2pi(mod2pi(yawR2-ext_axis_yaw) - MAX_ANGLE_DIFF) < MAX_ANGLE_DIFF){// || mod2pi(mod2pi(yawR2-ext_axis_yaw) - MIN_ANGLE_DIFF) < MIN_ANGLE_DIFF){
			// 	std::cout << "c2";
			// 	continue;
			// }
			// std::cout << "\t R1: " << mod2pi(yawR1-ext_axis_yaw) << "\t" << mod2pi(mod2pi(yawR1-ext_axis_yaw) - MAX_ANGLE_DIFF)<< "\t R2: " << mod2pi(yawR2-ext_axis_yaw) << "\t" << mod2pi(mod2pi(yawR2-ext_axis_yaw) - MAX_ANGLE_DIFF) << "\t C: " << mod2pi(MAX_ANGLE_DIFF) << "\t" << mod2pi(MIN_ANGLE_DIFF) << std::endl;

			getTestPositions(table_frame, wp1, wp2, waypoints_1, waypoints_2, yawR1, yawR2, offset);
			// std::cout << "yawR1: " << yawR1 << "\t yawR2: " << yawR2 << " (" << current_yawR1 << ", " << current_yawR2 << ")" << std::endl;

			moveit_msgs::RobotTrajectory trajectories;
			if(!planPoses(trajectories, elinks1, elinks2, wp1, wp2, true, TEST_STEP, false))
			{
				if(planPoses(trajectories, elinks1, elinks2, wp1, wp2, false, TEST_STEP, false))
				{
					if(planPoses(trajectories, elinks1, elinks2, wp1, wp2, false, STEP, false))
					{
						conf = false;	
						ROS_INFO_STREAM(i*ANGLE_NUMBER_STEP+j+1);				
						return true;
					}
				}
			} else if(planPoses(trajectories, elinks1, elinks2, wp1, wp2, true, STEP, false))
			{
				conf = true;
				ROS_INFO_STREAM(i*ANGLE_NUMBER_STEP+j+1);
				return true;
			}

		}
		// std::cout << "----------------" << std::endl;
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
		}else{
			isOk = false;
			mutex_z_r1_.lock();
			blank.position.z = z_r1_ + (WrenchR1_.getForce() - force) * FORCE_CONST;
			mutex_z_r1_.unlock();				
		}		
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
		wp1.push_back(blank);

		blank.position = waypoints_2[0];		
		if(!WrenchR1_.isForceOk(force)){			
			isOk = false;
			mutex_z_r2_.lock();
			blank.position.z = z_r2_ + (WrenchR2_.getForce() - force) * FORCE_CONST;
			mutex_z_r2_.unlock();
		}		
		blank.orientation = transformQuaternion(table_frame, ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
		wp2.push_back(blank);
	}
	
	if(isOk){ 
		return 2;
	} else if (planPoses(trajectory, elinks1, elinks2, wp1, wp2, conf, STEP, true)){
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

bool ScrollGarment::scrollOverTable(std::string table_frame , const double& yawR1, const double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, bool conf, double force) {	
	//----------------------------------------------------------------------------

	std::vector<geometry_msgs::Pose> wp1, wp2;
	geometry_msgs::Pose blank;
	
	
	for (int i = 1; i < waypoints_1.size(); i++){
		bool next = false, nextnext = false;
		while(!next) 
		{
			moveit_msgs::RobotTrajectory trajectory;
			wp1.clear();
			wp2.clear();


			blank.position = waypoints_1[i];
			mutex_z_r1_.lock();
			blank.position.z = z_r1_ + (WrenchR1_.getForce() - force) * FORCE_CONST;
			// std::cout << "R1: old z= " << z_r1_ << " new z= " << blank.position.z << " force: " << WrenchR1_.getForce() << std::endl;
			mutex_z_r1_.unlock();
			blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
			wp1.push_back(blank);

			blank.position = waypoints_2[i];
			mutex_z_r2_.lock();
			blank.position.z = z_r2_ + (WrenchR2_.getForce() - force) * FORCE_CONST;
			// std::cout << "R2: old z= " << z_r2_ << " new z= " << blank.position.z << " force: " << WrenchR2_.getForce() << std::endl;
			mutex_z_r2_.unlock();
			blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
			wp2.push_back(blank);


			if(planPoses(trajectory, elinks1, elinks2, wp1, wp2, conf, STEP, true)){
				trajectory.joint_trajectory.points.erase(trajectory.joint_trajectory.points.begin());
				// std::cout << "[scrollOverTable] Trajectory size: " << trajectory.joint_trajectory.points.size() << std::endl;
				if (trajectory.joint_trajectory.points.size() <= 4 || nextnext){
					next = true;
				} else if (trajectory.joint_trajectory.points.size() <= 7){
					nextnext = true;
				} else {
					trajectory.joint_trajectory.points.erase(trajectory.joint_trajectory.points.begin()+4,trajectory.joint_trajectory.points.end());					
				}				
				add_current_state_to_trajectory(trajectory.joint_trajectory);
				if(!crc_.execute_traj(trajectory.joint_trajectory)){
					ROS_ERROR("Cannot move.");
					return 0;
				}
			}else{				
				ROS_ERROR("Trajectory wasn't found.");
				return 0;				
			}			
		}
	}
	return 1;
}


bool ScrollGarment::startStopPosition(std::string table_frame , const double& yawR1, const double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, const bool& conf, bool start) {
	//------------------------------------------------------------------------
	std::vector<geometry_msgs::Pose> wp1, wp2;
	moveit_msgs::RobotTrajectory trajectory;
	geometry_msgs::Pose blank, p1, p2;
	// Prepare poses

	std::cout << yawR1 << " \t" << yawR2 << "\t\t" << conf << "\t" << waypoints_1.size() << "\t" << waypoints_2.size() << std::endl;

	if(start){

		blank.position = waypoints_1[0];
		blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
		p1 = blank; 
		blank.position.z = OVER_TABLE_HEIGHT + ADD_HEIGHT_TEST;
		wp1.push_back(blank);

		blank.position = waypoints_2[0];
		blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
		p2 = blank;
		blank.position.z = OVER_TABLE_HEIGHT + ADD_HEIGHT_TEST;
		wp2.push_back(blank);
	} else {
		blank.position = waypoints_1.back();
		blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
		wp1.push_back(blank);

		blank.position = waypoints_2.back();
		blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
		wp2.push_back(blank);
	}

	// std::cout << "wp1: " << wp1.size() << " wp2: " << wp2.size() << std::endl; 
	if(start){
		robot_state::RobotState rs(*crc_.getCurrentState());
		// crc_.setStartStateToCurrentState();
		std::string tip_1, tip_2, group_1, group_2;
		getNamesConfiguration(conf, tip_1, tip_2, group_1, group_2);
		if (!rs.setFromIK(rs.getJointModelGroup(group_1), p1, tip_1)) {
			ROS_WARN_STREAM("Cannot set from IK - second arm");
			return false;
		}
		if (!rs.setFromIK(rs.getJointModelGroup(group_2), p2, tip_2)) {
			ROS_WARN_STREAM("Cannot set from IK - second arm");
			return false;
		}
		crc_.setJointValueTarget(rs);
		if(!crc_.move()){
			return false;
		}
	}

	if(planPoses(trajectory, elinks1, elinks2, wp1, wp2, conf, STEP_START_STOP, JUMP_TRESHOLD_NONE, true)){
		trajectory.joint_trajectory.points.erase(trajectory.joint_trajectory.points.begin());
		add_current_state_to_trajectory(trajectory.joint_trajectory);
		pub_traj_.controle(trajectory.joint_trajectory);
		return crc_.execute_traj(trajectory.joint_trajectory);
	}else{		
		ROS_ERROR("Cannot interpolate");
		return false;
	}
}

void ScrollGarment::getNamesConfiguration(bool comb, std::string& tip_1, std::string& tip_2, std::string& group_1, std::string& group_2){
	//------------------------------------------------------------------------
	if(comb) {
		tip_1 = "r1_ee"; tip_2 = "r2_ee"; group_1 = "r1_arm"; group_2 = "r2_arm";
	} else {
		tip_1 = "r2_ee"; tip_2 = "r1_ee"; group_1 = "r2_arm"; group_2 = "r1_arm";
	}
}

bool ScrollGarment::planPoses(moveit_msgs::RobotTrajectory &trajectories, const std::vector<std::string> elinks1, const std::vector<std::string> elinks2, const std::vector<geometry_msgs::Pose>& wp1, const std::vector<geometry_msgs::Pose>& wp2, const bool first_combination, double step, bool current_state){
	//------------------------------------------------------------------------
	return planPoses(trajectories, elinks1, elinks2, wp1, wp2, first_combination, step, JUMP_TRESHOLD, current_state);
}

bool ScrollGarment::planPoses(moveit_msgs::RobotTrajectory &trajectories, const std::vector<std::string> elinks1, const std::vector<std::string> elinks2, const std::vector<geometry_msgs::Pose>& wp1, const std::vector<geometry_msgs::Pose>& wp2, const bool first_combination, double step, double jump_treshold, bool current_state){
	//------------------------------------------------------------------------

	char buffer [100];

	showMarkers(table_frame_, wp1, wp2);

	std::string tip_1, tip_2, group_1, group_2;
	getNamesConfiguration(first_combination, tip_1, tip_2, group_1, group_2);

	double d;
	int max=16;

	std::vector<geometry_msgs::Pose> wp1_copy=wp1, wp2_copy=wp2;
	crc_.setStartStateToCurrentState();
	robot_state::RobotState rs(*crc_.getCurrentState());

	if (!current_state){
		geometry_msgs::Pose pose_tmp;
		crc_.setPoseReferenceFrame("base_link");
		
		Eigen::Affine3d e, e_table, e_target, e_tmp;

		e_table = rs.getFrameTransform(table_frame_);
		e_target = rs.getFrameTransform("base_link");

		pose_tmp = wp1_copy.front();
		tf::poseMsgToEigen(pose_tmp, e);
		e_tmp = e_target.inverse() * e_table * e;
		tf::poseEigenToMsg(e_tmp, pose_tmp);

		if(!rs.setFromIK (rs.getJointModelGroup(group_1), pose_tmp, tip_1)){
			ROS_ERROR("Error - setFromIK 1 ");
			crc_.setPoseReferenceFrame(table_frame_);
			return false;
		}

		pose_tmp = wp2_copy.front();
		tf::poseMsgToEigen(pose_tmp, e);
		e_tmp = e_target.inverse() * e_table * e;
		tf::poseEigenToMsg(e_tmp, pose_tmp);

		if(!rs.setFromIK (rs.getJointModelGroup(group_2), pose_tmp, tip_2)){
			ROS_ERROR("Error - setFromIK 2 ");
			crc_.setPoseReferenceFrame(table_frame_);
			return false;
		}

		crc_.setPoseReferenceFrame(table_frame_);

		wp1_copy.erase(wp1_copy.begin());
		wp2_copy.erase(wp2_copy.begin());
		
	}
	crc_.setStartState(rs);

	d = crc_.computeCartesianPathDual(wp1_copy, tip_1, wp2_copy, tip_2, step, jump_treshold, trajectories, false);

	if(!(fabs(d - 1.0) < 0.001)) {
		sprintf(buffer, "cannot interpolate up. d = %.4f (%.4f).", d, fabs(d - 1.0));
		ROS_WARN_STREAM(buffer);
		return false;
	} else{
		if(!crc_.check_trajectory(trajectories, elinks1, elinks2)) {
			sprintf(buffer, "cannot interpolate up because of collision");
			ROS_WARN_STREAM(buffer);
			return false;
		}
		else{
			// std::cout << "INTERPOLATE OK!" << std::endl;
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
	crc_.setStartStateToCurrentState();

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
	// rs_actual_=rs_link;cb
	mutex_msg_.unlock();

	link_r1_force_sensor_ = rs_link.getGlobalLinkTransform ("r1_force_sensor").inverse();
	WrenchR1_.setRotMat(link_r1_force_sensor_);

	link_r2_force_sensor_ = rs_link.getGlobalLinkTransform ("r2_force_sensor").inverse();
	WrenchR2_.setRotMat(link_r2_force_sensor_);

	Eigen::Affine3d e_target, e_tmp, e_r1_ee, e_r2_ee, e_table, e_base;

	e_table = rs_link.getFrameTransform(table_frame_);
	e_r1_ee = rs_link.getFrameTransform("r1_ee");
	e_r2_ee = rs_link.getFrameTransform("r2_ee");

	e_tmp = e_table.inverse() * e_r1_ee;
	z_r1_ = e_tmp(2,3);
	e_tmp = e_table.inverse() * e_r2_ee;
	z_r2_ = e_tmp(2,3);
	
	tf::Pose blankPose;
	tf::Matrix3x3 blankMatrix;
	double roll, pitch;

	e_base = rs_link.getFrameTransform("base_link");

	e_tmp = e_base.inverse() * e_r1_ee;
	tf::poseEigenToTF(e_tmp, blankPose);
	blankMatrix = tf::Matrix3x3 (blankPose.getRotation());	
	blankMatrix.getRPY (roll, pitch, cur_yawR1_, 2);

	e_tmp = e_base.inverse() * e_r2_ee;
	tf::poseEigenToTF(e_tmp, blankPose);
	blankMatrix = tf::Matrix3x3 (blankPose.getRotation());	
	blankMatrix.getRPY (roll, pitch, cur_yawR2_, 2);

	double torso_yaw;
	Eigen::Affine3d e_torso = rs_link.getFrameTransform("torso_link");
	tf::poseEigenToTF(e_torso, blankPose);
	blankMatrix = tf::Matrix3x3 (blankPose.getRotation());	
	blankMatrix.getRPY (roll, pitch, torso_yaw, 1);

	ext_axis_yaw_ = torso_yaw;
	
	ready_g_ = true;
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
	char buffer[100];

	transformFromFrameToTable(frame_id, table_frame, waypoints_1, waypoints_2);

	/*
	At least two points in waypoints vectors
	*/
	if (!(waypoints_1.size() >= 2 && waypoints_2.size() >= 2)){
		ROS_INFO_STREAM(waypoints_1.size());
		ROS_INFO_STREAM(waypoints_2.size());
		ROS_ERROR("Path from waypoints does not contain final destination");
		return false;
	}

	/*
	Set force must be between FORCE_SET_MIN and FORCE_SET_MAX
	*/
	if (force > FORCE_SET_MAX){		
		sprintf(buffer, "Set force %.1f N is too high therefore was set to max %.1f N", force, FORCE_SET_MAX);
		force = FORCE_SET_MAX;
		ROS_WARN_STREAM(buffer);
	} else if (force < FORCE_SET_MIN) {
		sprintf(buffer, "Set force %.3f N is too low therefore was set to min %.3f N", force, FORCE_SET_MIN);
		force = FORCE_SET_MIN;
		ROS_WARN_STREAM(buffer);
	}
	// ==================================== ↑↑↑ INIT ↑↑↑ ====================================

	if(testTrajectory(table_frame, yawR1, yawR2, waypoints_1, waypoints_2, elinks1, elinks2, conf)){

		ROS_INFO_STREAM(conf); ROS_INFO_STREAM(yawR1); ROS_INFO_STREAM(yawR2);

		if(crc_.execute_traj(crc_.gripper_trajectory(clopema_robot::GRIPPER_OPEN))){
			ROS_INFO_STREAM("Open grippers OK.");
		} else {
			ROS_ERROR("Cannot open grippers.");
		}
		

		if( !startStopPosition(table_frame, yawR1, yawR2, waypoints_1, waypoints_2, elinks1, elinks2, conf, true) ){
			ROS_ERROR("Cannot execution move to the start position.");
			crc_.setServoPowerOff();
			return false;
		} else {
			ROS_INFO_STREAM("Start position OK.");
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

		ROS_INFO_STREAM("Press on table OK.");
		
		ros::Duration(SLEEP_AFTER_EXEC).sleep();

		if(!scrollOverTable(table_frame, yawR1, yawR2, waypoints_1, waypoints_2, elinks1, elinks2, conf, force)){
			ROS_ERROR("Cannot scroll.");
			crc_.setServoPowerOff();
			return false;
		} else {
			ROS_INFO_STREAM("Scroll garment on a table OK.");
		}

		if( !startStopPosition(table_frame, yawR1, yawR2, waypoints_1, waypoints_2, elinks1, elinks2, conf, false) ){
			ROS_ERROR("Cannot execution move to the end position.");
			crc_.setServoPowerOff();
			return false;
		} else {
			ROS_INFO_STREAM("Stop position OK.");
		}


	}else{
		ROS_ERROR("Trajectory wasn't found.");
		crc_.setServoPowerOff();
		return false;
	}
	crc_.setServoPowerOff();
	return true;
}

void ScrollGarment::showForces(){
	for (int n = 0; n < 2; n++){
		std::cout << std::endl;
	}
	std::cout << ros::Time::now() << std::endl;
	WrenchR1_.showForces();
	WrenchR2_.showForces();
}

bool ScrollGarment::testWeight(	std::string frame_id,	std::vector< geometry_msgs::Point >& waypoints_1,
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
	char buffer[100];

	transformFromFrameToTable(frame_id, table_frame, waypoints_1, waypoints_2);

	/*
	At least two points in waypoints vectors
	*/
	if (!(waypoints_1.size() >= 1 && waypoints_2.size() >= 1)){
		ROS_INFO_STREAM(waypoints_1.size());
		ROS_INFO_STREAM(waypoints_2.size());
		ROS_ERROR("Path from waypoints does not contain final destination");
		return false;
	}
	// waypoints_1.erase(waypoints_1.begin()+1,waypoints_1.end());
	// waypoints_2.erase(waypoints_2.begin()+1,waypoints_2.end());
	// waypoints_1.push_back(waypoints_1.front());
	// waypoints_2.push_back(waypoints_2.front());

	/*
	Set force must be between FORCE_SET_MIN and FORCE_SET_MAX
	*/
	if (force > FORCE_SET_MAX){		
		sprintf(buffer, "Set force %.1f N is too high therefore was set to max %.1f N", force, FORCE_SET_MAX);
		force = FORCE_SET_MAX;
		ROS_WARN_STREAM(buffer);
	} else if (force < FORCE_SET_MIN) {
		sprintf(buffer, "Set force %.3f N is too low therefore was set to min %.3f N", force, FORCE_SET_MIN);
		force = FORCE_SET_MIN;
		ROS_WARN_STREAM(buffer);
	}
	// ==================================== ↑↑↑ INIT ↑↑↑ ====================================

	std::cout << "Here" << std::endl;

	if(testTrajectory(table_frame, yawR1, yawR2, waypoints_1, waypoints_2, elinks1, elinks2, conf)){

		ROS_INFO_STREAM(conf); ROS_INFO_STREAM(yawR1); ROS_INFO_STREAM(yawR2);

		if(crc_.execute_traj(crc_.gripper_trajectory(clopema_robot::GRIPPER_OPEN))){
			ROS_INFO_STREAM("Open grippers OK.");
		} else {
			ROS_ERROR("Cannot open grippers.");
		}		

		if( !startStopPosition(table_frame, yawR1, yawR2, waypoints_1, waypoints_2, elinks1, elinks2, conf, true) ){
			ROS_ERROR("Cannot execution move to the start position.");
			crc_.setServoPowerOff();
			return false;
		} else {
			ROS_INFO_STREAM("Start position OK.");
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

		ROS_INFO_STREAM("Press on table OK.");

		crc_.setServoPowerOff();		
		ros::Duration(5).sleep();

		if( !startStopPosition(table_frame, yawR1, yawR2, waypoints_1, waypoints_2, elinks1, elinks2, conf, false) ){
			ROS_ERROR("Cannot execution move to the end position.");
			crc_.setServoPowerOff();
			return false;
		} else {
			ROS_INFO_STREAM("Stop position OK.");
		}
	}else{
		ROS_ERROR("Trajectory wasn't found.");
		crc_.setServoPowerOff();
		return false;
	}
	crc_.setServoPowerOff();
	return true;
}

//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

bool ScrollGarment::moveOverTablePiecewise(	std::string frame_id,	std::vector< geometry_msgs::Point >& waypoints_1,
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
	char buffer[100];

	transformFromFrameToTable(frame_id, table_frame, waypoints_1, waypoints_2);


		// At least two points in waypoints vectors

	if (!(waypoints_1.size() >= 2 && waypoints_2.size() >= 2)){
		ROS_INFO_STREAM(waypoints_1.size());
		ROS_INFO_STREAM(waypoints_2.size());
		ROS_ERROR("Path from waypoints does not contain final destination");
		return false;
	}

	if ((waypoints_1.size() != waypoints_2.size())){
		ROS_INFO_STREAM(waypoints_1.size());
		ROS_INFO_STREAM(waypoints_2.size());
		ROS_ERROR("Waypoints contain different number of points");
		return false;
	}


		// Set force must be between FORCE_SET_MIN and FORCE_SET_MAX

	if (force > FORCE_SET_MAX){		
		sprintf(buffer, "Set force %.1f N is too high therefore was set to max %.1f N", force, FORCE_SET_MAX);
		force = FORCE_SET_MAX;
		ROS_WARN_STREAM(buffer);
	} else if (force < FORCE_SET_MIN) {
		sprintf(buffer, "Set force %.3f N is too low therefore was set to min %.3f N", force, FORCE_SET_MIN);
		force = FORCE_SET_MIN;
		ROS_WARN_STREAM(buffer);
	}
	
	// ==================================== ↑↑↑ INIT ↑↑↑ ====================================

	ScrollGarmentSinglePathConfig currentPathConfig, previousPathConfig;
		//TODO

		//Basic test of existence path - necessary but not sufficient condition
	ROS_INFO_STREAM("Testting IK of waypoints...");
	if(!testWaypointsPosition(waypoints_1, waypoints_2)){
		ROS_ERROR("Path does not exist. Cannot set IK for some waypoint");
		return false;
	}
	ROS_INFO_STREAM("...is OK.");

	for (int i = 0; i < waypoints_1.size()-1; i++){

		currentPathConfig.changeWaypoint1(waypoints_1[i], waypoints_1[i+1]);
		currentPathConfig.changeWaypoint2(waypoints_2[i], waypoints_2[i+1]);



			// FIND PART OF (OR WHOLE) TRAJECTORY
		if(!findBiggestTrajectory(currentPathConfig, elinks1, elinks2)){
			
			// CONTROL MINIMALY LENGHT OF TRAJECTORY, NUMBER OF STEPS AND PERCENTAGE
			// 	- TESTING OF [ZACYKLENÍ]

			// 
			{
				std::cout << "DISTANCES 1: " << getLenght(waypoints_1, i, false) << " (" << getLenght(currentPathConfig.short_waypoints_1_) << ")" << std::endl;
				std::cout << "DISTANCES 2: " << getLenght(waypoints_2, i, false) << " (" << getLenght(currentPathConfig.short_waypoints_2_) << ")" << std::endl;
				
				if(currentPathConfig.isEqual(previousPathConfig)){
					std::cout << "\033[1;33mSame like prev.\033[0m"<< std::endl; //]]
				}

				if(currentPathConfig.isEqual(previousPathConfig) && getLenght(waypoints_1, i, false) < MINIMAL_DISTANCE &&  getLenght(waypoints_2, i, false) < MINIMAL_DISTANCE ) {
					return 0;					
				}

				if(currentPathConfig.isEqual(previousPathConfig) && getLenght(currentPathConfig.short_waypoints_1_) < MINIMAL_DISTANCE &&  getLenght(currentPathConfig.short_waypoints_2_) < MINIMAL_DISTANCE ) {
					return 0;					
				}
			}

			// IF TRAJECTORY IS NOT COMPLETE - FIND AND ADD LAST POINT OF TRAJECTORY TO WAYPOINTS
			previousPathConfig = currentPathConfig;
			std::vector<geometry_msgs::Point>::iterator it;
			it = waypoints_1.begin();
			waypoints_1.insert(it+1+i , currentPathConfig.short_waypoints_1_.back());
			it = waypoints_2.begin();
			waypoints_2.insert(it+1+i , currentPathConfig.short_waypoints_2_.back());
		}

		// std::cout << currentPathConfig.yawR1_ << "\t\t" << currentPathConfig.yawR2_ << std::endl;

			// MOVE PART
		{
			{ 
			// OPEN GRIPPER
				if(crc_.execute_traj(crc_.gripper_trajectory(clopema_robot::GRIPPER_OPEN))){
					ROS_INFO_STREAM("Open grippers OK.");
				} else {
					ROS_ERROR("Cannot open grippers.");
				}
			}

			{ 
			// GO TO START POSITION
				if( !startStopPosition(currentPathConfig, elinks1, elinks2, START) ){
					ROS_ERROR("Cannot execution move to the start position.");
					crc_.setServoPowerOff();
					return false;
				} else {
					ROS_INFO_STREAM("Start position OK.");
				}
			}

			ros::Duration(SLEEP_AFTER_EXEC).sleep();

			WrenchR1_.setInitialize();
			WrenchR2_.setInitialize();


			{ 
			// Press down on the table
				while(true){
					int statueOfPress = pressOnTheTable(currentPathConfig, elinks1, elinks2, force);
					if( statueOfPress == 0){
						ROS_ERROR("Cannot press on the table.");
						crc_.setServoPowerOff();
						return false;
					} else if (statueOfPress == 2){
						break;
					}
				}
			}
			ROS_INFO_STREAM("Press on table OK.");
			ros::Duration(SLEEP_AFTER_EXEC).sleep();

			{ 
			// Scroll over table
				bool split = true;
				if(!scrollOverTable(currentPathConfig, elinks1, elinks2, force)){
					if (split) {						
						if (sub_traj_ > MAX_SUB_TRAJ) {
							sub_traj_ += 0.05;
							i -= 1;
							continue;
						} else {
							sub_traj_ = 0;
							return 0;
						}
					} else {
						ROS_ERROR("Cannot scroll.");
						crc_.setServoPowerOff();
						return false;
					}
				} else {
					sub_traj_ = 0;
					ROS_INFO_STREAM("Scroll garment on a table OK.");
				}
			}

			{ 
			// Stop position
				if( !startStopPosition(currentPathConfig, elinks1, elinks2, STOP) ){
					ROS_ERROR("Cannot execution move to the end position.");
					crc_.setServoPowerOff();
					return false;
				} else {
					ROS_INFO_STREAM("Stop position OK.");
				}
			}
		}
	}
	return true;
}

double ScrollGarment::getLenght(const std::vector<geometry_msgs::Point>& wp, int i, bool fromCur2End){
	if (wp.size() > 1) {
		double diffx;
		double diffy;
		if(fromCur2End){
			diffx = wp[i].x - wp.back().x;
			diffy = wp[i].y - wp.back().y;
		}else {
			diffx = wp[i].x - wp[i+1].x;
			diffy = wp[i].y - wp[i+1].y;
		}
		return sqrt( diffx*diffx + diffy*diffy ); 
	}
	return 0;
}

double ScrollGarment::getLenght(const std::vector<geometry_msgs::Point>& wp){
	if (wp.size() > 1) {
		double diffx;
		double diffy;
		diffx = wp.front().x - wp.back().x;
		diffy = wp.front().y - wp.back().y;
		return sqrt( diffx*diffx + diffy*diffy ); 
	}
	return 0;
}

double ScrollGarment::testPathConfig(moveit_msgs::RobotTrajectory &trajectories, const std::vector<std::string> elinks1, const std::vector<std::string> elinks2, const std::vector<geometry_msgs::Pose>& wp1, const std::vector<geometry_msgs::Pose>& wp2, const bool first_combination, double step, bool current_state, const double percent_of_path, bool& isBiggest){
	//------------------------------------------------------------------------

	isBiggest = false;
	char buffer [100];
	showMarkers(table_frame_, wp1, wp2);

	std::string tip_1 = "r1_ee", tip_2 = "r2_ee", conf = "Configuration 1: ", group_1 = "r1_arm", group_2 = "r2_arm";
	if(!first_combination) {
		tip_1 = "r2_ee", tip_2 = "r1_ee", conf = "Configuration 2: ", group_1 = "r2_arm", group_2 = "r1_arm";
	}

	double d;
	int max=16;

	std::vector<geometry_msgs::Pose> wp1_copy = wp1;
	std::vector<geometry_msgs::Pose> wp2_copy = wp2;

	crc_.setStartStateToCurrentState();

	if (!current_state){
		geometry_msgs::Pose pose_tmp;
		crc_.setPoseReferenceFrame("base_link");

		robot_state::RobotState rs(*crc_.getCurrentState());

		// { // SET FIRSTLY TO START POSITION - DOESNT WORK
		// 	std::vector<std::string> j_names;
		// 	j_names = get_joint_names();
		// 	for (int i=0; i<j_names.size()-1; i++){
		// 		rs.setVariablePosition(j_names[i], 0);
		// 	}
		// 	crc_.setStartState(rs);
		// }

		Eigen::Affine3d e, e_table, e_target, e_tmp;

		e_table = rs.getFrameTransform(table_frame_);
		e_target = rs.getFrameTransform("base_link");

		pose_tmp = wp1_copy.front();
		tf::poseMsgToEigen(pose_tmp, e);
		e_tmp = e_target.inverse() * e_table * e;
		tf::poseEigenToMsg(e_tmp, pose_tmp);

		if(!rs.setFromIK (rs.getJointModelGroup(group_1), pose_tmp, tip_1)){
			// ROS_ERROR("Error - setFromIK 1 ");
			crc_.setPoseReferenceFrame(table_frame_);
			return -1;
		}

		pose_tmp = wp2_copy.front();
		tf::poseMsgToEigen(pose_tmp, e);
		e_tmp = e_target.inverse() * e_table * e;
		tf::poseEigenToMsg(e_tmp, pose_tmp);

		if(!rs.setFromIK (rs.getJointModelGroup(group_2), pose_tmp, tip_2)){
			// ROS_ERROR("Error - setFromIK 2 ");
			crc_.setPoseReferenceFrame(table_frame_);
			return -1;
		}

		crc_.setPoseReferenceFrame(table_frame_);

		wp1_copy.erase(wp1_copy.begin());
		wp2_copy.erase(wp2_copy.begin());

		crc_.setStartState(rs);
	}

	
	crc_.setPoseReferenceFrame(table_frame_);
	// crc_.setStartStateToCurrentState();

	d = crc_.computeCartesianPathDual(wp1_copy, tip_1, wp2_copy, tip_2, step, JUMP_TRESHOLD, trajectories, false);
	if(d > percent_of_path){
		if(!crc_.check_trajectory(trajectories, elinks1, elinks2)) {
			return -1.0;
		}
		else{
			isBiggest = true;
			return d;
		}
	}	
	return -1.0;	
}


bool ScrollGarment::testWaypointsPosition(const std::vector<geometry_msgs::Point> waypoints_1, const std::vector<geometry_msgs::Point> waypoints_2){
	//------------------------------------------------------------------------
	double yawR1, yawR2;
	geometry_msgs::Pose wp1, wp2;
	std::string group_1="r1_arm", group_2="r2_arm";
	std::string tip_1="r1_ee", tip_2="r2_ee";
	std::vector<double> heightVec;

	heightVec.push_back(TEST_TRAJECTORY_HEIGHT_MAX + ADD_HEIGHT_TEST);
	heightVec.push_back(TEST_TRAJECTORY_HEIGHT_MIN + ADD_HEIGHT_TEST);
	heightVec.push_back(PROBABLY_HEIGHT + ADD_HEIGHT_TEST);
	heightVec.push_back(START_STOP_HEIGHT + ADD_HEIGHT_TEST);

	bool thisWaypoint = false;

	for(int m=0; m<heightVec.size(); m++){		
		for (int k=0; k<waypoints_1.size(); k++) 
		{
			thisWaypoint = false;

			wp1.position = waypoints_1[k];			
			wp1.position.z = heightVec[m];
			wp2.position = waypoints_2[k];
			wp2.position.z = heightVec[m];

			robot_state::RobotState rs(*crc_.getCurrentState());
			Eigen::Affine3d e, e_table, e_target, e_tmp;

			e_table = rs.getFrameTransform(table_frame_);
			e_target = rs.getFrameTransform("base_link");

			tf::poseMsgToEigen(wp1, e);
			e_tmp = e_target.inverse() * e_table * e;
			tf::poseEigenToMsg(e_tmp, wp1);

			tf::poseMsgToEigen(wp2, e);
			e_tmp = e_target.inverse() * e_table * e;
			tf::poseEigenToMsg(e_tmp, wp2);


			for (int i=0; i< ANGLE_NUMBER_STEP; i++) 
			{
				if(thisWaypoint){
					break;
				}
				yawR1 = mod2pi(2*M_PI + i*(2*M_PI/ANGLE_NUMBER_STEP));

				for (int j=0; j< ANGLE_NUMBER_STEP; j++) 
				{		
					if(thisWaypoint) {
						break;
					}
					yawR2 = mod2pi(2*M_PI + j*(2*M_PI/ANGLE_NUMBER_STEP));

					wp1.orientation = transformQuaternion(table_frame_, ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
					wp2.orientation = transformQuaternion(table_frame_, ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);

					if(rs.setFromIK (rs.getJointModelGroup(group_1), wp1, tip_1)){
						if(rs.setFromIK (rs.getJointModelGroup(group_2), wp2, tip_2)){
							thisWaypoint = true;
							break;
						}
					}

					if(rs.setFromIK (rs.getJointModelGroup(group_1), wp2, tip_1)){
						if(rs.setFromIK (rs.getJointModelGroup(group_2), wp1, tip_2)){
							thisWaypoint = true;
							break;
						}
					}
				}
			}
			if(!thisWaypoint){
				return false;
			}
		}
	}

	return thisWaypoint;
}

void ScrollGarment::getLowerPositions(std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, ScrollGarmentSinglePathConfig& singlePathConfig){
	//-------------------------------------------------------------------------------------------

	std::vector<double> height;
	height.push_back(TEST_TRAJECTORY_HEIGHT_MIN + ADD_HEIGHT_TEST);
	getPositions(wp1, wp2, singlePathConfig.short_waypoints_1_, singlePathConfig.short_waypoints_2_, singlePathConfig.yawR1_, singlePathConfig.yawR2_, height, true);
}

void ScrollGarment::getUpperPositions(std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, ScrollGarmentSinglePathConfig& singlePathConfig){
	//-------------------------------------------------------------------------------------------

	std::vector<double> height;
	height.push_back(TEST_TRAJECTORY_HEIGHT_MAX + ADD_HEIGHT_TEST);
	getPositions(wp1, wp2, singlePathConfig.short_waypoints_1_, singlePathConfig.short_waypoints_2_, singlePathConfig.yawR1_, singlePathConfig.yawR2_, height, true);
}

void ScrollGarment::getCombinePositions(std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, ScrollGarmentSinglePathConfig& singlePathConfig){
	//-------------------------------------------------------------------------------------------

	std::vector<double> height;
	height.push_back(TEST_TRAJECTORY_HEIGHT_MAX + ADD_HEIGHT_TEST);
	height.push_back(TEST_TRAJECTORY_HEIGHT_MIN + ADD_HEIGHT_TEST);
	getPositions(wp1, wp2, singlePathConfig.short_waypoints_1_, singlePathConfig.short_waypoints_2_, singlePathConfig.yawR1_, singlePathConfig.yawR2_, height, true);
}

void ScrollGarment::getClearCentralPositions(std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, ScrollGarmentSinglePathConfig& singlePathConfig){
	//-------------------------------------------------------------------------------------------

	std::vector<double> height;
	height.push_back((TEST_TRAJECTORY_HEIGHT_MAX + TEST_TRAJECTORY_HEIGHT_MIN)/2 + ADD_HEIGHT_TEST);
	getPositions(wp1, wp2, singlePathConfig.short_waypoints_1_, singlePathConfig.short_waypoints_2_, singlePathConfig.yawR1_, singlePathConfig.yawR2_, height, false);
}

void ScrollGarment::getPositions(std::vector<geometry_msgs::Pose>& wp1,std::vector<geometry_msgs::Pose>& wp2, const std::vector< geometry_msgs::Point >& waypoints_1, const	std::vector< geometry_msgs::Point >& waypoints_2, double yawR1, double yawR2, const std::vector<double> height, bool startStop) {
	//-------------------------------------------------------------------------------------------

	geometry_msgs::Pose blank;
	int i = 0;

	wp1.erase(wp1.begin(),wp1.end());
	wp2.erase(wp2.begin(),wp2.end());

	if (waypoints_1.size() != waypoints_2.size()){
		ROS_ERROR("Waypoints have different number of points.");
	}

	blank.orientation = transformQuaternion(table_frame_ ,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);

	if (startStop){
		blank.position = waypoints_1.front();
		blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;		
		wp1.push_back(blank);
	}

	for(int h = 0; h < height.size(); h++) {
		for (i = 0; i < waypoints_1.size(); i++){
			blank.position = waypoints_1[i];
			blank.position.z = height[h];
			wp1.push_back(blank);
		}

		if (startStop){
			blank.position = waypoints_1.back();
			blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;
			wp1.push_back(blank);
		}

	}

	blank.orientation = transformQuaternion(table_frame_ ,ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);

	if (startStop){
		blank.position = waypoints_2.front();
		blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;		
		wp2.push_back(blank);
	}

	for(int h = 0; h < height.size(); h++) {		
		for (i = 0; i < waypoints_2.size(); i++){
			blank.position = waypoints_2[i];
			blank.position.z = height[h];
			wp2.push_back(blank);
		}

		if (startStop){
			blank.position = waypoints_2.back();
			blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;
			wp2.push_back(blank);	
		}	
	}	
}


bool ScrollGarment::startStopPosition(ScrollGarmentSinglePathConfig& singlePathConfig, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, bool start) {
	//------------------------------------------------------------------------
	return startStopPosition(table_frame_, singlePathConfig.yawR1_, singlePathConfig.yawR2_, singlePathConfig.short_waypoints_1_, singlePathConfig.short_waypoints_2_, elinks1, elinks2, singlePathConfig.configuration_, start);
}

int ScrollGarment::pressOnTheTable(ScrollGarmentSinglePathConfig& singlePathConfig, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, double force) {	
	//----------------------------------------------------------------------------
	return pressOnTheTable(table_frame_, singlePathConfig.yawR1_, singlePathConfig.yawR2_, singlePathConfig.short_waypoints_1_, singlePathConfig.short_waypoints_2_, elinks1, elinks2, singlePathConfig.configuration_, force);
}

bool ScrollGarment::scrollOverTable(ScrollGarmentSinglePathConfig& singlePathConfig, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, double force) {
	//----------------------------------------------------------------------------
	return scrollOverTable(table_frame_, singlePathConfig.yawR1_, singlePathConfig.yawR2_, singlePathConfig.short_waypoints_1_, singlePathConfig.short_waypoints_2_, elinks1, elinks2, singlePathConfig.configuration_, force);
}

bool ScrollGarment::findBiggestTrajectory(ScrollGarmentSinglePathConfig& currentPathConfig, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2){
//-------------------------------------------------------------------------------------------

	bool ready = false;
	double current_yawR1=0, current_yawR2=0;

	ScrollGarmentSinglePathConfig biggestSinglePathConfig;

	while(!ready){
		mutex_ready_g_.lock();
		ready = ready_g_;
		mutex_ready_g_.unlock();

		mutex_cur_yawR1_.lock();
		current_yawR1 = cur_yawR1_;
		mutex_cur_yawR1_.unlock();

		mutex_cur_yawR2_.lock();
		current_yawR2 = cur_yawR2_;
		mutex_cur_yawR2_.unlock();
	}

	std::vector<geometry_msgs::Pose> wp1, wp2;
	double offset = 0;
	double angle_number_step = ANGLE_NUMBER_STEP;

	// double ext_axis_yaw;
	// mutex_ext_axis_yaw_.lock();
	// ext_axis_yaw = ext_axis_yaw_;
	// mutex_ext_axis_yaw_.unlock();

	bool current_state = false;

	for (int i=0; i< angle_number_step; i++) {
		currentPathConfig.yawR1_ = mod2pi(2*M_PI + current_yawR1 + i*(2*M_PI/angle_number_step));

		for (int j=0; j< angle_number_step; j++) {
			currentPathConfig.yawR2_ = mod2pi(2*M_PI + current_yawR2 + j*(2*M_PI/angle_number_step));
			bool isBiggest = false;

			moveit_msgs::RobotTrajectory trajectories;

			currentPathConfig.configuration_ = true;

			// std::cout << currentPathConfig.yawR1_ << "\t" << currentPathConfig.yawR2_ << std::endl;

			do{
				getLowerPositions(wp1, wp2, currentPathConfig);
				double d = testPathConfig(trajectories, elinks1, elinks2, wp1, wp2, currentPathConfig.configuration_, TEST_STEP, current_state, biggestSinglePathConfig.getPercentage(), isBiggest);
				if(isBiggest) {				
					getUpperPositions(wp1, wp2, currentPathConfig);
					testPathConfig(trajectories, elinks1, elinks2, wp1, wp2, currentPathConfig.configuration_, TEST_STEP, current_state, biggestSinglePathConfig.getPercentage(), isBiggest);
					if(isBiggest) {
						getLowerPositions(wp1, wp2, currentPathConfig);
						currentPathConfig.lowerTrajPercentage_ = testPathConfig(trajectories, elinks1, elinks2, wp1, wp2, currentPathConfig.configuration_, STEP, current_state, biggestSinglePathConfig.getPercentage(), isBiggest);
						if(isBiggest) {
							getUpperPositions(wp1, wp2, currentPathConfig);
							currentPathConfig.upperTrajPercentage_ = testPathConfig(trajectories, elinks1, elinks2, wp1, wp2, currentPathConfig.configuration_, STEP, current_state, biggestSinglePathConfig.getPercentage(), isBiggest);
							if(isBiggest) {
								
								biggestSinglePathConfig = currentPathConfig;								
								
								std::cout << "Best:" << biggestSinglePathConfig.getPercentage() << std::endl;
								// std::cout << biggestSinglePathConfig.yawR1_ << "\t" << biggestSinglePathConfig.yawR2_ << "\t" << biggestSinglePathConfig.configuration_ << "\t" << wp1.size() << "\t" << wp2.size() <<std::endl;	
							}
						}
					}
				}		

				if(std::abs( biggestSinglePathConfig.getPercentage()-1) < 0.001 ){
					return true;
				}

				if(currentPathConfig.configuration_){
					currentPathConfig.configuration_ = false;
				} else {
					currentPathConfig.configuration_ = true;
				}
			} while(!currentPathConfig.configuration_);
		}
	}


	// 	IF NOT RETURN WITH TRUE => TRAJECTORY IS NOT COMPLETE
	// 		FIND SHORTER TRAJECTORY (FORM UPPER/LOWER) 
	// 		AND FROM LAST (DEPEND ON CONSTATNT "MAX_TRAJ_2_MAKE_WAYPOINT") 
	// 		POINT OF TRAJECTORY MAKE ANOTHER WAYPOINT 
	{

		if (biggestSinglePathConfig.getLPer() > biggestSinglePathConfig.getUPer()) {
		// TRAJ FROM UPPER
			getUpperPositions(wp1, wp2, biggestSinglePathConfig);
		} else {
		// TRAJ FORM LOWER
			getLowerPositions(wp1, wp2, biggestSinglePathConfig);
		}

		bool isBiggest = false;
		moveit_msgs::RobotTrajectory finalTraj;
		testPathConfig(finalTraj, elinks1, elinks2, wp1, wp2, biggestSinglePathConfig.configuration_, STEP, current_state, 0.0, isBiggest);

		sensor_msgs::JointState endPoint;
		endPoint.header = finalTraj.joint_trajectory.header;

		int sizeOfTraj = finalTraj.joint_trajectory.points.size();
		int posOfEnd = std::floor(sizeOfTraj * (MAX_TRAJ_2_MAKE_ENDPOINT - sub_traj_));

		endPoint.name = finalTraj.joint_trajectory.joint_names;
		endPoint.position = finalTraj.joint_trajectory.points[posOfEnd].positions;
		endPoint.velocity = finalTraj.joint_trajectory.points[posOfEnd].velocities;
		endPoint.effort = finalTraj.joint_trajectory.points[posOfEnd].effort;

						// MAY ERROR - NOT SURE IF RS COPY EXT.AXIS FROM CURRENT
		robot_state::RobotState rs(*crc_.getCurrentState());	
		rs.setVariableValues(endPoint);

		Eigen::Affine3d e_target, e_tmp, e_r1_ee, e_r2_ee, e_table, e_base;

		e_table = rs.getFrameTransform(table_frame_);
		e_r1_ee = rs.getFrameTransform("r1_ee");
		e_r2_ee = rs.getFrameTransform("r2_ee");

		geometry_msgs::Point p1, p2;
		e_tmp = e_table.inverse() * e_r1_ee;
		// std::cout << "\033[1;36m" << e_tmp.matrix() << "\033[0m"<< std::endl;
		p1.x=e_tmp(0,3);
		p1.y=e_tmp(1,3);
		e_tmp = e_table.inverse() * e_r2_ee;
		// std::cout << "\033[1;36m" << e_tmp.matrix() << "\033[0m"<< std::endl;
		p2.x=e_tmp(0,3);
		p2.y=e_tmp(1,3);


		biggestSinglePathConfig.short_waypoints_1_.erase(biggestSinglePathConfig.short_waypoints_1_.end());
		biggestSinglePathConfig.short_waypoints_2_.erase(biggestSinglePathConfig.short_waypoints_2_.end());
		if(biggestSinglePathConfig.configuration_){
			// CHANGE COMPUTED POINT WITH END POINT IN WAYPOINTS
			biggestSinglePathConfig.short_waypoints_1_.push_back(p1);
			biggestSinglePathConfig.short_waypoints_2_.push_back(p2);
		} else {
			// CHANGE COMPUTED POINT WITH END POINT IN WAYPOINTS (second configuration)
			biggestSinglePathConfig.short_waypoints_1_.push_back(p2);
			biggestSinglePathConfig.short_waypoints_2_.push_back(p1);
		}

	}

	currentPathConfig = biggestSinglePathConfig;
	return false;
}


// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 
// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 
// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 
// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 
// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 
// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 
// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 
// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 
// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 
// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 
// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 
// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 
// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 
// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 
// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 
// GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE GARBAGE 


// bool ScrollGarment::goToStartStopPosition(const std::vector<geometry_msgs::Pose>& wp1, const std::vector<geometry_msgs::Pose>& wp2, const bool first_combination){
// 	//------------------------------------------------------------------------

// 	std::string tip_1 = "r1_ee", tip_2 = "r2_ee", conf = "Configuration 1: ", group_1 = "r1_arm", group_2 = "r2_arm";
// 	if(!first_combination) {
// 		tip_1 = "r2_ee", tip_2 = "r1_ee", conf = "Configuration 2: ", group_1 = "r2_arm", group_2 = "r1_arm";
// 	}

// 	std::cout << wp1[0] << std::endl;
// 	std::cout << wp2[0] << std::endl;

// 	geometry_msgs::Pose pose_tmp;

// 	robot_state::RobotState rs(*crc_.getCurrentState());

// 	std::vector<geometry_msgs::Pose> wp1_copy=wp1, wp2_copy=wp2;
// 	Eigen::Affine3d e, e_table, e_target, e_tmp;

// 	e_table = rs.getFrameTransform(table_frame_);
// 	e_target = rs.getFrameTransform(BASE_TARGET);

// 	pose_tmp = wp1_copy.front();
// 	tf::poseMsgToEigen(pose_tmp, e);
// 	e_tmp = e_target.inverse() * e_table * e;
// 	tf::poseEigenToMsg(e_tmp, pose_tmp);

// 	if(!rs.setFromIK (rs.getJointModelGroup(group_1), pose_tmp, tip_1)){
// 		ROS_ERROR("Error - setFromIK 1 ");
// 		return false;
// 	}

// 	std::cout << pose_tmp << std::endl;

// 	pose_tmp = wp2_copy.front();
// 	tf::poseMsgToEigen(pose_tmp, e);
// 	e_tmp = e_target.inverse() * e_table * e;
// 	tf::poseEigenToMsg(e_tmp, pose_tmp);

// 	if(!rs.setFromIK (rs.getJointModelGroup(group_2), pose_tmp, tip_2)){
// 		ROS_ERROR("Error - setFromIK 2 ");
// 		return false;
// 	}

// 	std::cout << pose_tmp << std::endl;

// 	crc_.setJointValueTarget(rs);
// 	return crc_.move();

// }