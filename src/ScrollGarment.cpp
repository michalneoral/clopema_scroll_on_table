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
			/*mutex_z_r1_.lock();
			std::cout << "R1 - OK, old z = " << z_r1_ << " force: " << WrenchR1_.getForce() << std::endl << std::endl;
			mutex_z_r1_.unlock();*/
		}else{
			isOk = false;
			mutex_z_r1_.lock();
			blank.position.z = z_r1_ + (WrenchR1_.getForce() - force) * FORCE_CONST;
			/*std::cout << "R1 - NOT OK, old z = " << z_r1_ << " new z = " << blank.position.z << " force: " << WrenchR1_.getForce() << std::endl << std::endl;*/
			mutex_z_r1_.unlock();				
		}		
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
		wp1.push_back(blank);

		blank.position = waypoints_2[0];		
		if(!WrenchR1_.isForceOk(force)){			
			isOk = false;
			mutex_z_r2_.lock();
			blank.position.z = z_r2_ + (WrenchR2_.getForce() - force) * FORCE_CONST;
			/*std::cout << "R2 - NOT OK, old z = " << z_r2_ << " new z = " << blank.position.z << " force: " << WrenchR2_.getForce() << std::endl << std::endl;*/
			mutex_z_r2_.unlock();
		}		
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
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

bool ScrollGarment::scrollOverTable(std::string table_frame , const double& yawR1, const double& yawR2, const std::vector< geometry_msgs::Point >& waypoints_1,	const std::vector< geometry_msgs::Point >& waypoints_2, const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2, bool conf, double force) {	//----------------------------------------------------------------------------

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
			std::cout << "R1: old z= " << z_r1_ << " new z= " << blank.position.z << " force: " << WrenchR1_.getForce() << std::endl;
			mutex_z_r1_.unlock();
			blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
			wp1.push_back(blank);

			blank.position = waypoints_2[i];
			mutex_z_r2_.lock();
			blank.position.z = z_r2_ + (WrenchR2_.getForce() - force) * FORCE_CONST;
			std::cout << "R2: old z= " << z_r2_ << " new z= " << blank.position.z << " force: " << WrenchR2_.getForce() << std::endl;
			mutex_z_r2_.unlock();
			blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
			wp2.push_back(blank);


			if(planPoses(trajectory, elinks1, elinks2, wp1, wp2, conf, STEP, true)){
				trajectory.joint_trajectory.points.erase(trajectory.joint_trajectory.points.begin());
				std::cout << "[scrollOverTable] Trajectory size: " << trajectory.joint_trajectory.points.size() << std::endl;
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
	geometry_msgs::Pose blank;
	// Prepare poses 
	if(start){
		blank.position = waypoints_1[0];
		blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R1, PITCH2DESK_R1, YAW2DESK_R1 + yawR1);
		wp1.push_back(blank);
		blank.position.z = OVER_TABLE_HEIGHT + ADD_HEIGHT_TEST;
		wp1.push_back(blank);

		blank.position = waypoints_2[0];
		blank.position.z = START_STOP_HEIGHT + ADD_HEIGHT_TEST;
		blank.orientation = transformQuaternion(table_frame,ROLL2DESK_R2, PITCH2DESK_R2, YAW2DESK_R2 + yawR2);
		
		wp2.push_back(blank);
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

	std::cout << "wp1: " << wp1.size() << " wp2: " << wp2.size() << std::endl;

	if(planPoses(trajectory, elinks1, elinks2, wp1, wp2, conf, STEP, true)){
		trajectory.joint_trajectory.points.erase(trajectory.joint_trajectory.points.begin());
		add_current_state_to_trajectory(trajectory.joint_trajectory);
		pub_traj_.controle(trajectory.joint_trajectory);
		return crc_.execute_traj(trajectory.joint_trajectory);
	}else{
		ROS_ERROR("Cannot interpolate");
		return false;
	}
}

bool ScrollGarment::planPoses(moveit_msgs::RobotTrajectory &trajectories, const std::vector<std::string> elinks1, const std::vector<std::string> elinks2, const std::vector<geometry_msgs::Pose>& wp1, const std::vector<geometry_msgs::Pose>& wp2, const bool first_combination, double step, bool current_state){
	//------------------------------------------------------------------------

	char buffer [100];

	showMarkers(table_frame_, wp1, wp2);

	std::string tip_1 = "r1_ee", tip_2 = "r2_ee", conf = "Configuration 1: ", group_1 = "r1_arm", group_2 = "r2_arm";
	if(!first_combination) {
		tip_1 = "r2_ee", tip_2 = "r1_ee", conf = "Configuration 2: ", group_1 = "r2_arm", group_2 = "r1_arm";
	}

	double d;
	int max=16;

	robot_state::RobotState rs(*crc_.getCurrentState());

	std::vector<geometry_msgs::Pose> wp1_copy=wp1, wp2_copy=wp2;
	// for (int i=0; i<max; i++){
	// 	std::cout << rs.getVariablePositions()[i] << "  ";
	// }
	// std::cout << std::endl;

	if (!current_state){
		geometry_msgs::Pose pose_tmp;
		crc_.setPoseReferenceFrame("base_link");

		robot_state::RobotState rs(*crc_.getCurrentState());
		Eigen::Affine3d e, e_table, e_target, e_tmp;

		e_table = rs.getFrameTransform(table_frame_);
		e_target = rs.getFrameTransform("base_link");

		pose_tmp = wp1_copy.front();
		tf::poseMsgToEigen(pose_tmp, e);
		e_tmp = e_target.inverse() * e_table * e;
		tf::poseEigenToMsg(e, pose_tmp);

		if(!rs.setFromIK (rs.getJointModelGroup(group_1), pose_tmp, tip_1)){
			ROS_ERROR("Error - setFromIK 1 ");
			return false;
		}

		pose_tmp = wp2_copy.front();
		tf::poseMsgToEigen(pose_tmp, e);
		e_tmp = e_target.inverse() * e_table * e;
		tf::poseEigenToMsg(e, pose_tmp);

		if(!rs.setFromIK (rs.getJointModelGroup(group_2), pose_tmp, tip_2)){
			ROS_ERROR("Error - setFromIK 2 ");
			return false;
		}

		crc_.setPoseReferenceFrame(table_frame_);

		wp1_copy.erase(wp1_copy.begin());
		wp2_copy.erase(wp2_copy.begin());
		// ROS_WARN_STREAM("WAS ERASED");
	}
	crc_.setStartState(rs);
	// for (int i=0; i<max; i++){
	// 	std::cout << rs.getVariablePositions()[i] << "  ";
	// }
	// std::cout << std::endl << "-----------------------------------------------" << std::endl;
	
	d = crc_.computeCartesianPathDual(wp1_copy, tip_1, wp2_copy, tip_2, step, JUMP_TRESHOLD, trajectories, false);

	if(!(fabs(d - 1.0) < 0.001)) {
		sprintf(buffer, "cannot interpolate up. d = %.4f (%.4f).", d, fabs(d - 1.0));
		ROS_WARN_STREAM(buffer);
		// std::cout << "wp1: " << wp1.size() << " wp2: " << wp2.size() << " wp1_copy: " << wp1_copy.size() << " wp2_copy: " << wp2_copy.size() << std::endl;
		return false;
	} else{
		if(!crc_.check_trajectory(trajectories, elinks1, elinks2)) {
			sprintf(buffer, "cannot interpolate up because of collision");
			ROS_WARN_STREAM(buffer);
			return false;
		}
		else{
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

		std::cout << "wp1: " << waypoints_1.size() << " wp2: " << waypoints_2.size() << std::endl;

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