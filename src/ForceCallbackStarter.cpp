#include "ForceCallbackStarter.h"

ForceCallbackStarter::ForceCallbackStarter(): crc_("arms"){

	sub_joints_ = node_.subscribe("/joint_states", 1, &ForceCallbackStarter::cb_joint, this);
	// WrenchR1_.startSub("r1");
	// WrenchR2_.startSub("r2");
	// WrenchR1_.setInitialize();
	// WrenchR2_.setInitialize();	
}

void ForceCallbackStarter::cb_joint(const sensor_msgs::JointState& msg) {

	robot_state::RobotState rs_link(*crc_.getCurrentState());	
	// mutex_msg_.lock();
	// rs_link.setVariableValues(msg);
	// mutex_msg_.unlock();

	// link_r1_force_sensor_ = rs_link.getGlobalLinkTransform ("r1_force_sensor").inverse();
	// WrenchR1_.setRotMat(link_r1_force_sensor_);

	// link_r2_force_sensor_ = rs_link.getGlobalLinkTransform ("r2_force_sensor").inverse();
	// WrenchR2_.setRotMat(link_r2_force_sensor_);
}

// bool ForceCallbackStarter::joints_to_point(const sensor_msgs::JointState& js, trajectory_msgs::JointTrajectoryPoint& p)  {	
// 	std::vector<std::string> jns = get_joint_names();

// 	for(auto jn : jns) {
// 		auto pos = find(js.name.begin(), js.name.end(), jn);
// 		if(pos == js.name.end())
// 			return false;		
// 		auto d = distance(js.name.begin(), pos);
// 		p.positions.push_back(js.position[d]);
// 		p.velocities.push_back(0.0);
// 		p.accelerations.push_back(0.0);
// 		p.effort.push_back(0.0);
// 	}
// 	return true;
// }

// std::vector<std::string> ForceCallbackStarter::get_joint_names()  {
// 	std::vector<std::string> joint_names;
// 	joint_names.push_back("r1_joint_s");
// 	joint_names.push_back("r1_joint_l");
// 	joint_names.push_back("r1_joint_u");
// 	joint_names.push_back("r1_joint_r");
// 	joint_names.push_back("r1_joint_b");
// 	joint_names.push_back("r1_joint_t");
// 	joint_names.push_back("r2_joint_s");
// 	joint_names.push_back("r2_joint_l");
// 	joint_names.push_back("r2_joint_u");
// 	joint_names.push_back("r2_joint_r");
// 	joint_names.push_back("r2_joint_b");
// 	joint_names.push_back("r2_joint_t");
// 	joint_names.push_back("ext_axis");
// 	return joint_names;
// }

// bool ForceCallbackStarter::get_current_point(trajectory_msgs::JointTrajectoryPoint& p) {
// 	bool rtn = true;
// 	mutex_joints_.lock();
// 	rtn = joints_to_point(joints_, p);
// 	mutex_joints_.unlock();
// 	return rtn;
// }

// void ForceCallbackStarter::add_current_state_to_trajectory(trajectory_msgs::JointTrajectory& trajectory){
// 	trajectory_msgs::JointTrajectory oldTrajectory = trajectory;
// 	trajectory_msgs::JointTrajectoryPoint p;

// 	get_current_point(p);
// 	std::vector<std::string> joint_names = get_joint_names();

// 	trajectory.joint_names.clear();
// 	trajectory.points.clear();

// 	// trajectory.points.push_back(p);
	

// 	robot_state::RobotState rs(*crc_.getCurrentState());
// 	crc_.setStartState(rs);
// 	crc_.setStartStateToCurrentState();

// 	BOOST_FOREACH(const std::string & jn, joint_names) {
// 		trajectory.joint_names.push_back(jn);
// 	}

// 	BOOST_FOREACH(const trajectory_msgs::JointTrajectoryPoint & point, oldTrajectory.points) {
// 		for(unsigned int i = 0 ; i < oldTrajectory.joint_names.size(); ++i) {
// 			std::string jn = oldTrajectory.joint_names[i];
// 			std::vector<std::string>::iterator it;
// 			it = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), jn);
// 			unsigned int ind = std::distance(trajectory.joint_names.begin(), it);
// 			p.positions[ind] = point.positions[i];
// 		}

// 		p.time_from_start = point.time_from_start;
// 		trajectory.points.push_back(p);		
// 	}	
// }