#include <ros/ros.h>
#include <clopema_robot/robot_commander.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <string>

#define START_HEIGHT 0.78
#define M_TO_MM 1000

class ForceSimulator {

public:
	ForceSimulator(): crc_("arms"){
		seq_=1;

		sub_joints_ = node_.subscribe("/joint_states", 1, &ForceSimulator::cb_joint, this);

		pub_force_R1_ = node_.advertise<geometry_msgs::WrenchStamped>("/r1_force_data_filtered", 1);
		ros::Duration(0.2).sleep();			
		pub_force_R2_ = node_.advertise<geometry_msgs::WrenchStamped>("/r2_force_data_filtered", 1);
		ros::Duration(0.2).sleep();

		ROS_INFO_STREAM("Simulator started");
	}
	void pub();
	void cb_joint(const sensor_msgs::JointState& msg);
	void force_info_stream(geometry_msgs::WrenchStamped msg);
	double computeSingle (double z, double a2, double a1, double a0);
	void computeForceR1 (geometry_msgs::WrenchStamped& simForce);
	void computeForceR2 (geometry_msgs::WrenchStamped& simForce);


public:
	ros::Subscriber sub_joints_;
	ros::Publisher pub_force_R1_;
	ros::Publisher pub_force_R2_;
	double seq_;

	clopema_robot::ClopemaRobotCommander crc_;

	ros::NodeHandle node_;
	double z_r1_;
	double z_r2_;
};


void ForceSimulator::cb_joint(const sensor_msgs::JointState& msg) {
	robot_state::RobotState rs_link(*crc_.getCurrentState());
	rs_link.setVariableValues(msg);

	Eigen::Affine3d link_r1_ee, link_r2_ee;

	link_r1_ee = rs_link.getGlobalLinkTransform ("r1_ee");
	link_r2_ee = rs_link.getGlobalLinkTransform ("r2_ee");

	z_r1_ = link_r1_ee(2,3);
	z_r2_ = link_r2_ee(2,3);
}

void ForceSimulator::force_info_stream(geometry_msgs::WrenchStamped msg) {
	char buffer [100];
	sprintf(buffer, "%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f", msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z);
	ROS_INFO_STREAM(buffer);
}


double ForceSimulator::computeSingle (double z, double a2, double a1, double a0){
	double force=0;
	if (z>START_HEIGHT)
		{z=0;}
	else {z=(START_HEIGHT-z)*M_TO_MM;}
	force += pow(z,2) * (a2);
	force += z * (a1);
	force += a0;
	return force;
}

void ForceSimulator::computeForceR1 (geometry_msgs::WrenchStamped& simForce){
	simForce.header.seq = seq_;
	double z;
	z = z_r1_;
	ros::Time t = ros::Time::now();
	simForce.header.stamp = t;
	simForce.header.frame_id = "r1_force_data_filtered";
	
	if (z>START_HEIGHT)
		{z=0;}
	else {z=(START_HEIGHT-z)*M_TO_MM;}
	simForce.wrench.force.z = z * -0.64 * 8 + ((double)std::rand()/RAND_MAX * 5);

	std::cout << "[ " << ros::Time::now() << " ] z_r1: " << z << " Force: " << simForce.wrench.force.z << std::endl;
	// simForce.wrench.force.x = computeSingle (z, -0.0138,    1.2562,   -3.2764);
	// simForce.wrench.force.y = computeSingle (z, -0.0290,   -1.4388,    2.2879);
	// simForce.wrench.force.z = computeSingle (z, -0.0082,   -1.3716,   -1.2039);
	// simForce.wrench.torque.x = computeSingle (z, 0.0098,    0.4168,   -0.3085);
	// simForce.wrench.torque.y = computeSingle (z, -0.0016,   0.3460,   -0.5092);
	// simForce.wrench.torque.z = computeSingle (z, -0.0004,   0.0016,    0.0081);
}

void ForceSimulator::computeForceR2 (geometry_msgs::WrenchStamped& simForce){
	simForce.header.seq = seq_;
	double z;
	z = z_r2_;
	ros::Time t = ros::Time::now();
	simForce.header.stamp = t;
	simForce.header.frame_id = "r2_force_data_filtered";
	
	if (z>START_HEIGHT)
		{z=0;}
	else {z=(START_HEIGHT-z)*M_TO_MM;}
	simForce.wrench.force.z = z * -0.64 * 8 + ((double)std::rand()/RAND_MAX * 20);

	std::cout << "[ " << ros::Time::now() << " ] z_r2: " << z << " Force: " << simForce.wrench.force.z <<std::endl;
	// simForce.wrench.force.x = computeSingle (z, 0.0025,    1.3178,   -1.4595);
	// simForce.wrench.force.y = computeSingle (z, -0.0046,   -2.1012,    2.3930);
	// simForce.wrench.force.z = computeSingle (z, -0.0150 ,  -1.2819 ,   1.0194);
	// simForce.wrench.torque.x = computeSingle (z, 0.0033  ,  0.5024  , -0.6238);
	// simForce.wrench.torque.y = computeSingle (z, 0.0014   , 0.3429   ,-0.4181);
	// simForce.wrench.torque.z = computeSingle (z, 0.0001    ,0.0005   ,-0.0025);
}

void ForceSimulator::pub(){
	geometry_msgs::WrenchStamped simForceR1, simForceR2;
	computeForceR1(simForceR1);
	computeForceR2(simForceR2);
	// force_info_stream(simForceR1);
	// force_info_stream(simForceR2);
	pub_force_R1_.publish(simForceR1);
	pub_force_R2_.publish(simForceR2);
	seq_++;
}




int main(int argc, char **argv) {
	ros::init(argc, argv, "r2_force_simulator");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ForceSimulator fs;

	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		fs.pub();
		loop_rate.sleep();
		ros::spinOnce();
	}
	// ros::spin();
	return 0;
}