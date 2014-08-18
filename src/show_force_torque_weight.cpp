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


#define X 0
#define Y 1
#define Z 2

void showPose(geometry_msgs::Pose pose) {
	char buffer [100];
	sprintf(buffer, "%.3f\t%.3f\t%.4f\t%.2f\t%.2f\t%.2f\t%f", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	ROS_INFO_STREAM(buffer);
}

geometry_msgs::Pose getDiffR2ee(std::string name){
	
	ros::Rate rate(10.0);
	geometry_msgs::Pose pos;
	tf::TransformListener listener;
	tf::StampedTransform transform;

	while (ros::ok()){
		try{
			ros::Time latest_transform_time;
			listener.getLatestCommonTime(name,"/r2_ee", latest_transform_time, NULL);
			listener.waitForTransform(name,"/r2_ee", latest_transform_time, ros::Duration(0.1) );
			listener.lookupTransform(name, "/r2_ee",  
				latest_transform_time, transform);
			pos.position.x=transform.getOrigin().x();
			pos.position.y=transform.getOrigin().y();
			pos.position.z=transform.getOrigin().z();
			pos.orientation.x=transform.getRotation().x();
			pos.orientation.y=transform.getRotation().y();
			pos.orientation.z=transform.getRotation().z();
			pos.orientation.w=transform.getRotation().w();
			// showPose(pos);
			break;
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		rate.sleep();
	}
	return pos;
}

Eigen::Vector3d getAngles(geometry_msgs::Pose pos){
	tf::Quaternion q(pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w);
	tf::Matrix3x3 m(q);
	Eigen::Vector3d rpy;
	m.getRPY(rpy(0), rpy(1), rpy(2));
	return rpy;
}

void force_info_stream(geometry_msgs::WrenchStamped msg) {
	char buffer [100];
	sprintf(buffer, "%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f", msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z);
	ROS_INFO_STREAM(buffer);
}


class Listener
{
public:
	Eigen::Vector3d rpy;
	geometry_msgs::WrenchStamped initialForce;
	double lenght;

public:
	Listener();
	void callback(geometry_msgs::WrenchStamped forceMsgs);
	geometry_msgs::WrenchStamped computeForceOutputEE (geometry_msgs::WrenchStamped forceMsgs);
};

Listener::Listener(){
	geometry_msgs::Pose pos;
	pos = getDiffR2ee("/r2_force_sensor");
	
	initialForce.header.frame_id = "init";
	lenght = pos.position.z;
	rpy = getAngles(pos);		
	
}

void getRotateForce(Eigen::Vector3d & force, Eigen::Vector3d rpy){
	Eigen::Matrix3d rotMat;

	rotMat =  Eigen::AngleAxisd(-rpy(0), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-rpy(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-rpy(2), Eigen::Vector3d::UnitZ());

	force = force.transpose() * rotMat;

	std::cout << "\n" << force << "\n\n";
}

geometry_msgs::WrenchStamped Listener::computeForceOutputEE (geometry_msgs::WrenchStamped forceMsgs)
{
	Eigen::Matrix3d newMat;
	ROS_INFO_STREAM(newMat);

	geometry_msgs::WrenchStamped computeForceEE;
	Eigen::Vector3d forceX(0,0,0), forceY(0,0,0), forceZ(0,0,0), force(0,0,0);
	Eigen::Vector3d torqueX(0,0,0), torqueY(0,0,0), torqueZ(0,0,0), torque(0,0,0);

	forceX(X) = forceMsgs.wrench.force.x - initialForce.wrench.force.x;
	forceY(Y) = forceMsgs.wrench.force.y - initialForce.wrench.force.y;
	forceZ(Z) = forceMsgs.wrench.force.z - initialForce.wrench.force.z;
	getRotateForce(forceX, rpy);
	getRotateForce(forceY, rpy);
	getRotateForce(forceZ, rpy);

	torqueX(X) = forceMsgs.wrench.torque.x - initialForce.wrench.torque.x;
	torqueY(Y) = forceMsgs.wrench.torque.y - initialForce.wrench.torque.y;
	torqueZ(Z) = forceMsgs.wrench.torque.z - initialForce.wrench.torque.z;
	getRotateForce(torqueX, rpy);
	getRotateForce(torqueY, rpy);
	getRotateForce(torqueZ, rpy);

	for(int i=0; i<3; i++){
		force(i)=forceX(i)+forceY(i)+forceZ(i);
		torque(i)=torqueX(i)+torqueY(i)+torqueZ(i);
	}

	computeForceEE.wrench.force.x = force(X);
	computeForceEE.wrench.force.y = force(Y);
	computeForceEE.wrench.force.z = force(Z);
	computeForceEE.wrench.torque.x = torque(X);
	computeForceEE.wrench.torque.y = torque(Y);
	computeForceEE.wrench.torque.z = torque(Z);
	return computeForceEE;
}

void Listener::callback(geometry_msgs::WrenchStamped forceMsgs){
	
	if(0==initialForce.header.frame_id.compare("init"))
		initialForce=forceMsgs;

	geometry_msgs::WrenchStamped computeForceEE;
	computeForceEE = computeForceOutputEE (forceMsgs);

	force_info_stream(computeForceEE);
	ROS_INFO_STREAM(std::abs(computeForceEE.wrench.force.z));
	ROS_INFO_STREAM(std::abs(computeForceEE.wrench.torque.x));

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pos_r2ee_r2force");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	Listener listener;
	ros::Subscriber sub = node.subscribe<geometry_msgs::WrenchStamped>("/r2_force_data_filtered", 1000, &Listener::callback, &listener);	

	ros::spin();
	return 0;
}