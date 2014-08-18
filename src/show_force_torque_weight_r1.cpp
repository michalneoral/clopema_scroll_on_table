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


void showPose(geometry_msgs::Pose pose) {
	char buffer [100];
	sprintf(buffer, "%.3f\t%.3f\t%.4f\t%.2f\t%.2f\t%.2f\t%f", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	ROS_INFO_STREAM(buffer);
}

geometry_msgs::Pose getDiffR1ee(std::string name){
	
	ros::Rate rate(10.0);
	geometry_msgs::Pose pos;
	tf::TransformListener listener;
	tf::StampedTransform transform;

	while (ros::ok()){
		try{
			ros::Time latest_transform_time;
			listener.getLatestCommonTime(name,"/r1_ee", latest_transform_time, NULL);
			listener.waitForTransform(name,"/r1_ee", latest_transform_time, ros::Duration(0.1) );
			// listener.lookupTransform(name, "/r1_ee",  
			// 	ros::Time(0), transform);
			listener.lookupTransform(name, "/r1_ee",  
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
			// ROS_ERROR("%s",ex.what());
			// ros::Duration(1.0).sleep();
		}
		rate.sleep();
	}
	return pos;
}

double getAngle(geometry_msgs::Pose pos, int rpy){
	tf::Quaternion q(pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	switch(rpy) {
		case 1 : return roll; break;
		case 2 : return pitch; break;
		case 3 : return yaw; break;
	}
}

void force_info_stream(geometry_msgs::WrenchStamped msg) {
	char buffer [100];
	sprintf(buffer, "%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f", msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z);
	ROS_INFO_STREAM(buffer);
}


class Listener
{
public:
	double length;
	double roll;
	double pitch;
	double initZ;
	double startZ;
	geometry_msgs::WrenchStamped initialForce;

public:
	Listener();
	void callback(geometry_msgs::WrenchStamped forceMsgs);
	geometry_msgs::WrenchStamped computeForceOutputEE (geometry_msgs::WrenchStamped forceMsgs, double phi, double thetaCor);
	double getCorrectedTheta(geometry_msgs::WrenchStamped forceMsgs, double zPos);
};

Listener::Listener(){
	geometry_msgs::Pose posDiff, posR1;
	// posDiff = getDiffR1ee("/r1_force_sensor");
	posDiff = getDiffR1ee("/r1_force_sensor");
	posR1 = getDiffR1ee("/ctu_floor");
	initialForce.header.frame_id = "init";
	length = posDiff.position.z;
	roll = getAngle(posDiff,3);		
	pitch = getAngle(posR1,2);
	initZ = 0;
	startZ = 0;
}

double Listener::getCorrectedTheta(geometry_msgs::WrenchStamped forceMsgs, double zPos){	// 
	double testSum = (std::abs(forceMsgs.wrench.force.x - initialForce.wrench.force.x)+std::abs(forceMsgs.wrench.force.y - initialForce.wrench.force.y )+std::abs(forceMsgs.wrench.force.z - initialForce.wrench.force.z ));
	
	if (initZ==0) {initZ=zPos;}
	if (testSum > 0.1)
	{ 
		if (startZ==0) {startZ=zPos-initZ;}
		return atan((length * sin(pitch)) / ( ((length * sin(pitch)) / tan(pitch)) - (zPos-initZ-startZ)/1000 ) );
	}
	else {
		return pitch;
	}
}

geometry_msgs::WrenchStamped Listener::computeForceOutputEE (geometry_msgs::WrenchStamped forceMsgs, double phi, double thetaCor)
{
	geometry_msgs::WrenchStamped computeForceEE;
/*	computeForceEE.wrench.force.x = (forceMsgs.wrench.force.x - initialForce.wrench.force.x ) / (cos(roll) * sin(pitch));
	computeForceEE.wrench.force.y = (forceMsgs.wrench.force.y - initialForce.wrench.force.y ) / (cos(roll+M_M_PI/2) * sin(pitch-M_PI));
	computeForceEE.wrench.force.z = (forceMsgs.wrench.force.z - initialForce.wrench.force.z ) / (cos(M_PI-pitch));
	computeForceEE.wrench.torque.x = (forceMsgs.wrench.torque.x - initialForce.wrench.torque.x) / (cos(roll+M_M_PI/2) * length * sin(pitch));
	computeForceEE.wrench.torque.y = (forceMsgs.wrench.torque.y - initialForce.wrench.torque.y) / (cos(roll) * length * sin(pitch));
	computeForceEE.wrench.torque.z = 0;
*/	

	double phiB=-phi;
	double thetaCorB=M_PI-thetaCor;

	computeForceEE.wrench.force.x = (forceMsgs.wrench.force.x - initialForce.wrench.force.x ) * sin(thetaCorB) * cos(phiB);
	computeForceEE.wrench.force.y = (forceMsgs.wrench.force.x - initialForce.wrench.force.x ) * sin(thetaCorB) * sin(phiB);
	computeForceEE.wrench.force.z = (forceMsgs.wrench.force.x - initialForce.wrench.force.x ) * cos(thetaCorB);
	computeForceEE.wrench.force.x = computeForceEE.wrench.force.x +  (forceMsgs.wrench.force.y - initialForce.wrench.force.y ) * cos(phiB-M_PI/2) * sin(thetaCorB);
	computeForceEE.wrench.force.y = computeForceEE.wrench.force.y +  (forceMsgs.wrench.force.y - initialForce.wrench.force.y ) * sin(phiB-M_PI/2) * sin(thetaCorB);
	computeForceEE.wrench.force.z = computeForceEE.wrench.force.z +  (forceMsgs.wrench.force.y - initialForce.wrench.force.y ) * cos(thetaCorB);
	computeForceEE.wrench.force.x = computeForceEE.wrench.force.x +  (forceMsgs.wrench.force.z - initialForce.wrench.force.z ) * cos(phiB-M_PI) * sin(-M_PI/2+thetaCorB);
	computeForceEE.wrench.force.y = computeForceEE.wrench.force.y +  (forceMsgs.wrench.force.z - initialForce.wrench.force.z ) * sin(phiB-M_PI) * sin(-M_PI/2+thetaCorB);
	computeForceEE.wrench.force.z = computeForceEE.wrench.force.z +  (forceMsgs.wrench.force.z - initialForce.wrench.force.z ) * cos(-M_PI/2+thetaCorB);

	phiB=phiB-M_PI/2;
	thetaCorB=thetaCorB+M_PI/2;

	computeForceEE.wrench.torque.x = ((forceMsgs.wrench.torque.x - initialForce.wrench.torque.x) * sin(thetaCorB) * cos(phiB)) / length;
	computeForceEE.wrench.torque.y = ((forceMsgs.wrench.torque.x - initialForce.wrench.torque.x) * sin(thetaCorB) * sin(phiB)) / length;
	computeForceEE.wrench.torque.z = ((forceMsgs.wrench.torque.x - initialForce.wrench.torque.x) * cos(thetaCorB)) / length;
	computeForceEE.wrench.torque.x = computeForceEE.wrench.torque.x +  ((forceMsgs.wrench.torque.y - initialForce.wrench.torque.y) * cos(phiB-M_PI/2) * sin(thetaCorB)) / length;
	computeForceEE.wrench.torque.y = computeForceEE.wrench.torque.y +  ((forceMsgs.wrench.torque.y - initialForce.wrench.torque.y) * sin(phiB-M_PI/2) * sin(thetaCorB)) / length;
	computeForceEE.wrench.torque.z = computeForceEE.wrench.torque.z +  ((forceMsgs.wrench.torque.y - initialForce.wrench.torque.y) * cos(thetaCorB)) / length;
	computeForceEE.wrench.torque.x = computeForceEE.wrench.torque.x +  ((forceMsgs.wrench.torque.z - initialForce.wrench.torque.z) * cos(phiB-M_PI) * sin(-M_PI/2+thetaCorB)) / length;
	computeForceEE.wrench.torque.y = computeForceEE.wrench.torque.y +  ((forceMsgs.wrench.torque.z - initialForce.wrench.torque.z) * sin(phiB-M_PI) * sin(-M_PI/2+thetaCorB)) / length;
	computeForceEE.wrench.torque.z = computeForceEE.wrench.torque.z +  ((forceMsgs.wrench.torque.z - initialForce.wrench.torque.z) * cos(-M_PI/2+thetaCorB)) / length;



	return computeForceEE;
}

void Listener::callback(geometry_msgs::WrenchStamped forceMsgs){

	if(0==initialForce.header.frame_id.compare("init"))
		initialForce=forceMsgs;

	geometry_msgs::WrenchStamped computeForceEE;

	geometry_msgs::Pose posR1;
	posR1 = getDiffR1ee("/ctu_floor");
	double zPos = posR1.position.z; 
	double CorTheta = getCorrectedTheta(forceMsgs, zPos);

	computeForceEE = computeForceOutputEE (forceMsgs, roll, CorTheta);

	ROS_INFO_STREAM("===========================");
	force_info_stream(forceMsgs);
	force_info_stream(computeForceEE);
	ROS_INFO_STREAM(std::abs(computeForceEE.wrench.force.y));
	ROS_INFO_STREAM(std::abs(computeForceEE.wrench.torque.z));
	ROS_INFO_STREAM(std::abs(computeForceEE.wrench.force.y)/2+std::abs(computeForceEE.wrench.torque.z)/2);

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pos_r1ee_r1force");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	Listener listener;
	// ros::Subscriber sub = node.subscribe<geometry_msgs::WrenchStamped>("/r1_force_data_filtered", 1000, &Listener::callback, &listener);	
	ros::Subscriber sub = node.subscribe<geometry_msgs::WrenchStamped>("/r1_force_data_filtered", 1000, &Listener::callback, &listener);	


	ros::spin();
	return 0;
}