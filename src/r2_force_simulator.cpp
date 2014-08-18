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



void force_info_stream(geometry_msgs::WrenchStamped msg) {
	char buffer [100];
	sprintf(buffer, "%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f", msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z);
	ROS_INFO_STREAM(buffer);
}

double getDiffR2ee(){
	
	ros::Rate rate(100);
	double z;
	tf::TransformListener listener;
	tf::StampedTransform transform;

	while (ros::ok()){
		try{
			ros::Time latest_transform_time=ros::Time::now();
			// listener.getLatestCommonTime("/ctu_floor","/r2_ee", latest_transform_time, NULL);

			listener.waitForTransform("/ctu_floor","/r2_ee", latest_transform_time, ros::Duration(0.01) );
			listener.lookupTransform("/ctu_floor", "/r2_ee",  
				latest_transform_time, transform);
			z=transform.getOrigin().z();
			ROS_INFO_STREAM("----------------------------");
			break;
		}
		catch (tf::TransformException ex){
			// ROS_ERROR("%s",ex.what());
		}
		rate.sleep();
	}
	return z;
}

class Listener
{
public:
	int seq;
	ros::Publisher pub;
public:
	Listener();
	void callback(tf2_msgs::TFMessage data);
	geometry_msgs::WrenchStamped computeForceOutputEE ();
	double computeSingle (double z, double a2, double a1, double a0);
};

Listener::Listener(){
	seq=1;
}

double Listener::computeSingle (double z, double a2, double a1, double a0){
	double force;
	if (z>START_HEIGHT)
		{z=0;}
	else {z=(START_HEIGHT-z)*M_TO_MM;}
	force += pow(z,2) * (a2);
	force += z * (a1);
	force += a0;
	return force;
}

geometry_msgs::WrenchStamped Listener::computeForceOutputEE (){
	geometry_msgs::WrenchStamped simulateForce;
	simulateForce.header.seq=seq;
	seq+=1;
	double z;
	z = getDiffR2ee();
	ros::Time t = ros::Time::now();
	simulateForce.header.stamp = t;
	simulateForce.header.frame_id = "r2_force_data_filtered";
	ROS_INFO_STREAM(z);
	simulateForce.wrench.force.x = computeSingle (z, 0.0025,    1.3178,   -1.4595);
	simulateForce.wrench.force.y = computeSingle (z, -0.0046,   -2.1012,    2.3930);
	simulateForce.wrench.force.z = computeSingle (z, -0.0150 ,  -1.2819 ,   1.0194);
	simulateForce.wrench.torque.x = computeSingle (z, 0.0033  ,  0.5024  , -0.6238);
	simulateForce.wrench.torque.y = computeSingle (z, 0.0014   , 0.3429   ,-0.4181);
	simulateForce.wrench.torque.z = computeSingle (z, 0.0001    ,0.0005   ,-0.0025);
	return simulateForce;
}

void Listener::callback(tf2_msgs::TFMessage data){
	

	geometry_msgs::WrenchStamped simForce=computeForceOutputEE();
	force_info_stream(simForce);
	pub.publish(simForce);
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "r2_force_simulator");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Publisher pub = node.advertise<geometry_msgs::WrenchStamped>("/r2_force_data_filtered", 1000);

	Listener listener;
	listener.pub=pub;
	ros::Subscriber sub = node.subscribe<tf2_msgs::TFMessage>("/tf", 10, &Listener::callback, &listener);	

	ros::spin();
	return 0;
}