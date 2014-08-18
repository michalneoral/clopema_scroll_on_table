#include <ros/ros.h>
#include <clopema_robot/robot_commander.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>
#include <clopema_gripper/SkinData.h>

#define MAG 21
#define OFFSET 10000


class Listener
{
public:
	Listener();
	void callback(clopema_gripper::SkinData data);
	double computeForceOutputEE (clopema_gripper::SkinData data);
};

Listener::Listener(){	
}

double Listener::computeForceOutputEE (clopema_gripper::SkinData data){
	double mag = data.sensor_responses[MAG] - OFFSET;
	double force = pow(mag,3) * (-2.972e-07);
	force += pow(mag,2) * (3.231e-04);
	force += mag * (5.755e-02);
	force += -12.14;
	// -2,97221846036182e-07	0,000323050478907719	0,0575535201012086	-12,1493660911300
	return force;
}

void Listener::callback(clopema_gripper::SkinData data){
/*	char buffer [100];
	sprintf(buffer, "Force Fz from mag sensor %.2f", data.sensor_responses);
	ROS_INFO_STREAM(buffer);*/
	ROS_INFO_STREAM(data.sensor_responses[MAG]);
	double force = computeForceOutputEE(data);
	char buffer [100];
	sprintf(buffer, "Force compute from mag sensor: %6.2f [N]", force);
	ROS_INFO_STREAM(buffer);

}

// -7,34970992747361e-07	0,0227494954523723	-234,545718338912	805475,773338617

int main(int argc, char **argv) {
	ros::init(argc, argv, "show_r1force_from_r1magsense");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	Listener listener;
	ros::Subscriber sub = node.subscribe<clopema_gripper::SkinData>("/clopema_gripper/SkinDataPeriodic", 1000, &Listener::callback, &listener);	

	ros::spin();
	return 0;
}