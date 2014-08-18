#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/WrenchStamped.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "force_mypreposition_r2");
    ros::NodeHandle node("~");

	return 0;
}
