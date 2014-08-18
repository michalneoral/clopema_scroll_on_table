#ifndef SCROLLINGGARMENT_H
#define SCROLLINGGARMENT_H

#include <ros/ros.h>
#include <clopema_robot/robot_commander.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/thread.hpp>

class ScrollingGarment {
	public:
		ros::subscriber sub;
		
	public:
		ScrollingGarment();
	};


#endif