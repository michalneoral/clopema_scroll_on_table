#include <ros/ros.h>
#include <clopema_robot/robot_commander.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <fstream>

int saveFile(geometry_msgs::Pose pose){
	std::ofstream myfile;
	myfile.open ("/home/neosh/ros_catkin_ws/src/clopema_cvut/clopema_dep_force_position/matlab/forceSensor/transformR2.txt");
	char buffer [256];
	sprintf(buffer, "%.12f\t%.12f\t%.12f\t%.12f\t%.12f\t%.12f\t%.12f\n", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	
	myfile << buffer;
	myfile.close();
	return 0;
}

void showPose(geometry_msgs::Pose pose) {
	char buffer [100];
	sprintf(buffer, "%.3f\t%.3f\t%.4f\t%.2f\t%.2f\t%.2f\t%f", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	ROS_INFO_STREAM(buffer);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pos_r2ee_r2force");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	tf::TransformListener listener;
	int i=0;
	ros::Rate rate(1.0);
	while (ros::ok()){
		
		tf::StampedTransform transform;
	    // geometry_msgs::PoseStamped transform;
		try{
			listener.lookupTransform("/r2_force_sensor", "/r2_ee",  
				ros::Time(0), transform);
			geometry_msgs::Pose pos;
			pos.position.x=transform.getOrigin().x();
			pos.position.y=transform.getOrigin().y();
			pos.position.z=transform.getOrigin().z();
			pos.orientation.x=transform.getRotation().x();
			pos.orientation.y=transform.getRotation().y();
			pos.orientation.z=transform.getRotation().z();
			pos.orientation.w=transform.getRotation().w();
	    // ROS_INFO_STREAM(transform.getOrigin().x());
			showPose(pos);
			if (i==100)
				{ROS_INFO_STREAM("E");
				saveFile(pos);
				break;}
			i++;
			rate.sleep();

		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}



	}

    // (trans,rot) = listener.lookupTransform('/ctu_floor', '/r2_ee', t);

	spinner.stop();
	return 0;
}