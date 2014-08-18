#include <ros/ros.h>
#include <clopema_robot/robot_commander.h>

void showPose(geometry_msgs::Pose pose){
	char buffer [100];
	sprintf(buffer, "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f",pose.position.x,pose.position.y,pose.position.z,pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
	ROS_INFO_STREAM(buffer);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_state", ros::init_options::AnonymousName);
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

	clopema_robot::ClopemaRobotCommander robot("arms");

	geometry_msgs::Pose p;
	p = robot.getCurrentPose("r1_ee").pose;
	showPose(p);
	p = robot.getCurrentPose("r2_ee").pose;
	showPose(p);

	ros::shutdown();
	return 0;
}
