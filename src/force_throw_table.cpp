#include <ros/ros.h>
#include <clopema_robot/robot_commander.h>
#include <fstream>
#include <iostream>
#include <clopema_utilities/clopema_pose_generator.h>
#include <boost/foreach.hpp>

//void load_poses(std::vector<geometry_msgs::Pose>& poses, const std::string& file) {
//    std::ifstream ifs;
//    ifs.open(file.c_str());
//    if(!ifs.is_open()) {
//        ROS_ERROR_STREAM("Cannot open file: " << file);
//    }

//    while(!ifs.eof()) {
//        geometry_msgs::Pose p;
//        char c;
//        ifs >> p.position.x >> c;
//        ifs >> p.position.y >> c;
//        ifs >> p.position.z >> c;
//        ifs >> p.orientation.x >> c;
//        ifs >> p.orientation.y >> c;
//        ifs >> p.orientation.z >> c;
//        ifs >> p.orientation.w;
//        poses.push_back(p);
//    }
//    poses.pop_back();
//}

int main(int argc, char **argv) {
    ros::init(argc, argv, "throw_table");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    clopema_robot::ClopemaRobotCommander g("arms");

    geometry_msgs::Pose pose;
    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, 0, 0*M_PI / 2);
    pose.position.y = -0.2;
    pose.position.x = -1.2;
    pose.position.z = 0.50;
    std::vector<trajectory_msgs::JointTrajectory> trajectories;

    if(!g.grasp_from_table_plan(pose, "r1_ee", "t3_desk", trajectories)) {
        ROS_ERROR_STREAM("Cannot grasp from table");
        return 0;
    }

    ROS_INFO_STREAM("Execute? yes,next,abort [y/N] ");
    char c('n');
    std::cin >> c;
    if (c == 'n' || c == 'N')
        return 0;

    move_group_interface::MoveGroup::Plan plan;
    for (unsigned int i = 0; i < trajectories.size(); ++i) {
        plan.trajectory_.joint_trajectory = trajectories[i];
        g.execute(plan);
    }

    spinner.stop();
    ros::shutdown();
    return 0;
}
