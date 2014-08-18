#include "TrajectoryPublisherScroll.h"

TrajectoryPublisherScroll::TrajectoryPublisherScroll(){
    pub_traj_ = node_.advertise<trajectory_msgs::JointTrajectory>("dual_command", 1);
    while(pub_traj_.getNumSubscribers() == 0) {
        ros::Duration(0.05).sleep();
        if(!ros::ok())
            return;
    }
    pub_rosbag_ = node_.advertise<trajectory_msgs::JointTrajectory>("where_is_error", 1);
    while(pub_traj_.getNumSubscribers() == 0) {
        ros::Duration(0.05).sleep();
        if(!ros::ok())
            return;
    }
}

// void TrajectoryPublisherScroll::execute(const trajectory_msgs::JointTrajectory& trajectory) {    
//     pub_traj_.publish(trajectory);
// }

void TrajectoryPublisherScroll::controle(const trajectory_msgs::JointTrajectory& trajectory) {    
    pub_rosbag_.publish(trajectory);
}