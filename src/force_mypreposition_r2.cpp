#include <ros/ros.h>
#include <clopema_robot/robot_commander.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>

#define ROLL2DESK 0 //(M_PI)
#define PITCH2DESK M_PI/2+M_PI/6 //(M_PI/2-M_PI/6)
#define YAW2DESK  3*M_PI/4//(-M_PI/2)

#define XPOS (-0.9)
#define YPOS (-0.4)
#define ZPOS 1.05

// #define OPR_X 1.5
// #define OPR_Y 0
// #define OPR_Z (-0.78)

#define OPR_X 0
#define OPR_Y 0
#define OPR_Z 0

class MoveForceArm {
public:

    MoveForceArm() : node("~"), crc("arms"){
        // crc.reset(new clopema_robot::ClopemaRobotCommander("arms"));
        crc.setRobotSpeed(0.2);
        crc.setPoseReferenceFrame("t3_desk");
        // crc.setPoseReferenceFrame("ctu_floor");
//        sub_force = node.subscribe("/r2_force_data_filtered", 1, &MoveForceArm::force_filtred_sub, this);
    }

    ~MoveForceArm() {

    }

    bool moveExtM90();
    bool moveExtHome();
    bool moveArmsHome();
    bool preparePosition();
    void openGripper();
    void closeGripper();
    bool downThrowTheDesk(double n);
    void servoOff();
    void force_filtred_sub(const geometry_msgs::WrenchStamped& msg);
    



private:
    ros::NodeHandle node;
    boost::shared_ptr<Eigen::Vector3d> force_offset;
    double force_mag;
    boost::mutex mutex_wrench;
    ros::Subscriber sub_force;
public:
    // boost::shared_ptr<clopema_robot::ClopemaRobotCommander> crc;
    //		clopema_robot::ClopemaRobotCommander g("arms");
    clopema_robot::ClopemaRobotCommander crc;
};

void MoveForceArm::force_filtred_sub(const geometry_msgs::WrenchStamped& msg) {
    char buffer [100];
    sprintf(buffer, "%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f", msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z);
    ROS_INFO_STREAM(buffer);
}

//abraka

void showPose(geometry_msgs::Pose pose) {
    char buffer [100];
    sprintf(buffer, "%.3f\t%.3f\t%.4f\t%.2f\t%.2f\t%.2f\t%.2f", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    ROS_INFO_STREAM(buffer);
}

bool MoveForceArm::downThrowTheDesk(double n) {
    crc.setRobotSpeed(0.02);
    robot_state::RobotState rs(*crc.getCurrentState());
    geometry_msgs::Pose p;
    p = crc.getCurrentPose("r2_ee").pose;
    p.position.z = p.position.z - 0.001 * n;
    showPose(p);
    if (!rs.setFromIK(rs.getJointModelGroup("r2_arm"), p, "r2_ee")) {
        ROS_WARN_STREAM("Cannot set from IK - second arm");
        return false;
    }
    crc.setJointValueTarget(rs);
    return crc.move();
}

void MoveForceArm::openGripper() {
    crc.setGripperState(clopema_robot::ACTION_R1_GRIPPER, clopema_robot::GRIPPER_OPEN);
    crc.setGripperState(clopema_robot::ACTION_R2_GRIPPER, clopema_robot::GRIPPER_OPEN, true);
    crc.move();
    //    crc.execute_traj(crc.gripper_trajectory(clopema_robot::GRIPPER_OPEN));
    //    crc.setServoPowerOff();
}

void MoveForceArm::servoOff() {
    crc.setServoPowerOff();
}

void MoveForceArm::closeGripper() {
    crc.setGripperState(clopema_robot::ACTION_R1_GRIPPER, clopema_robot::GRIPPER_CLOSE);
    crc.setGripperState(clopema_robot::ACTION_R2_GRIPPER, clopema_robot::GRIPPER_CLOSE, true);
    crc.move();
    //    crc.execute_traj(crc.gripper_trajectory(clopema_robot::GRIPPER_CLOSE));
    //    crc.setServoPowerOff();
}

bool MoveForceArm::preparePosition() {
    crc.setPoseReferenceFrame("t3_desk");
    ROS_INFO_STREAM(crc.getPoseReferenceFrame());
    crc.setRobotSpeed(0.2);
    geometry_msgs::Pose p1, p2;
    robot_state::RobotState rs(*crc.getCurrentState());
    crc.setStartState(rs);
    ROS_INFO_STREAM(crc.getPoseReferenceFrame());

    p2.position.x = XPOS+OPR_X;
    p2.position.y = YPOS+OPR_Y;
    p2.position.z = ZPOS+OPR_Z;
    p2.orientation = tf::createQuaternionMsgFromRollPitchYaw(ROLL2DESK, PITCH2DESK, YAW2DESK);
    ROS_INFO_STREAM(p2.orientation);
    if (!rs.setFromIK(rs.getJointModelGroup("r2_arm"), p2, "r2_ee")) {
        ROS_WARN_STREAM("Cannot set from IK - second arm");
        return false;
    }
    // p1.position.x = -1.2+OPR_X;
    // p1.position.y = 0.5+OPR_Y;
    // p1.position.z = 1.2+OPR_Z;
    // p1.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
    // if (!rs.setFromIK(rs.getJointModelGroup("r1_arm"), p1, "r1_ee")) {
    //     ROS_WARN_STREAM("Cannot set from IK - first arm");
    //     return false;
    // }
    crc.setJointValueTarget(rs);
    ROS_INFO_STREAM(crc.getPoseReferenceFrame());
    return crc.move();
}

bool MoveForceArm::moveExtM90() {
    crc.setRobotSpeed(0.2);
    clopema_robot::ClopemaRobotCommander ext("ext");
    ext.setNamedTarget("ext_minus_90");
    return ext.move();
}

bool MoveForceArm::moveExtHome() {
    crc.setRobotSpeed(0.2);
    clopema_robot::ClopemaRobotCommander ext("ext");
    ext.setNamedTarget("ext_home");
    return ext.move();
}

bool MoveForceArm::moveArmsHome() {
    crc.setRobotSpeed(0.2);
    clopema_robot::ClopemaRobotCommander arms("arms");
    arms.setNamedTarget("home_arms");
    return arms.move();
}

void pLine(int n) {
    int i;
    for (i = 0; i < n; i++) {
        std::cout << "*";
    }
}



void printMenu() {
    pLine(40);
    std::cout << "\n";
    std::cout << "1) Home position\n";
    std::cout << "2) Prepare position\n";
    std::cout << "3) Down (z-axis minus 1mm)\n";
    std::cout << "4) Down (z-axis plus 1mm)\n";
    std::cout << "5) Down (z-axis minus x-mm)\n";
    std::cout << "6) Set servo off\n";
    std::cout << "\n";
    std::cout << "7) Exit\n";
    pLine(40);
    std::cout << "\n";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "force_mypreposition_r2");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    MoveForceArm m;
    int choice;
    bool isOn = true;
    double n = 0.0;

    while (isOn) {
        printMenu();
        std::cin >> choice;

        switch (choice) {
            case 1:
                m.closeGripper();
                m.moveArmsHome();
                m.moveExtHome();
                m.servoOff();
                break;
            case 2:
                m.openGripper();
                m.moveExtM90();
                m.preparePosition();
                m.servoOff();
                break;
            case 3:
                m.downThrowTheDesk(1.0);
                break;
            case 4:
                m.downThrowTheDesk(-1.0);
                break;
            case 5:
                std::cout << "Set z-axis relative position\n";
                std::cin >> n;
                m.downThrowTheDesk(n);
                break;
            case 6:
                m.servoOff();
                break;
            case 7:
                m.servoOff();
                isOn = false;
                break;
        }
    }
    spinner.stop();
    return 0;
}
