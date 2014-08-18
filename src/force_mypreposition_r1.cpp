#include <ros/ros.h>
#include <clopema_robot/robot_commander.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>

#define ROLL2DESK (M_PI)
#define PITCH2DESK (M_PI/2-M_PI/6)
#define YAW2DESK (M_PI/2)

#define XPOS -0.9
#define YPOS 0.4
#define ZPOS 1.05

#define XPOS_CAM -0.7844
#define YPOS_CAM 0.1878
#define ZPOS_CAM 1.0221

// #define ROLL_CAM 0.4548
// #define PITCH_CAM 0.7965
// #define YAW_CAM 1.7461

#define QX_CAM 0.64395
#define QY_CAM 0.63198
#define QZ_CAM 0.401903
#define QW_CAM -0.156217

// #define ROLL_CAM (-3*M_PI/2)
// #define PITCH_CAM (2*M_PI/3)
// #define YAW_CAM (M_PI/2)



class MoveForceArm {
public:

    MoveForceArm() : node("~") {
        crc.reset(new clopema_robot::ClopemaRobotCommander("arms"));
        crc->setRobotSpeed(0.2);
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
    boost::shared_ptr<clopema_robot::ClopemaRobotCommander> crc;
    //		clopema_robot::ClopemaRobotCommander g("arms");
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
    crc->setRobotSpeed(0.02);
    robot_state::RobotState rs(*crc->getCurrentState());
    geometry_msgs::Pose p;
    p = crc->getCurrentPose("r1_ee").pose;
    p.position.z = p.position.z - 0.001 * n;
    showPose(p);
    if (!rs.setFromIK(rs.getJointModelGroup("r1_arm"), p, "r1_ee")) {
        ROS_WARN_STREAM("Cannot set from IK - first arm");
        return false;
    }
    crc->setJointValueTarget(rs);
    return crc->move();
}

void MoveForceArm::openGripper() {
    crc->setGripperState(clopema_robot::ACTION_R1_GRIPPER, clopema_robot::GRIPPER_OPEN, true);
    crc->setGripperState(clopema_robot::ACTION_R2_GRIPPER, clopema_robot::GRIPPER_OPEN, true);
    crc->move();
    //    crc->execute_traj(crc->gripper_trajectory(clopema_robot::GRIPPER_OPEN));
    //    crc->setServoPowerOff();
}

void MoveForceArm::servoOff() {
    crc->setServoPowerOff();
}

void MoveForceArm::closeGripper() {
    crc->setGripperState(clopema_robot::ACTION_R1_GRIPPER, clopema_robot::GRIPPER_CLOSE, true);
    crc->setGripperState(clopema_robot::ACTION_R2_GRIPPER, clopema_robot::GRIPPER_CLOSE, true);
    crc->move();
    //    crc.execute_traj(crc.gripper_trajectory(clopema_robot::GRIPPER_CLOSE));
    //    crc.setServoPowerOff();
}

bool MoveForceArm::preparePosition() {
    crc->setRobotSpeed(0.2);
    geometry_msgs::Pose p1, p2;
    robot_state::RobotState rs(*crc->getCurrentState());

    p1.position.x = XPOS;
    p1.position.y = YPOS;
    p1.position.z = ZPOS;
    p1.orientation = tf::createQuaternionMsgFromRollPitchYaw(ROLL2DESK, PITCH2DESK, YAW2DESK);
    if (!rs.setFromIK(rs.getJointModelGroup("r1_arm"), p1, "r1_ee")) {
        ROS_WARN_STREAM("Cannot set from IK - first arm");
        return false;
    }
    p2.position.x = XPOS_CAM;
    p2.position.y = YPOS_CAM;
    p2.position.z = ZPOS_CAM;
    // p2.orientation = tf::createQuaternionMsgFromRollPitchYaw(ROLL_CAM, PITCH_CAM, YAW_CAM);
    p2.orientation.x = QX_CAM;
    p2.orientation.y = QY_CAM;
    p2.orientation.z = QZ_CAM;
    p2.orientation.w = QW_CAM;

    if (!rs.setFromIK(rs.getJointModelGroup("r2_arm"), p2, "r2_ee")) {
        ROS_WARN_STREAM("Cannot set from IK - second arm");
        return false;
    }
    crc->setJointValueTarget(rs);
    return crc->move();
}

bool MoveForceArm::moveExtM90() {
    crc->setRobotSpeed(0.2);
    clopema_robot::ClopemaRobotCommander ext("ext");
    ext.setNamedTarget("ext_minus_90");
    return ext.move();
}

bool MoveForceArm::moveExtHome() {
    crc->setRobotSpeed(0.2);
    clopema_robot::ClopemaRobotCommander ext("ext");
    ext.setNamedTarget("ext_home");
    return ext.move();
}

bool MoveForceArm::moveArmsHome() {
    crc->setRobotSpeed(0.2);
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
    ros::init(argc, argv, "force_mypreposition_r1");
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
