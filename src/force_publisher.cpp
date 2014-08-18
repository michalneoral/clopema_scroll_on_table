#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/WrenchStamped.h>

void twoSecondsForce(ros::Publisher& chatter_pub){
  ros::Rate loop_rate(10);
  uint count = 0;
  while (ros::ok())
  {
    geometry_msgs::WrenchStamped msg;
    geometry_msgs::WrenchStamped msg2;
    msg2.wrench.force.x = 0;
    msg2.wrench.force.y = 0;
    msg2.wrench.force.z = 0;
    msg2.wrench.torque.x = 0;
    msg2.wrench.torque.y = 0;
    msg2.wrench.torque.z = 0;
    msg2.header.frame_id = "r2_force_sensor";

    double x,y,z,wx,wy,wz;
    if (6==scanf("%lf%lf%lf%lf%lf%lf",&x,&y,&z,&wx,&wy,&wz)) {
      msg.wrench.force.x = x;
      msg.wrench.force.y = y;
      msg.wrench.force.z = z;
      msg.wrench.torque.x = wx;
      msg.wrench.torque.y = wy;
      msg.wrench.torque.z = wz;
      msg.header.frame_id = "r2_force_sensor";
      msg.header.stamp = ros::Time::now();
      msg.header.seq = count;
      chatter_pub.publish(msg);
      count++;
      ROS_INFO("OK");
      ros::Duration(2).sleep();

      msg2.header.stamp = ros::Time::now();
      msg2.header.seq = count;
      chatter_pub.publish(msg2);
      ROS_INFO("OK2");
    } else {

      ROS_INFO("NONE");
    }
  }
}

void permanentConstantForce(ros::Publisher& pub1, ros::Publisher& pub2, double force, bool init){
  ros::Rate loop_rate(10);
  uint count = 0;

  geometry_msgs::WrenchStamped msg;
  msg.wrench.force.x = force*0;
  msg.wrench.force.y = force*0;
  msg.wrench.force.z = force;
  msg.wrench.torque.x = 0;
  msg.wrench.torque.y = 0;
  msg.wrench.torque.z = 0;
  msg.header.frame_id = "r2_force_sensor";

  while (ros::ok())
  {
    count++;

    msg.header.stamp = ros::Time::now();
    msg.header.seq = count;

    pub1.publish(msg);
    pub2.publish(msg);

    std::cout << count << std::endl;
    ros::Duration(0.05).sleep();

    if(count >= 10 && init){
      break;
    }
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "force_publisher_test");
  ros::NodeHandle n;
  ros::Publisher pub1 = n.advertise<geometry_msgs::WrenchStamped>("r1_force_data_filtered", 1000);
  ros::Publisher pub2 = n.advertise<geometry_msgs::WrenchStamped>("r2_force_data_filtered", 1000);
  
  // twoSecondsForce(chatter_pub);
  std::cout << "number of arguments: " << argc << std::endl; 

  permanentConstantForce(pub1, pub2, 0, true);
  if (argc >= 2){
    permanentConstantForce(pub1, pub2, std::stoi(argv[1]), false);
  } else {
    permanentConstantForce(pub1, pub2, 15, false);
  }

  return 0;
}
