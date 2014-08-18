#include <std_msgs/Empty.h>

int main(int argc, char **argv){

init(argc, argv, "reset_time");
Nodehandle n;
std_msg::Empty myMsg;
Publisher takeOff=n.advertise<std_msgs::Empty>("/reset_time",1,true);
takeOff.publish(myMsg);
spinOnce();

return 0;
}