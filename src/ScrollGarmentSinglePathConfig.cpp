#include "ScrollGarmentSinglePathConfig.h"

ScrollGarmentSinglePathConfig::ScrollGarmentSinglePathConfig(){

}

void ScrollGarmentSinglePathConfig::changeWaypoint1(geometry_msgs::Point p1, geometry_msgs::Point p2){
	short_waypoints_1_.clear();
	short_waypoints_1_.push_back(p1);
	short_waypoints_1_.push_back(p2);
}

void ScrollGarmentSinglePathConfig::changeWaypoint2(geometry_msgs::Point p1, geometry_msgs::Point p2){
	short_waypoints_2_.clear();
	short_waypoints_2_.push_back(p1);
	short_waypoints_2_.push_back(p2);
}

double ScrollGarmentSinglePathConfig::getPercentage(){
	return std::min(lowerTrajPercentage_, upperTrajPercentage_);
}