#include "ScrollGarmentSinglePathConfig.h"

ScrollGarmentSinglePathConfig::ScrollGarmentSinglePathConfig(){
	lowerTrajPercentage_ = 0;
	upperTrajPercentage_ = 0;
	centerTrajPercentage_ = 0;
	centerTrajSteps_ = 0;
	configuration_ = false;
	yawR1_ = 0;
	yawR2_ = 0;
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

double ScrollGarmentSinglePathConfig::getLPer(){
	return lowerTrajPercentage_;
}

double ScrollGarmentSinglePathConfig::getUPer(){
	return upperTrajPercentage_;
}

bool ScrollGarmentSinglePathConfig::isEqual(const ScrollGarmentSinglePathConfig& a){
	// std::cout << "\033[1;34mEqual: " << a.yawR1_ - yawR1_ << "\t" << a.yawR2_ - yawR2_ << "\t" << (configuration_ == a.configuration_) << "\033[0m" << std::endl; //]]

	if( !(configuration_ == a.configuration_) ) return false;
	if( std::abs(a.yawR1_ - yawR1_) >= OFFSET ) return false;
	if( std::abs(a.yawR2_ - yawR2_) >= OFFSET ) return false;
	return true;
}
