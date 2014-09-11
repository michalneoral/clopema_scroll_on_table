#include <ros/ros.h>
#include "ScrollGarment.h"

void getListOfPoints3(std::vector<std::vector< geometry_msgs::Point>>& wp_map_1,std::vector<std::vector< geometry_msgs::Point>>& wp_map_2){
	//-----------------------------------------------------------------------
	geometry_msgs::Point blank;

	wp_map_1.clear();
	wp_map_2.clear();

	std::vector< geometry_msgs::Point> waypoints_1, waypoints_2;

	{
		blank.x = -1.4;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.y = -0.3;
		waypoints_1.push_back(blank);
		blank.x = -1.4;
		waypoints_1.push_back(blank);
		blank.y = 0.1;
		waypoints_1.push_back(blank);

		blank.x = -1.4;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.y = -0.5;
		waypoints_2.push_back(blank);
		blank.x = -1.4;
		waypoints_2.push_back(blank);
		blank.y = -0.1;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -0.9;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.x = -1.5;
		waypoints_1.push_back(blank);
		blank.x = -0.9;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.x = -1.5;
		waypoints_2.push_back(blank);
		blank.x = -0.9;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.2;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.y = -0.3;
		waypoints_1.push_back(blank);
		blank.x = -1.2;
		waypoints_1.push_back(blank);
		blank.y = 0.1;
		waypoints_1.push_back(blank);

		blank.x = -1.2;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.y = -0.5;
		waypoints_2.push_back(blank);
		blank.x = -1.2;
		waypoints_2.push_back(blank);
		blank.y = -0.1;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.2;
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.y = -0.3;
		waypoints_1.push_back(blank);

		blank.x = -1.1;
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.y = -0.5;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -0.9;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.x = -1.3;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.x = -1.3;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -0.8;
		blank.y = 0.15;
		waypoints_1.push_back(blank);
		blank.x = -1.2;
		blank.y = 0.6;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.15;
		waypoints_2.push_back(blank);
		blank.x = -1.3;
		blank.y = 0.4;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		blank.y = 0.15;
		waypoints_1.push_back(blank);
		blank.x = -1.2;		
		waypoints_1.push_back(blank);

		blank.x = -1.4;
		blank.y = -0.15;
		waypoints_2.push_back(blank);
		blank.x = -1.2;		
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();
}

void getListOfPoints2(std::vector<std::vector< geometry_msgs::Point>>& wp_map_1,std::vector<std::vector< geometry_msgs::Point>>& wp_map_2){
	geometry_msgs::Point blank;

	wp_map_1.clear();
	wp_map_2.clear();

	std::vector< geometry_msgs::Point> waypoints_1, waypoints_2;

	{
		blank.x = -1.4;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.x = -1.4;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.y = 0.3;
		waypoints_2.push_back(blank);
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.5;
		waypoints_1.push_back(blank);		

		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.3;
		waypoints_2.push_back(blank);		
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.5;
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.y = -0.3;
		waypoints_1.push_back(blank);
		
		blank.x = -1.5;
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.y = -0.5;
		waypoints_2.push_back(blank);		
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.y = -0.3;
		waypoints_1.push_back(blank);
		blank.x = -1.4;
		waypoints_1.push_back(blank);

		blank.y = -0.5;
		waypoints_2.push_back(blank);
		blank.x = -1.4;
		waypoints_2.push_back(blank);
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		waypoints_1.push_back(blank);
		blank.y = 0.1;
		waypoints_1.push_back(blank);

		blank.x = -1.4;
		waypoints_2.push_back(blank);
		blank.y = -0.1;
		waypoints_2.push_back(blank);
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();
}

void getListOfPoints4(std::vector<std::vector< geometry_msgs::Point>>& wp_map_1,std::vector<std::vector< geometry_msgs::Point>>& wp_map_2){
	geometry_msgs::Point blank;

	wp_map_1.clear();
	wp_map_2.clear();

	std::vector< geometry_msgs::Point> waypoints_1, waypoints_2;

	{
		// blank.x = -0.9;
		blank.x = -0.9;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.x = -1.7;
		waypoints_1.push_back(blank);

		// blank.x = -0.9;
		blank.x = -0.9;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.x = -1.7;
		waypoints_2.push_back(blank);
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();
}

// ALL
void getListOfPoints6(std::vector<std::vector< geometry_msgs::Point>>& wp_map_1,std::vector<std::vector< geometry_msgs::Point>>& wp_map_2){
	//-----------------------------------------------------------------------
	geometry_msgs::Point blank;

	wp_map_1.clear();
	wp_map_2.clear();

	std::vector< geometry_msgs::Point> waypoints_1, waypoints_2;

	{
		blank.x = -0.9;
		blank.y = 0.2;
		waypoints_1.push_back(blank);
		blank.x = -1.0;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.2;
		waypoints_2.push_back(blank);
		blank.x = -1.0;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -0.9;
		blank.y = 0.2;
		waypoints_1.push_back(blank);
		blank.x = -1.2;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.2;
		waypoints_2.push_back(blank);
		blank.x = -1.2;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -0.9;
		blank.y = 0.2;
		waypoints_1.push_back(blank);
		blank.x = -1.3;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.2;
		waypoints_2.push_back(blank);
		blank.x = -1.3;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		blank.y = 0.2;
		waypoints_1.push_back(blank);
		blank.x = -0.9;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.2;
		waypoints_2.push_back(blank);
		blank.x = -1.3;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		blank.y = 0.2;
		waypoints_1.push_back(blank);
		blank.x = -0.9;
		waypoints_1.push_back(blank);
		
		blank.x = -0.9;
		blank.y = -0.2;
		waypoints_2.push_back(blank);
		blank.x = -1.3;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.0;
		blank.y = 0.0;
		waypoints_1.push_back(blank);
		// blank.x = -0.9;
		blank.y = 0.13;
		waypoints_1.push_back(blank);
		
		blank.x = -1.0;
		blank.y = -0.07;
		waypoints_2.push_back(blank);
		blank.y = -0.07;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.0;
		blank.y = 0.07;
		waypoints_1.push_back(blank);
		// blank.x = -0.9;
		blank.y = 0.07;
		waypoints_1.push_back(blank);
		
		blank.x = -1.0;
		blank.y = -0.0;
		waypoints_2.push_back(blank);
		blank.y = -0.13;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.5;
		blank.y = 0.13;
		waypoints_1.push_back(blank);
		blank.x = -1.4;
		waypoints_1.push_back(blank);
		
		blank.x = -1.5;
		blank.y = -0.07;
		waypoints_2.push_back(blank);
		blank.x = -1.4;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		blank.y = 0.13;
		waypoints_1.push_back(blank);
		blank.x = -1.0;
		waypoints_1.push_back(blank);
		
		blank.x = -1.4;
		blank.y = -0.07;
		waypoints_2.push_back(blank);
		blank.x = -1.0;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		// blank.x = -0.9;
		blank.x = -0.9;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.x = -1.7;
		waypoints_1.push_back(blank);

		// blank.x = -0.9;
		blank.x = -0.9;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.x = -1.7;
		waypoints_2.push_back(blank);
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.x = -1.4;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.y = 0.3;
		waypoints_2.push_back(blank);
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.5;
		waypoints_1.push_back(blank);		

		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.3;
		waypoints_2.push_back(blank);		
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.5;
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.y = -0.3;
		waypoints_1.push_back(blank);
		
		blank.x = -1.5;
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.y = -0.5;
		waypoints_2.push_back(blank);		
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.y = -0.3;
		waypoints_1.push_back(blank);
		blank.x = -1.4;
		waypoints_1.push_back(blank);

		blank.y = -0.5;
		waypoints_2.push_back(blank);
		blank.x = -1.4;
		waypoints_2.push_back(blank);
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		waypoints_1.push_back(blank);
		blank.y = 0.1;
		waypoints_1.push_back(blank);

		blank.x = -1.4;
		waypoints_2.push_back(blank);
		blank.y = -0.1;
		waypoints_2.push_back(blank);
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.y = -0.3;
		waypoints_1.push_back(blank);
		blank.x = -1.4;
		waypoints_1.push_back(blank);
		blank.y = 0.1;
		waypoints_1.push_back(blank);

		blank.x = -1.4;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.y = -0.5;
		waypoints_2.push_back(blank);
		blank.x = -1.4;
		waypoints_2.push_back(blank);
		blank.y = -0.1;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -0.9;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.x = -1.5;
		waypoints_1.push_back(blank);
		blank.x = -0.9;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.x = -1.5;
		waypoints_2.push_back(blank);
		blank.x = -0.9;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.2;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.y = -0.3;
		waypoints_1.push_back(blank);
		blank.x = -1.2;
		waypoints_1.push_back(blank);
		blank.y = 0.1;
		waypoints_1.push_back(blank);

		blank.x = -1.2;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.y = -0.5;
		waypoints_2.push_back(blank);
		blank.x = -1.2;
		waypoints_2.push_back(blank);
		blank.y = -0.1;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.2;
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.y = -0.3;
		waypoints_1.push_back(blank);

		blank.x = -1.1;
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.y = -0.5;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -0.9;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.x = -1.3;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.x = -1.3;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -0.8;
		blank.y = 0.15;
		waypoints_1.push_back(blank);
		blank.x = -1.2;
		blank.y = 0.6;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.15;
		waypoints_2.push_back(blank);
		blank.x = -1.3;
		blank.y = 0.4;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		blank.y = 0.15;
		waypoints_1.push_back(blank);
		blank.x = -1.2;		
		waypoints_1.push_back(blank);

		blank.x = -1.4;
		blank.y = -0.15;
		waypoints_2.push_back(blank);
		blank.x = -1.2;		
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();
}

// EXECUTE WHOLE TRAJECTORY WITH ONE CONFIG
void getListOfPoints(std::vector<std::vector< geometry_msgs::Point>>& wp_map_1,std::vector<std::vector< geometry_msgs::Point>>& wp_map_2){
	//-----------------------------------------------------------------------
	geometry_msgs::Point blank;

	wp_map_1.clear();
	wp_map_2.clear();

	std::vector< geometry_msgs::Point> waypoints_1, waypoints_2;

	{
		blank.x = -0.9;
		blank.y = 0.2;
		waypoints_1.push_back(blank);
		blank.x = -1.0;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.2;
		waypoints_2.push_back(blank);
		blank.x = -1.0;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -0.9;
		blank.y = 0.2;
		waypoints_1.push_back(blank);
		blank.x = -1.2;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.2;
		waypoints_2.push_back(blank);
		blank.x = -1.2;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -0.9;
		blank.y = 0.2;
		waypoints_1.push_back(blank);
		blank.x = -1.3;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.2;
		waypoints_2.push_back(blank);
		blank.x = -1.3;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		blank.y = 0.2;
		waypoints_1.push_back(blank);
		blank.x = -0.9;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.2;
		waypoints_2.push_back(blank);
		blank.x = -1.3;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		blank.y = 0.2;
		waypoints_1.push_back(blank);
		blank.x = -0.9;
		waypoints_1.push_back(blank);
		
		blank.x = -0.9;
		blank.y = -0.2;
		waypoints_2.push_back(blank);
		blank.x = -1.3;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.0;
		blank.y = 0.0;
		waypoints_1.push_back(blank);
		// blank.x = -0.9;
		blank.y = 0.13;
		waypoints_1.push_back(blank);
		
		blank.x = -1.0;
		blank.y = -0.07;
		waypoints_2.push_back(blank);
		blank.y = -0.07;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.0;
		blank.y = 0.07;
		waypoints_1.push_back(blank);
		// blank.x = -0.9;
		blank.y = 0.07;
		waypoints_1.push_back(blank);
		
		blank.x = -1.0;
		blank.y = -0.0;
		waypoints_2.push_back(blank);
		blank.y = -0.13;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.5;
		blank.y = 0.13;
		waypoints_1.push_back(blank);
		blank.x = -1.4;
		waypoints_1.push_back(blank);
		
		blank.x = -1.5;
		blank.y = -0.07;
		waypoints_2.push_back(blank);
		blank.x = -1.4;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		blank.y = 0.13;
		waypoints_1.push_back(blank);
		blank.x = -1.0;
		waypoints_1.push_back(blank);
		
		blank.x = -1.4;
		blank.y = -0.07;
		waypoints_2.push_back(blank);
		blank.x = -1.0;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.x = -1.4;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.y = 0.3;
		waypoints_2.push_back(blank);
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.2;
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.y = -0.3;
		waypoints_1.push_back(blank);

		blank.x = -1.1;
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.y = -0.5;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -0.9;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.x = -1.3;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.x = -1.3;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -0.8;
		blank.y = 0.15;
		waypoints_1.push_back(blank);
		blank.x = -1.2;
		blank.y = 0.6;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.15;
		waypoints_2.push_back(blank);
		blank.x = -1.3;
		blank.y = 0.4;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		blank.y = 0.15;
		waypoints_1.push_back(blank);
		blank.x = -1.2;		
		waypoints_1.push_back(blank);

		blank.x = -1.4;
		blank.y = -0.15;
		waypoints_2.push_back(blank);
		blank.x = -1.2;		
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();
}

// EXECUTE PARTLY
void getListOfPoints5(std::vector<std::vector< geometry_msgs::Point>>& wp_map_1,std::vector<std::vector< geometry_msgs::Point>>& wp_map_2){
	//-----------------------------------------------------------------------
	geometry_msgs::Point blank;

	wp_map_1.clear();
	wp_map_2.clear();

	std::vector< geometry_msgs::Point> waypoints_1, waypoints_2;

	{
		// blank.x = -0.9;
		blank.x = -0.9;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.x = -1.7;
		waypoints_1.push_back(blank);

		// blank.x = -0.9;
		blank.x = -0.9;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.x = -1.7;
		waypoints_2.push_back(blank);
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.5;
		waypoints_1.push_back(blank);		

		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.3;
		waypoints_2.push_back(blank);		
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.5;
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.y = -0.3;
		waypoints_1.push_back(blank);
		
		blank.x = -1.5;
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.y = -0.5;
		waypoints_2.push_back(blank);		
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.y = -0.3;
		waypoints_1.push_back(blank);
		blank.x = -1.4;
		waypoints_1.push_back(blank);

		blank.y = -0.5;
		waypoints_2.push_back(blank);
		blank.x = -1.4;
		waypoints_2.push_back(blank);
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		waypoints_1.push_back(blank);
		blank.y = 0.1;
		waypoints_1.push_back(blank);

		blank.x = -1.4;
		waypoints_2.push_back(blank);
		blank.y = -0.1;
		waypoints_2.push_back(blank);
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.4;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.y = -0.3;
		waypoints_1.push_back(blank);
		blank.x = -1.4;
		waypoints_1.push_back(blank);
		blank.y = 0.1;
		waypoints_1.push_back(blank);

		blank.x = -1.4;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.y = -0.5;
		waypoints_2.push_back(blank);
		blank.x = -1.4;
		waypoints_2.push_back(blank);
		blank.y = -0.1;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -0.9;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.x = -1.5;
		waypoints_1.push_back(blank);
		blank.x = -0.9;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.x = -1.5;
		waypoints_2.push_back(blank);
		blank.x = -0.9;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();

	{
		blank.x = -1.2;
		blank.y = 0.1;
		waypoints_1.push_back(blank);
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.5;
		waypoints_1.push_back(blank);
		blank.y = -0.3;
		waypoints_1.push_back(blank);
		blank.x = -1.2;
		waypoints_1.push_back(blank);
		blank.y = 0.1;
		waypoints_1.push_back(blank);

		blank.x = -1.2;
		blank.y = -0.1;
		waypoints_2.push_back(blank);
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.x = -1.5;
		blank.y = 0.3;
		waypoints_2.push_back(blank);
		blank.y = -0.5;
		waypoints_2.push_back(blank);
		blank.x = -1.2;
		waypoints_2.push_back(blank);
		blank.y = -0.1;
		waypoints_2.push_back(blank);
	}

	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();
}

void getListOfTestsPoints(std::vector<std::vector< geometry_msgs::Point>>& wp_map_1,std::vector<std::vector< geometry_msgs::Point>>& wp_map_2){
	//-----------------------------------------------------------------------
	geometry_msgs::Point blank;
	wp_map_1.clear();
	wp_map_2.clear();

	std::vector< geometry_msgs::Point> waypoints_1, waypoints_2;

	{
		blank.x = -0.9;
		blank.y = 0.2;
		waypoints_1.push_back(blank);

		blank.x = -0.9;
		blank.y = -0.2;
		waypoints_2.push_back(blank);
	}
	wp_map_1.push_back(waypoints_1);
	wp_map_2.push_back(waypoints_2);
	waypoints_1.clear();
	waypoints_2.clear();
}

void posOK(int i){
	std::cout << "\033[1;35mTrajectory " << i << " was successfully executed.\033[0m"<< std::endl;
}

void posNotOK(int i){
	std::cout << "\033[1;31mTrajectory " << i << " \033[4;31mwas not\033[24;31m successfully executed.\033[0m"<< std::endl;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "scrolling_garment_on_the_table");
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	double force = 15;
	int testPos;
	bool isTest = false;
	if (argc >= 2) {
		force = std::stod(argv[1]);
		if(argc == 3) {
			testPos = std::stoi(argv[2]);
			std::cout << testPos << std::endl;
			isTest = true;
		}
	}

	clopema_robot::ClopemaRobotCommander ext("ext");
	ext.setNamedTarget("ext_minus_90");
	ext.move();
	ros::Duration(0.1).sleep();

	clopema_robot::ClopemaRobotCommander home("arms");
	home.setNamedTarget("home_arms");
	home.move();

	std::string frame_id = "base_link";
	std::string table_frame = "t3_desk";

	std::vector<std::vector<geometry_msgs::Point>> waypoints_1, waypoints_2;
	std::vector<bool> recap;
	
	ScrollGarment sg;
	sg.table_frame_ = table_frame;

	int start = 0;

	if(!isTest){
		getListOfPoints(waypoints_1, waypoints_2);
		for (int i=start; i < waypoints_1.size(); i++) {
			// {
			// 	ros::Duration(SLEEP_AFTER_EXEC).sleep();
			// 	clopema_robot::ClopemaRobotCommander arms("arms");
			// 	arms.setNamedTarget("home_arms");
			// 	arms.move();
			// }
			if(sg.moveOverTablePiecewise(frame_id,	waypoints_1[i], waypoints_2[i], table_frame, force))
			// if(sg.moveOverTable(frame_id,	waypoints_1[i], waypoints_2[i], table_frame, force))
			{
				posOK(i);
				recap.push_back(true);
			}else{
				posNotOK(i);
				recap.push_back(false);
			}
		}
	} else {
		getListOfTestsPoints(waypoints_1, waypoints_2);
		if(sg.testWeight(frame_id,	waypoints_1[testPos], waypoints_2[testPos], table_frame, force))
		{
			posOK(testPos);
			recap.push_back(true);
		}else{
			posNotOK(testPos);
			recap.push_back(false);
		}
	}
	
	std::cout << "Con:" << std::endl;
	for (int i = 0; i < recap.size(); i++){
		if(recap[i])
		{
			posOK(i+start);
		}else{
			posNotOK(i+start);
		}
	}

	ROS_INFO_STREAM("GOOD BYE");
	ros::spinOnce();
	return 0;
}