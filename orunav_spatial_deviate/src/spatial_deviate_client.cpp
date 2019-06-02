#include <ros/ros.h>
#include <orunav_msgs/SpatialDeviate.h>
#include <orunav_conversions/conversions.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>
#include <orunav_generic/types.h>
#include <orunav_generic/path_utils.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#define _USE_MATH_DEFINES   
#include <math.h>
#include <cmath>
#include <algorithm>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "get_spatial_deviation_client");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<orunav_msgs::SpatialDeviate>("get_spatial_deviation");
  orunav_msgs::SpatialDeviate srv;
  orunav_msgs::Path p1, p2;
  srv.request.path1 = p1;
  srv.request.path2 = p2;

  if(client.call(srv)) {
  	ROS_INFO("Got a spatially adjusted from SD service"); //srv.response.sptl_path1;srv.response.sptl_path2
  }

  else {
  	ROS_ERROR("Failed to call SD service");
  	return 1;
  }

  return 0;
}