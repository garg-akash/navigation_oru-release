#include <ros/ros.h>
#include <boost/bind.hpp>

#include <orunav_conversions/conversions.h>

#include <orunav_msgs/ControllerReport.h>

#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>

#include <iostream>

using namespace std;
using namespace orunav_msgs;

bool match(const ControllerReportConstPtr& rep_match, orunav_generic::Path P) {
	double g_x = abs(rep_match->state.position_x - P.getPose2d(P.sizePath()-1)(0));
  	double g_y = abs(rep_match->state.position_y - P.getPose2d(P.sizePath()-1)(1));
  	double g_th = abs(rep_match->state.orientation_angle - P.getPose2d(P.sizePath()-1)(2));
  	double g_st = abs(rep_match->state.steering_angle - P.getSteeringAngle(P.sizePath()-1)); 

  	if (g_x <= 0.1  && g_y <= 0.1 && g_th <= 2 && g_st <= 0.5) {
  	  	return true;
  	}
  	else
  	  	return false;
}

void callback1(ros::NodeHandle &node_handle, const ControllerReportConstPtr& rep1) {
	int bp_robot1;
	double bp_robot1_begin;
	node_handle.param<int>("/bp_robot1", bp_robot1, 0);
	if (bp_robot1 == 1) {
		orunav_generic::Path loaded_path_1 = orunav_generic::loadPathTextFile("/home/akash/catkin_myoru/path11.txt");
		if (match(rep1, loaded_path_1)) {
			node_handle.getParam("/bp_robot1_begin", bp_robot1_begin);
			double goalTime_1 = ros::Time::now().toSec() - bp_robot1_begin; 
			cout<<"[TimeToGoal] for robot1 is : "<<ros::Time::now().toSec()<<"sec - "<<bp_robot1_begin<<"sec = "<<goalTime_1<<"sec"<<endl;
			node_handle.setParam("/bp_robot1", 0);
		}
	}
}

void callback2(ros::NodeHandle &node_handle, const ControllerReportConstPtr& rep2) {
	int bp_robot2;
	double bp_robot2_begin;
	node_handle.param<int>("/bp_robot2", bp_robot2, 0);
	if (bp_robot2 == 1) {
		orunav_generic::Path loaded_path_2 = orunav_generic::loadPathTextFile("/home/akash/catkin_myoru/path22.txt");
		if (match(rep2, loaded_path_2)) {
			node_handle.getParam("/bp_robot2_begin", bp_robot2_begin);
			double goalTime_2 = ros::Time::now().toSec() - bp_robot2_begin;
			cout<<"[TimeToGoal] for robot2 is : "<<ros::Time::now().toSec()<<"sec - "<<bp_robot2_begin<<"sec = "<<goalTime_2<<"sec"<<endl;
			node_handle.setParam("/bp_robot2", 0);
		}
	}
}

void callback3(ros::NodeHandle &node_handle, const ControllerReportConstPtr& rep3) {
	int bp_robot3;
	double bp_robot3_begin;
	node_handle.param<int>("/bp_robot3", bp_robot3, 0);
	if (bp_robot3 == 1) {
		orunav_generic::Path loaded_path_3 = orunav_generic::loadPathTextFile("/home/akash/catkin_myoru/path33.txt");
		if (match(rep3, loaded_path_3)) {
			node_handle.getParam("/bp_robot3_begin", bp_robot3_begin);
			double goalTime_3 = ros::Time::now().toSec() - bp_robot3_begin;
			cout<<"[TimeToGoal] for robot3 is : "<<ros::Time::now().toSec()<<"sec - "<<bp_robot3_begin<<"sec = "<<goalTime_3<<"sec"<<endl;
			node_handle.setParam("/bp_robot3", 0);
		}
	}		
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "TimeToGoal");
	ros::NodeHandle nh;

	ros::Subscriber report1_sub = nh.subscribe<ControllerReport>("/robot1/controller/reports", 1, boost::bind(&callback1, boost::ref(nh), _1));
	ros::Subscriber report2_sub = nh.subscribe<ControllerReport>("/robot2/controller/reports", 1, boost::bind(&callback2, boost::ref(nh), _1));
	ros::Subscriber report3_sub = nh.subscribe<ControllerReport>("/robot3/controller/reports", 1, boost::bind(&callback3, boost::ref(nh), _1));

	ros::spin();
	return 0;
}