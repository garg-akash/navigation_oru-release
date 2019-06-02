#include <ros/ros.h>
#include <orunav_msgs/GetBestPath.h>
#include <orunav_conversions/conversions.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>
#include <orunav_generic/types.h>
#include <orunav_generic/path_utils.h>
#include <orunav_msgs/Task.h>
#include <orunav_msgs/RobotTarget.h>

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
  ros::init(argc, argv, "get_best_path_client");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<orunav_msgs::GetBestPath>("get_best_path");
  orunav_msgs::GetBestPath srv;
  orunav_msgs::RobotTarget robo_t;
  orunav_msgs::Task obs1_t;
  vector<orunav_msgs::Task> obs_t; 

  orunav_generic::Path path_1 = orunav_generic::loadPathTextFile("path10.txt"); //for now just two paths
  orunav_generic::Path path_2 = orunav_generic::loadPathTextFile("path20.txt");
  if((path_1.sizePath())*(path_1.sizePath()) != 0 ) {
  	cout<<"Path files read"<<endl;
  	cout<<"Size of file1: "<<path_1.sizePath()<<endl;
  	cout<<"Size of file2: "<<path_2.sizePath()<<endl;
  }

  robo_t.start = orunav_conversions::createPoseSteeringMsgFromState2d(path_2.getState2d(0));
  robo_t.goal = orunav_conversions::createPoseSteeringMsgFromState2d(path_2.getState2d(path_2.sizePath()-1));
  robo_t.start_op.operation = robo_t.start_op.NO_OPERATION;
  robo_t.goal_op.operation = robo_t.goal_op.NO_OPERATION;
  robo_t.current_load.status = robo_t.current_load.EMPTY;
  robo_t.goal_load.status = robo_t.goal_load.EMPTY;
     
  robo_t.robot_id = 2;
  robo_t.task_id = 1;
  //robo_t.criticalPoint = -1;  
  //robo_t.path = orunav_conversions::createPathMsgFromPathInterface(path_2);
  //robo_t.update = false;
  //robo_t.abort = false;

  obs1_t.target.start = orunav_conversions::createPoseSteeringMsgFromState2d(path_1.getState2d(0));
  obs1_t.target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(path_1.getState2d(path_1.sizePath()-1));
  obs1_t.target.start_op.operation = obs1_t.target.start_op.NO_OPERATION;
  obs1_t.target.goal_op.operation = obs1_t.target.goal_op.NO_OPERATION;
  obs1_t.target.current_load.status = obs1_t.target.current_load.EMPTY;
  obs1_t.target.goal_load.status = obs1_t.target.goal_load.EMPTY;
     
  obs1_t.target.robot_id = 1;
  obs1_t.target.task_id = 1;
  obs1_t.criticalPoint = -1;  
  obs1_t.path = orunav_conversions::createPathMsgFromPathInterface(path_1);
  obs1_t.update = false;
  obs1_t.abort = false;
  // obs_t.push_back(obs1_t);

  srv.request.a = robo_t;
  srv.request.b = obs_t;  
  if(client.call(srv)) {
  	ROS_INFO("Got a task from GBP service");
  	orunav_generic::Path pt = orunav_conversions::createPathFromPathMsg(srv.response.c.path);
  	for (int i = 0; i<pt.sizePath(); ++i) 
  		cout<<pt.getPose2d(i)(0)<<" "<<pt.getPose2d(i)(1)<<" "<<pt.getPose2d(i)(2)<<" "<<pt.getSteeringAngle(i)<<endl;
  }

  else {
  	ROS_ERROR("Failed to call service GBP");
  	return 1;
  }

   size_t tt = 0;
  // orunav_generic::State2d st(1., 0., 0., 0.);
   orunav_generic::Path path_t;
  // State2d 
  // path_t.setState2d(st,tt);
  // cout<<"You setted: "<<path_t.getState2d(0)<<endl;

  // orunav_generic::Pose2d pp(1.5,2.5,0.5);
  // cout<<"1\n"<<path_t.sizePath()<<"\n";
  // path_t.setPose2d(pp,path_t.sizePath());
  // cout<<"2\n";
  // path_t.setSteeringAngle(10,tt);
  // cout<<"3\n";
  //cout<<path_t.getSteeringAngle(0)<<endl;
  //cout<<"You setted: "<<path_t.getPose2d(0)(0)<<" "<<path_t.getPose2d(0)(1)<<" "<<path_t.getPose2d(0)(2)<<endl;
  return 0;
}