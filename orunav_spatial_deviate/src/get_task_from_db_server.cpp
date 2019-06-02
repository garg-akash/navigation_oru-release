#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <orunav_msgs/GetTaskFromDB.h>
#include <orunav_conversions/conversions.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>
#include <orunav_msgs/Task.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#define _USE_MATH_DEFINES   
#include <math.h>
#include <cmath>
#include <algorithm>

using namespace std;
using namespace orunav_generic;
using namespace orunav_msgs;

class TaskFromDB {
  private:
	ros::NodeHandle nh_;
	ros::ServiceServer service_;

  public:
  	TaskFromDB(ros::NodeHandle param_nh) {
	    service_ = nh_.advertiseService("get_task_from_db", &TaskFromDB::taskFromDBCB, this);
	  }

	int matchPose(orunav_msgs::RobotTarget A, orunav_generic::Path P) {
  	  double s_x = abs(A.start.pose.position.x - P.getPose2d(0)(0));
  	  double s_y = abs(A.start.pose.position.y - P.getPose2d(0)(1));
  	  double s_th = abs(tf::getYaw(A.start.pose.orientation) - P.getPose2d(0)(2));
  	  double s_st = abs(A.start.steering - P.getSteeringAngle(0));

  	  double g_x = abs(A.goal.pose.position.x - P.getPose2d(P.sizePath()-1)(0));
  	  double g_y = abs(A.goal.pose.position.y - P.getPose2d(P.sizePath()-1)(1));
  	  double g_th = abs(tf::getYaw(A.goal.pose.orientation) - P.getPose2d(P.sizePath()-1)(2));
  	  double g_st = abs(A.goal.steering - P.getSteeringAngle(P.sizePath()-1)); 

  	  if (s_x <= 0.5  && s_y <= 0.5 && s_th <= 1 && s_st <= 0.1
  	  	  && g_x <= 0.5  && g_y <= 0.5 && g_th <= 1 && g_st <= 0.1) {
  	  	return 1;
  	  }
  	  else
  	  	return 0;
	}

	bool taskFromDBCB(orunav_msgs::GetTaskFromDB::Request &req,
                     orunav_msgs::GetTaskFromDB::Response &res) {
  
  	  ROS_INFO("Obtained a request to get task from DB");
  	  
      orunav_generic::Path loaded_path_1 = loadPathTextFile("path10.txt"); //here we make sure that pathX0.txt in database is undeviated/pure path
      orunav_generic::Path loaded_path_2 = loadPathTextFile("path11.txt");
      orunav_generic::Path loaded_path_3 = loadPathTextFile("path12.txt");
  	  orunav_generic::Path loaded_path_4 = loadPathTextFile("path20.txt"); 
  	  orunav_generic::Path loaded_path_5 = loadPathTextFile("path21.txt");
      orunav_generic::Path loaded_path_6 = loadPathTextFile("path22.txt");

  	  std::vector<orunav_generic::Path> database_path;
  	  database_path.push_back(loaded_path_1);
  	  database_path.push_back(loaded_path_2);
      database_path.push_back(loaded_path_3);
      database_path.push_back(loaded_path_4);
      database_path.push_back(loaded_path_5);
      database_path.push_back(loaded_path_6);

  	  ROS_INFO("Database of paths loaded");
  	  cout<<"Size of Database: "<<database_path.size()<<endl;

      int flag = 0;
      Task ret;
  	  for(int i=0; i<database_path.size(); i++) {
  	  	if (matchPose(req.tr, database_path.at(i)) == 1) {
  	  	  cout<<"Matched with Database Path "<<i<<endl;
          ret.target = req.tr;
          ret.path = orunav_conversions::createPathMsgFromPathInterface(database_path.at(i));
          ret.target.start = orunav_conversions::createPoseSteeringMsgFromState2d(database_path.at(i).getState2d(0));
          ret.target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(database_path.at(i).getState2d(database_path.at(i).sizePath()-1));

          flag = 1;
          res.ts = ret;
          break;
  	  	}
  	  }

  	  if(flag == 0) {
  	  	ROS_INFO("No path with matching pose found");
        return false;
      }

  	  return true;
  	}
  };



int main(int argc, char **argv) {
  ros::init(argc, argv, "get_task_from_db_server");
  ros::NodeHandle parameters("~");
  ROS_INFO("GTFDB service running");
  TaskFromDB tfdb(parameters);
  ros::spin();
}