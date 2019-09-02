#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <orunav_msgs/GetBestPath.h>
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
#include <casadi/casadi.hpp>

#include <orunav_spatial_deviate/get_rectangle.h>
#include <orunav_spatial_deviate/get_overlap.h>

using namespace std;
using namespace orunav_generic;
using namespace orunav_msgs;

class BestPath {
  private:
	ros::NodeHandle nh_;
	ros::ServiceServer service_;
	float l_1 = 2.1;
	float b_1 = 0.6;
	float l_2 = 2.1;
	float b_2 = 0.6;
	float L = 1.19;
	float d_rear = L/2; 
  std::string bestpath_database_dir_;
  //VehicleState vehicle_state_;

  public:
  	BestPath(ros::NodeHandle param_nh) {
      param_nh.param<std::string>("bestpath_database_directory", bestpath_database_dir_, "./BPDatabase/");
	    service_ = nh_.advertiseService("get_best_path", &BestPath::bestpathCB, this);
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

  	  if (s_x <= 1.5  && s_y <= 1.5 && s_th <= 5 && s_st <= 2.1
  	  	  && g_x <= 1.5  && g_y <= 1.5 && g_th <= 5 && g_st <= 2.1) {
  	  	return 1;
  	  }
  	  else
  	  	return 0;
	}

	double cal_net_olp(int num, orunav_generic::Path p, std::vector<orunav_msgs::Task> t_all) {
	  double arr = 0; //It stores sum of overlaps of database path num with all current execution paths
	  double ar[t_all.size()] = {0}; //It stores overlaps of database path num with individual current execution paths
	  for(int i = 0; i < p.sizePath(); ++i) { //p is of type orunav_generic::Path
		  double** rect_A = rectangle_plot(l_2,b_2,p.getPose2d(i)(2),p.getPose2d(i)(0) + d_rear*cos(p.getPose2d(i)(2))
										,p.getPose2d(i)(1) + d_rear*sin(p.getPose2d(i)(2)));  	
		  for(int j = 0; j < t_all.size(); ++j) {
        orunav_msgs::Task temp = t_all[j];
		    orunav_generic::Path p_t = orunav_conversions::createPathFromPathMsg(temp.path);
		    for(int m = 0; m < p_t.sizePath(); ++m) {	
		  	  double** rect_B = rectangle_plot(l_1,b_1,p_t.getPose2d(m)(2),p_t.getPose2d(m)(0) + d_rear*cos(p_t.getPose2d(m)(2))
										,p_t.getPose2d(m)(1) + d_rear*sin(p_t.getPose2d(m)(2)));  	
			    ar[j] = ar[j] + cal_overlap(rect_A,rect_B);	
		    }
		  }
	  }
	  cout<<"Overlap of Database Path "<<num + 1<<" calculated with current executions!!!"<<endl;
	  for(int j = 0; j < t_all.size(); ++j) {
	  	arr += ar[j];
	  }
	  return arr;
	}

	bool bestpathCB(orunav_msgs::GetBestPath::Request &req,
                     orunav_msgs::GetBestPath::Response &res) {
  
  	  ROS_INFO("Obtained a request for best path");
  	  
  	  orunav_generic::Path loaded_path_1 = loadPathTextFile(bestpath_database_dir_ + "path11.txt"); //here we make sure that pathXX.txt in database is undeviated/pure path
      orunav_generic::Path loaded_path_2 = loadPathTextFile(bestpath_database_dir_ + "path12.txt"); //pathXY.txt is path of robotX deviated wrt robotY
      orunav_generic::Path loaded_path_3 = loadPathTextFile(bestpath_database_dir_ + "path21.txt"); 
  	  orunav_generic::Path loaded_path_4 = loadPathTextFile(bestpath_database_dir_ + "path22.txt");


  	  std::vector<orunav_generic::Path> database_path;
  	  database_path.push_back(loaded_path_1);
  	  database_path.push_back(loaded_path_2);
      database_path.push_back(loaded_path_3);
      database_path.push_back(loaded_path_4);

  	  ROS_INFO_STREAM("[Get Best Path] - Database of paths loaded from : " << bestpath_database_dir_ <<endl);
  	  cout<<"Size of Database: "<<database_path.size()<<endl;
      //req.a.start = orunav_conversions::createPoseSteeringMsgFromState2d(vehicle_state_.getCurrentState2d());
      cout<<"Request came for: ("<<req.a.start.pose.position.x<<", "<<req.a.start.pose.position.y<<", "<<tf::getYaw(req.a.start.pose.orientation)<<", "<<req.a.start.steering<<") ";
      cout<<" to ("<<req.a.goal.pose.position.x<<", "<<req.a.goal.pose.position.y<<", "<<tf::getYaw(req.a.goal.pose.orientation)<<", "<<req.a.goal.steering<<")"<<endl;

      int f_match = 0;
      int f_empty = 0;
      
      Task ret;
  	  std::vector<double> total_olp;
  	  std::vector<double>::iterator it;
  	  for(int i=0; i<database_path.size(); i++) {
        if (matchPose(req.a, database_path.at(i)) != 1)
          total_olp.push_back(100000);
  	  	else {
  	  	  cout<<"Matched with Database Path "<<i+1<<endl;
          f_match = 1;
          if(req.b.size() == 0) {
            cout<<"Execution task vector is empty"<<endl;
            ret.target = req.a;

            ret.target.start_op.operation = ret.target.start_op.NO_OPERATION;
            ret.target.goal_op.operation = ret.target.goal_op.NO_OPERATION;
            ret.target.current_load.status = ret.target.current_load.EMPTY;
            ret.target.goal_load.status = ret.target.goal_load.EMPTY;
            ret.target.task_id = 1;

            ret.path = orunav_conversions::createPathMsgFromPathInterface(database_path.at(i));
            ret.target.start = orunav_conversions::createPoseSteeringMsgFromState2d(database_path.at(i).getState2d(0));
            ret.target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(database_path.at(i).getState2d(database_path.at(i).sizePath()-1));

            ret.criticalPoint = -1;  
            ret.update = false;
            ret.abort = false;

            res.c = ret;

            f_empty = 1;
            //break;
            return true;
          }
          cout<<"Execution task vector size is: "<<req.b.size()<<endl;
          for(int p=0; p<req.b.size(); p++) {
            cout<<"Execution task vector robotID "<< req.b[p].target.robot_id<<" with pathLength "<<orunav_conversions::createPathFromPathMsg(req.b[p].path).sizePath()<<endl;
          }
  	  	  total_olp.push_back(cal_net_olp(i, database_path.at(i), req.b)); 
  	  	}
  	  }

      if(f_empty == 0) {
  	  if(f_match == 0) {
  	  	ROS_INFO("No path with matching pose found in Database");
        return false;
      }
  	  else {
  	  	cout<<"Overlap with maching pose path are as follows:"<<endl;
        for(int j = 0; j<total_olp.size(); ++j) 
          cout<<total_olp.at(j)<<endl;
  	  	it = std::min_element(total_olp.begin(), total_olp.end());
  	  	cout<<"Minimum overlap from database path is: "<<*it<<endl;
        cout<<"Minimum overlap path from database is: "<<std::distance(std::begin(total_olp), it) + 1<<endl;
        ret.target = req.a;
        orunav_generic::Path p_db = database_path.at(std::distance(std::begin(total_olp), it));
        ret.path = orunav_conversions::createPathMsgFromPathInterface(p_db);
  	    ret.target.start = orunav_conversions::createPoseSteeringMsgFromState2d(p_db.getState2d(0));
        ret.target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(p_db.getState2d(p_db.sizePath()-1));

        ret.criticalPoint = -1;  
        ret.update = false;
        ret.abort = false;

        res.c = ret;
        return true;
      }
  	}
    }
  };



int main(int argc, char **argv) {
  ros::init(argc, argv, "get_best_path_server");
  ros::NodeHandle parameters("~");
  ROS_INFO("GBP service running");
  BestPath bp(parameters);
  ros::spin();
}