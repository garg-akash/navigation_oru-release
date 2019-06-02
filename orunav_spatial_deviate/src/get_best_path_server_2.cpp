// #include <ros/ros.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <orunav_msgs/GetBestPath.h>
// #include <orunav_conversions/conversions.h>
// #include <orunav_generic/io.h>
// #include <orunav_generic/utils.h>
// #include <orunav_msgs/Task.h>

// #include <iostream>
// #include <fstream>
// #include <string>
// #include <vector>
// #define _USE_MATH_DEFINES   
// #include <math.h>
// #include <cmath>
// #include <algorithm>
// #include <casadi/casadi.hpp>

// #include <orunav_spatial_deviate/get_rectangle.h>
// #include <orunav_spatial_deviate/get_overlap.h>

// using namespace std;
// using namespace orunav_generic;
// using namespace orunav_msgs;

// 	float l_1 = 2.1;
// 	float b_1 = 0.6;
// 	float l_2 = 2.1;
// 	float b_2 = 0.6;
// 	float L = 1.19;
// 	float d_rear = L/2; 

// 	int matchPose(orunav_msgs::Task A, orunav_generic::Path P) {
//   	  double s_x = abs(A.target.start.pose.position.x - P.getPose2d(0)(0));
//   	  double s_y = abs(A.target.start.pose.position.y - P.getPose2d(0)(1));
//   	  double s_th = abs(tf::getYaw(A.target.start.pose.orientation) - P.getPose2d(0)(2));
//   	  double s_st = abs(A.target.start.steering - P.getSteeringAngle(0));

//   	  double g_x = abs(A.target.goal.pose.position.x - P.getPose2d(P.sizePath()-1)(0));
//   	  double g_y = abs(A.target.goal.pose.position.y - P.getPose2d(P.sizePath()-1)(1));
//   	  double g_th = abs(tf::getYaw(A.target.goal.pose.orientation) - P.getPose2d(P.sizePath()-1)(2));
//   	  double g_st = abs(A.target.goal.steering - P.getSteeringAngle(P.sizePath()-1)); 

//   	  if (s_x <= 0.5  && s_y <= 0.5 && s_th <= 1 && s_st <= 0.1
//   	  	  && g_x <= 0.5  && g_y <= 0.5 && g_th <= 1 && g_st <= 0.1) {
//   	  	return 1;
//   	  }
//   	  else
//   	  	return 0;
// 	}

// 	double cal_net_olp(int num, orunav_generic::Path p, std::vector<orunav_msgs::Task> t_all) {
// 	  double arr = 0; //It stores sum of overlaps of database path num with all current execution paths
// 	  double ar[t_all.size()] = {0}; //It stores overlaps of database path num with individual current execution paths
// 	  for(int i = 0; i < p.sizePath(); ++i) { //p is of type orunav_generic::Path
// 		  double** rect_A = rectangle_plot(l_2,b_2,p.getPose2d(i)(2),p.getPose2d(i)(0) + d_rear*cos(p.getPose2d(i)(2))
// 										,p.getPose2d(i)(1) + d_rear*sin(p.getPose2d(i)(2)));  	
// 		  for(int j = 0; j < t_all.size(); ++j) {
//         orunav_msgs::Task temp = t_all[j];
// 		    orunav_generic::Path p_t = orunav_conversions::createPathFromPathMsg(temp.path);
// 		    for(int m = 0; m < p_t.sizePath(); ++m) {	
// 		  	  double** rect_B = rectangle_plot(l_1,b_1,p_t.getPose2d(m)(2),p_t.getPose2d(m)(0) + d_rear*cos(p_t.getPose2d(m)(2))
// 										,p_t.getPose2d(m)(1) + d_rear*sin(p_t.getPose2d(m)(2)));  	
// 			    ar[j] = ar[j] + cal_overlap(rect_A,rect_B);	
// 		    }
// 		  }
// 	  }
// 	  cout<<"Overlap of Database Path "<<num<<" calculated with current executions!!!"<<endl;
// 	  for(int j = 0; j < t_all.size(); ++j) {
// 	  	arr += ar[j];
// 	  }
// 	  return arr;
// 	}

// 	bool bestpathCB(orunav_msgs::GetBestPath::Request &req,
//                      orunav_msgs::GetBestPath::Response &res) {
  
//   	  ROS_INFO("Obtained a request for best path");
  	  
//   	  orunav_generic::Path loaded_path_1 = loadPathTextFile("path20.txt"); //for now just two paths
//   	  orunav_generic::Path loaded_path_2 = loadPathTextFile("path21.txt");
//   	  std::vector<orunav_generic::Path> database_path;
//   	  database_path.push_back(loaded_path_1);
//   	  database_path.push_back(loaded_path_2);
//   	  ROS_INFO("Database of paths loaded");
//   	  cout<<"Size of Database: "<<database_path.size()<<endl;

//       Task ret;
//   	  std::vector<double> total_olp;
//   	  std::vector<double>::iterator it;
//   	  for(int i=0; i<database_path.size(); i++) {
//   	  	if (matchPose(req.a, database_path.at(i)) == 1) {
//   	  	  cout<<"Matched with Database Path "<<i<<endl;
//   	  	  total_olp.push_back(cal_net_olp(i, database_path.at(i), req.b)); 
//   	  	}
//   	  }
//   	  if(total_olp.size() == 0) {
//   	  	ROS_INFO("No path with matching pose found");
//         ret.path = req.a.path;
//       }
//   	  else {
//   	  	ROS_INFO("Path with matching pose found");
//   	  	it = std::min_element(total_olp.begin(), total_olp.end());
//   	  	cout<<"Minimum overlap from database path is: "<<*it<<endl;
//         cout<<"Minimum overlap path from database is: "<<std::distance(std::begin(total_olp), it)<<endl;
//         ret.path = orunav_conversions::createPathMsgFromPathInterface(database_path.at(std::distance(std::begin(total_olp), it)));
//   	  }

//       ret.target = req.a.target;
//       ret.criticalPoint = req.a.criticalPoint;
//       ret.criticalRobotID = req.a.criticalRobotID;
//       ret.releasingPoint = req.a.releasingPoint;
//       ret.constraints = req.a.constraints;
//       ret.dts = req.a.dts;
//       ret.cts = req.a.cts;
//       ret.update = req.a.update;
//       ret.abort = req.a.abort;

//       res.c = ret;

//   	  return true;
//   	}

// int main(int argc, char **argv) {
//   ros::init(argc, argv, "get_best_path_server");
//   ros::NodeHandle n;
//   ros::ServiceServer service = n.advertiseService("get_best_path", bestpathCB);
//   ROS_INFO("GBP service running");
//   ros::spin();

//   return 0;
// }