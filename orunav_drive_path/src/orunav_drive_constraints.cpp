#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <orunav_msgs/PolygonConstraint.h>
#include <orunav_msgs/GetPolygonConstraints.h>
#include <orunav_conversions/conversions.h>
#include <orunav_generic/io.h>
#include <orunav_constraint_extract/polygon_constraint.h>
#include <orunav_constraint_extract/utils.h>
#include <orunav_constraint_extract/conversions.h>
#include <orunav_trajectory_processor/trajectory_processor_naive.h>
#include <orunav_msgs/GetPolygonConstraints.h>

#include <boost/program_options.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

namespace po = boost::program_options;
using namespace std;


void computeThBounds(const double &lb_orig, const double &ub_orig, const double &th, double &lb_new, double &ub_new) {
    // We know that the provided th should be the closest fit between the orignal bounds.
    // However, due to       orunav_generic::removeThNormalization(traj); the orientation angle are not between (0..2PI) here.
  lb_new = lb_orig;
  while (lb_new - th > 0) {
      lb_new -= 2*M_PI;
  }
  while (lb_new - th < -2*M_PI) {
      lb_new += 2*M_PI;
  }
  if (!(lb_new < th)) {
      cerr << "===ERROR===" << endl;
      cerr << "lb_new : " << lb_new << endl;
      cerr << "th : " << th << endl;
      assert(false);
  }
  assert(lb_new < th); // Check.

  ub_new = ub_orig;
  while (ub_new - th > 2*M_PI) {
      ub_new -= 2*M_PI;
  }
  while (ub_new - th < 0) {
      ub_new += 2*M_PI;
  }
  if (!(ub_new > th)) {
      cerr << "===ERROR===" << endl;
      cerr << "ub_new : " << ub_new << endl;
      cerr << "th : " << th << endl;
  }
  assert(ub_new > th); // Check.

}

int main(int argc, char **argv)
{
  int robot_id_;
  ros::init(argc, argv, "get_constraints");
  ros::NodeHandle nh_;
  nh_.param<int>("robot_id", robot_id_, 1);
  string path_file_name;
  po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    ("debug", "print debug output")
    ("fileName", po::value<string>(&path_file_name)->required(), "path file to be used")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  cout << "Loading : " << path_file_name << endl;
  orunav_generic::Path loaded_path = orunav_generic::loadPathTextFile(path_file_name);
  cout << "Loaded path sizes : " << loaded_path.sizePath() << endl;
  if (loaded_path.sizePath() == 0) {
      cout << "no points - exiting" << endl;
      exit(-1);
  }

  double T = 0.;
  orunav_generic::Trajectory traj_gen;
  {
  TrajectoryProcessorNaive gen;
  TrajectoryProcessor::Params p;
  p.maxVel = 0.5;
  p.maxAcc = 0.2;
  p.wheelBaseX = 1.19;
  gen.setParams(p);
  gen.addPathInterface(loaded_path);
  traj_gen = gen.getTrajectory(); 
  T = orunav_generic::getTotalTime(gen);
  }
  // Get the min / max time from the trajectory
      
  std::cout << "--------- Estimated total time T : " << T << " -----------" << std::endl;
  double d_t = T / loaded_path.sizePath();
  std::cout << "--------- Estimated delta time T : " << d_t << " -----------" << std::endl;

  double dt = 0.5;
  orunav_generic::Trajectory traj = orunav_generic::convertPathToTrajectoryWithoutModel(loaded_path, d_t);
  assert(orunav_generic::validPath(traj, M_PI));
  if (orunav_generic::validPath(traj, M_PI))
    std::cerr << "Non-normalized path(!) - should never happen" << std::endl;
  orunav_generic::removeThNormalization(traj);
  assert(orunav_generic::validPath(traj, M_PI));

  std::cout << "Saving trajectory txt file: " << std::endl;
  orunav_generic::saveTrajectoryTextFile(traj, "constraints_traj.txt");      

  boost::shared_ptr<nav_msgs::OccupancyGrid const> map_msg;
  nav_msgs::OccupancyGrid current_map_;
  map_msg  = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", nh_);
  if (map_msg != NULL) {
      current_map_ = *map_msg;
  }
  else {
    	cout<<"Empty content in map message"<<endl;
  }
	//constraint_extract::PolygonConstraintsVec current_constraints_;
	//orunav_msgs::GetPolygonConstraints srv_constraints;

	orunav_msgs::GetPolygonConstraints srv;

  orunav_msgs::RobotTarget target;
  target.start = orunav_conversions::createPoseSteeringMsgFromState2d(loaded_path.getState2d(0));
  target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(loaded_path.getState2d(loaded_path.sizePath()-1));
  target.start_op.operation = target.start_op.NO_OPERATION;
  target.goal_op.operation = target.goal_op.NO_OPERATION;
  target.current_load.status = target.current_load.EMPTY;
  target.goal_load.status = target.goal_load.EMPTY;
  target.start_earliest = ros::Time::now();
  target.robot_id = robot_id_;
  target.task_id = 1;
        
  srv.request.map = current_map_;
  srv.request.target = target;
  /*srv.request.path = orunav_conversions::createPathMsgFromPathAndState2dInterface(loaded_path,
                                                                                  orunav_conversions::createState2dFromPoseSteeringMsg(target.start),
                                                                                  orunav_conversions::createState2dFromPoseSteeringMsg(target.goal));*/
        
  srv.request.path = orunav_conversions::createPathMsgFromPathInterface(loaded_path);
  ros::ServiceClient client = nh_.serviceClient<orunav_msgs::GetPolygonConstraints>("/robot1/polygonconstraint_service");
  if (client.call(srv)) {
      ROS_INFO("[Akash Stuff] - polygonconstraint_service - successfull");
  }
  else {
      ROS_ERROR("[Akash Stuff] - Failed to call service: PolygonConstraint");
      return 1;
  }
	
  constraint_extract::PolygonConstraintsVec constraints = orunav_conversions::createPolygonConstraintsVecFromRobotConstraintsMsg(srv.response.constraints);
  if (constraints.size() != loaded_path.sizePath()) {
      ROS_ERROR("[Akash Stuff] - constraints size and path size differs");
      return 1;
  }

  cout<<"size of constraints : "<<constraints.size()<<endl;

  std::ofstream con_th("constraints_th.txt");
  std::ofstream con_A0("constraints_A0.txt");
  std::ofstream con_A1("constraints_A1.txt");
  std::ofstream con_b("constraints_b.txt");
  for (size_t i = 0; i < constraints.size(); i++) {
      double lb_th, ub_th;
      computeThBounds(constraints[i].getThBounds()[0], constraints[i].getThBounds()[1], loaded_path.getPose2d(i)(2), lb_th, ub_th);
      con_th << lb_th << " " << ub_th << endl;
      std::vector<double> A0, A1, b;
      constraints[i].getInnerConstraint().getMatrixFormAsVectors(A0, A1, b);
      size_t size = A0.size();
      for (size_t j = 0; j < size; j++) {
          con_A0 << A0[j] << " ";
          con_A1 << A1[j] << " ";
          con_b << b[j] << " ";
      }
      con_A0 << endl;
      con_A1 << endl;
      con_b << endl;
  }
  con_th.close();
  con_A0.close();
  con_A1.close();
  con_b.close();
                                                                              
  ros::spin();  
  return 0;
}