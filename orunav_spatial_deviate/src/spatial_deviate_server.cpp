//This code takes hell lot of time....take popcorn, sit back and enjoy!!!!
#include <ros/ros.h>
#include <orunav_msgs/SpatialDeviate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <orunav_msgs/PolygonConstraint.h>
#include <orunav_msgs/GetPolygonConstraints.h>
#include <orunav_conversions/conversions.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>
#include <orunav_generic/functions.h>
#include <orunav_trajectory_processor/trajectory_processor_naive.h>
#include <orunav_constraint_extract/polygon_constraint.h>
#include <orunav_constraint_extract/utils.h>
#include <orunav_constraint_extract/conversions.h>
#include <orunav_msgs/GetPolygonConstraints.h>

#include <boost/program_options.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#define _USE_MATH_DEFINES   
#include <math.h>
#include <algorithm>
#include <casadi/casadi.hpp>

#include <orunav_spatial_deviate/get_rectangle.h>
#include <orunav_spatial_deviate/get_overlap.h>
#include <orunav_spatial_deviate/get_sptl_nlp.h>

using namespace casadi;
namespace po = boost::program_options;
using namespace std;

class GetSpatialDeviation {
	private:
		ros::NodeHandle nh_;
		ros::ServiceServer service_;
    int robot_id_;
    float L, l_1, b_1, l_2, b_2;

	public:
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

GetSpatialDeviation(ros::NodeHandle &param_nh) {
  param_nh.param<int>("robot_id_", robot_id_, 1);
  param_nh.param<float>("wheel_base_x", L, 1.19);
  param_nh.param<float>("l_1", l_1, 2.1);
  param_nh.param<float>("b_1", b_1, 0.6);
  param_nh.param<float>("l_2", l_2, 2.1);
  param_nh.param<float>("b_2", b_2, 0.6);
	service_ = nh_.advertiseService("get_spatial_deviation", &GetSpatialDeviation::spatialDeviateCB, this);
}

bool spatialDeviateCB(orunav_msgs::SpatialDeviate::Request &req,
                     orunav_msgs::SpatialDeviate::Response &res) {
  
  ROS_INFO("Obtained a request for a spatial deviation");
  //float del_t = 0.5;
  int N_init = 9; //initial horizon 
  int n_area = 3; //points to be considered for overlap calculation for each horizon
  //int robot_id_ = 2;
  int nx = 4;
  int nu = 2; // Number of control segments
  int discard = int(N_init/2); //int(9/2)=4
  discard = 5; //just for N9
  //int discard = N_init-1;
  float d_rear = L/2;
  vector<double> robo_start, robo_vel, robo_omg, robo_th, robo_p;
  boost::shared_ptr<nav_msgs::OccupancyGrid const> map_msg;
  nav_msgs::OccupancyGrid current_map_;
  map_msg  = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", nh_);
  if (map_msg != NULL) {
    current_map_ = *map_msg;
  }
  else {
    cout<<"Empty content in map message"<<endl;
  }
  orunav_generic::Path pathIn1 = orunav_conversions::createPathFromPathMsg(req.path1);
  orunav_generic::Path pathIn2 = orunav_conversions::createPathFromPathMsg(req.path2);

  double x1_ref[pathIn1.sizePath()], y1_ref[pathIn1.sizePath()], th1_ref[pathIn1.sizePath()], p1_ref[pathIn1.sizePath()];
  for (unsigned int i = 0; i < pathIn1.sizePath(); i++) {
    x1_ref[i] = pathIn1.getPose2d(i)(0);
    y1_ref[i] = pathIn1.getPose2d(i)(1);
    th1_ref[i] = pathIn1.getPose2d(i)(2);
	  p1_ref[i] = pathIn1.getSteeringAngle(i);
  }	  
  //obtain trajectory from path
  double x2_ref[pathIn2.sizePath()+N_init], y2_ref[pathIn2.sizePath()+N_init], th2_ref[pathIn2.sizePath()+N_init]
  		, p2_ref[pathIn2.sizePath()+N_init], v2_ref[pathIn2.sizePath()+N_init], omg2_ref[pathIn2.sizePath()+N_init];

  double T = 0.;
  orunav_generic::Trajectory traj_gen;
  {
  TrajectoryProcessorNaive gen;
  TrajectoryProcessor::Params p;
  p.maxVel = 0.5;
  p.maxAcc = 0.2;
  p.wheelBaseX = L;
  gen.setParams(p);
  gen.addPathInterface(pathIn2);
  traj_gen = gen.getTrajectory(); 
  T = orunav_generic::getTotalTime(gen);
  }
  double del_t = T/pathIn2.sizePath(); 

  orunav_generic::Trajectory traj = orunav_generic::convertPathToTrajectoryWithoutModel(pathIn2, del_t);
  assert(orunav_generic::validPath(traj, M_PI));
  if (orunav_generic::validPath(traj, M_PI))
  std::cerr << "Non-normalized path(!) - should never happen" << std::endl;
  orunav_generic::removeThNormalization(traj);
  assert(orunav_generic::validPath(traj, M_PI));

  for (unsigned int i = 0; i < traj.sizePath()+N_init; i++) {
  	if(i < traj.sizePath()) {
      x2_ref[i] = traj.getPose2d(i)(0);
      y2_ref[i] = traj.getPose2d(i)(1);
      th2_ref[i] = traj.getPose2d(i)(2);
	    p2_ref[i] = traj.getSteeringAngle(i);
	    v2_ref[i] = traj.getDriveVel(i);
	    omg2_ref[i] = traj.getSteeringVel(i);
	  }
	  else {
	    x2_ref[i] = x2_ref[traj.sizePath()-1];
      y2_ref[i] = y2_ref[traj.sizePath()-1];
      th2_ref[i] = th2_ref[traj.sizePath()-1];
	    p2_ref[i] = p2_ref[traj.sizePath()-1];
	    v2_ref[i] = v2_ref[traj.sizePath()-1];
	    omg2_ref[i] = omg2_ref[traj.sizePath()-1];
	  }
  }	  

 //obtain constraints
  double cons_A0[pathIn2.sizePath()+N_init][4], cons_A1[pathIn2.sizePath()+N_init][4]
  		, cons_b[pathIn2.sizePath()+N_init][4], cons_th[pathIn2.sizePath()+N_init][2];
  orunav_msgs::GetPolygonConstraints srv;
  orunav_msgs::RobotTarget target;
  target.start = orunav_conversions::createPoseSteeringMsgFromState2d(pathIn2.getState2d(0));
  target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(pathIn2.getState2d(pathIn2.sizePath()-1));
  target.start_op.operation = target.start_op.NO_OPERATION;
  target.goal_op.operation = target.goal_op.NO_OPERATION;
  target.current_load.status = target.current_load.EMPTY;
  target.goal_load.status = target.goal_load.EMPTY;
  target.start_earliest = ros::Time::now();
  target.robot_id = robot_id_;
  target.task_id = 1;
        
  srv.request.map = current_map_;
  srv.request.target = target;
  srv.request.path = req.path2;
       
  ros::ServiceClient client = nh_.serviceClient<orunav_msgs::GetPolygonConstraints>(orunav_generic::getRobotTopicName(robot_id_, "/polygonconstraint_service"));
  if (client.call(srv)) {
      ROS_INFO("[Akash Stuff] - polygonconstraint_service - successfull");
  }
  else {
      ROS_ERROR("[Akash Stuff] - Failed to call service: PolygonConstraint");
      return 1;
  }
	
  constraint_extract::PolygonConstraintsVec constraints = orunav_conversions::createPolygonConstraintsVecFromRobotConstraintsMsg(srv.response.constraints);
  if (constraints.size() != pathIn2.sizePath()) {
      ROS_ERROR("[Akash Stuff] - constraints size and path size differs");
      return 1;
  }

  cout<<"size of constraints : "<<constraints.size()<<endl;
  size_t sz;
  for (size_t i = 0; i < constraints.size()+N_init; i++) {
  	if(i < constraints.size()) {
      double lb_th, ub_th;
      computeThBounds(constraints[i].getThBounds()[0], constraints[i].getThBounds()[1], pathIn2.getPose2d(i)(2), lb_th, ub_th);
      cons_th[i][0] = lb_th;
      cons_th[i][1] = ub_th;
      std::vector<double> A0, A1, b;
      constraints[i].getInnerConstraint().getMatrixFormAsVectors(A0, A1, b);
      sz = A0.size();
      for (size_t j = 0; j < sz; j++) {
        cons_A0[i][j] = A0[j];
        cons_A1[i][j] = A1[j];
        cons_b[i][j] = b[j];
      }
  	}
  	else {
      cons_th[i][0] = cons_th[constraints.size()-1][0];
      cons_th[i][1] = cons_th[constraints.size()-1][1];
  	  for (size_t j = 0; j < sz; j++) {
  	  	cons_A0[i][j] = cons_A0[constraints.size()-1][j];
      	cons_A1[i][j] = cons_A1[constraints.size()-1][j];
      	cons_b[i][j] = cons_b[constraints.size()-1][j];
  	  }	
  	}
  }

  for (size_t i = 0; i < (pathIn2.sizePath()+N_init); ++i) { //constraints on th needs to be fixed
    if (th2_ref[i] > M_PI) {
      cons_th[i][0] += 2*M_PI;
      cons_th[i][1] += 2*M_PI;
    }
    else if (th2_ref[i] < -M_PI) {
      cons_th[i][0] -= 2*M_PI;
      cons_th[i][1] -= 2*M_PI;
    }
  }
   //Exciting stuff starts here
  float sum[pathIn1.sizePath()];
  double Ar[pathIn1.sizePath()][pathIn2.sizePath()]; 
  int Nb = pathIn2.sizePath();
  vector<int> ind, ind_or; 
  for (int i = 0; i < pathIn1.sizePath(); ++i) {
    //cout<<"i : "<<i<<endl;
    sum[i] = 0;
    double** rect_A = rectangle_plot(l_1,b_1,th1_ref[i],x1_ref[i] + d_rear*cos(th1_ref[i]),y1_ref[i] + d_rear*sin(th1_ref[i]));

    for (int j = 0; j < Nb; ++j) {
      double** rect_B = rectangle_plot(l_2,b_2,th2_ref[j],x2_ref[j] + d_rear*cos(th2_ref[j]),y2_ref[j] + d_rear*sin(th2_ref[j]));
      
      Ar[i][j] = cal_overlap(rect_A,rect_B);
      sum[i] += Ar[i][j];
      //cout<<Ar[i][j]<<" ";
    }
    if (sum[i] > 0) {
      ind.push_back(i);
    }
    cout<<endl;
  }
  ind_or = ind;
  cout<<"ind is:"<<endl;
  for (auto i: ind)
    cout << i << ' ';
  
  double dist_st, dist_ed;
  if (ind.size() > 0) {
    dist_st = sqrt(pow( (x2_ref[0] - x1_ref[ind[0]]),2 ) + pow( (y2_ref[0] - y1_ref[ind[0]]),2 ));
    dist_ed = sqrt(pow( (x2_ref[0] - x1_ref[ind.back()]),2 ) + pow( (y2_ref[0] - y1_ref[ind.back()]),2 ));
    if (dist_ed < dist_st) {
      reverse(ind_or.begin(), ind_or.end());
      cout<<"\nReversing "<<ind_or.size()<<endl;
    }
  }
  else {
    cout<<"No ind"<<endl;
    ind_or.clear();
    ind_or.push_back(1);
  }
  cout<<endl<<dist_st<<" "<<dist_ed<<endl;
  cout<<"ind_or is:"<<endl;
  for (auto i: ind_or)
    cout << i << ' ';

  double rad_2 = sqrt(pow(l_2,2)+pow(b_2,2))/2;
  double rad_1 = sqrt(pow(l_1,2)+pow(b_1,2))/2;

  int ind_ob = 0;
  int ind_rb;
  int ind_left = 0;
  int flag1[N_init] = {0};
  double dist1, dist2, dist3;
  robo_start.push_back(x2_ref[0]);
  robo_start.push_back(y2_ref[0]);
  robo_start.push_back(th2_ref[0]);
  robo_start.push_back(p2_ref[0]);
  vector<vector<double>> robo_traj(1, robo_start);
  robo_vel.push_back(v2_ref[0]);
  robo_omg.push_back(omg2_ref[0]);

  MX vel_pichla, omg_pichla;

  ofstream file;
  string filename = "opt_trajectory_results.txt";
  file.open(filename.c_str());
  file <<x2_ref[0] << " "<<y2_ref[0]<< " "<<th2_ref[0]<< " "<<p2_ref[0]<<endl;

  ofstream fvel;
  string fvelname = "opt_control_results.txt";
  fvel.open(fvelname.c_str());
  fvel <<v2_ref[0] << " "<<omg2_ref[0]<<endl;
  int N, k;
  double x1_ref_or[ind_or.size()], y1_ref_or[ind_or.size()], th1_ref_or[ind_or.size()]; //to get agent 1 pose in ordered format
  auto start = chrono::steady_clock::now();
  for (k = 0; ; k+=(N-discard)) {
  //for (int k = 0; k <= pathIn2.sizePath()/2; ++k) {
    cout<<"Sim no.........................: "<<k<<endl;
    //if (k<=pathIn2.sizePath()/2 - 1) {
      N = N_init;

      for (int i = 0; i < N; ++i) {
        flag1[i] = 0;
        ind_rb = i;
        for (int j = 0; j < ind_or.size(); ++j) {
          dist1 = sqrt(pow( (x2_ref[k+i] - x1_ref[ind_or[j]]),2 ) + pow( (y2_ref[k+i] - y1_ref[ind_or[j]]),2 )) - rad_1 - rad_2;
          dist2 = sqrt(pow( (x2_ref[k+i] - x2_ref[Nb+N]),2 ) + pow( (y2_ref[k+i] - y2_ref[Nb+N]),2 ));
          dist3 = sqrt(pow( (x2_ref[Nb+N] - x1_ref[ind_or[j]]),2 ) + pow( (y2_ref[Nb+N] - y1_ref[ind_or[j]]),2 ));
          if( dist1 < 0.1 && dist3 <= dist2) {
            for (int lil = i; lil < N; ++lil ) 
              flag1[lil] = 1;
            ind_ob = j;
            ind_left = ind_or.size() - ind_ob + 1;
            break;
          }
        }
        if (flag1[i] == 1)
          break; 
      } 
    //}

    // else
    //   N = pathIn2.sizePath() - k; //pathIn2.sizePath()39 k19 ie N = 20

    vel_pichla = robo_vel[k];
    omg_pichla = robo_omg[k];

    cout<<"ind_ob: "<<ind_ob<<endl;
    for (int i = 0; i < N; ++i) 
      cout<<flag1[i]<<endl;

    std::vector<double> Px,Py,V_opt,Omg_opt,P_opt,Th_opt;

    for (int i = 0; i < ind_or.size(); ++i) {
      x1_ref_or[i] = x1_ref[ind_or[i]];
      y1_ref_or[i] = y1_ref[ind_or[i]];
      th1_ref_or[i] = th1_ref[ind_or[i]];
    }
    ind_left = 0;
    //TODO opt function
    oru_splt_dev(Px, Py, V_opt, Omg_opt, P_opt, Th_opt, k, pathIn2.sizePath(), N, n_area, discard, del_t, robo_start, x1_ref_or, y1_ref_or, th1_ref_or
                , x2_ref, y2_ref, th2_ref, p2_ref, v2_ref, omg2_ref, l_1, b_1, l_2, b_2, L, d_rear, flag1, ind_ob, ind_or, ind_left
                , cons_A0, cons_A1, cons_b, cons_th, vel_pichla, omg_pichla, Nb); 
    robo_start.clear();
    robo_start.push_back(Px[N-discard]); 
    robo_start.push_back(Py[N-discard]);
    robo_start.push_back(Th_opt[N-discard]);
    robo_start.push_back(P_opt[N-discard]);
    //if (k<=pathIn2.sizePath()/2 - 1) {
    for (int i = 1; i < (N-discard+1); ++i) {
      vector<double> traj;
      traj.push_back(Px[i]); 
      traj.push_back(Py[i]);
      traj.push_back(Th_opt[i]);
      traj.push_back(P_opt[i]);
      robo_traj.push_back(traj);
      robo_vel.push_back(V_opt[i-1]);
      robo_omg.push_back(Omg_opt[i-1]);  
      robo_p.push_back(P_opt[i-1]);
      robo_th.push_back(Th_opt[i-1]);

      //v2_ref[k+N-discard-1+i] = V_opt[N-discard-1+i];
      //omg2_ref[k+N-discard-1+i] = Omg_opt[N-discard-1+i];

      file << Px[i] << " "<<Py[i]<< " "<<Th_opt[i]<< " "<<P_opt[i]<<endl;
      fvel << V_opt[i-1] << " "<<Omg_opt[i-1]<<endl;
    }
    //}
    // else {
    //   for (int i = 1; i <= N; ++i) {
    //     file << Px[i] << ","<<Py[i]<< ","<<Th_opt[i]<< ","<<P_opt[i]<<";"<<endl;   
    //     fvel << V_opt[i-1] << ","<<Omg_opt[i-1]<<";"<<endl;
    //   }
    // }
    cout<<"Discard "<<discard<<endl;
    for (auto i: robo_traj)
      cout << i << endl;

    cout<<"Next Start "<<robo_start<<endl;
    if ( k > (pathIn2.sizePath()/2-(N-discard)) )
      break;
    // if (k == 0)
    //   break;
  }
  k = k + N - discard;
  cout<<"Sim no.........................Exit loop: "<<k<<endl;
  N = pathIn2.sizePath() - k;
  vel_pichla = robo_vel[k];
  omg_pichla = robo_omg[k];
  std::vector<double> Px,Py,V_opt,Omg_opt,P_opt,Th_opt;
  oru_splt_dev(Px, Py, V_opt, Omg_opt, P_opt, Th_opt, k, pathIn2.sizePath(), N, n_area, discard, del_t, robo_start, x1_ref_or, y1_ref_or, th1_ref_or
              , x2_ref, y2_ref, th2_ref, p2_ref, v2_ref, omg2_ref, l_1, b_1, l_2, b_2, L, d_rear, flag1, ind_ob, ind_or, ind_left
              , cons_A0, cons_A1, cons_b, cons_th, vel_pichla, omg_pichla, Nb); 
  for (int i = 1; i <= N; ++i) {
    file << Px[i] << " "<<Py[i]<< " "<<Th_opt[i]<< " "<<P_opt[i]<<endl;   
    fvel << V_opt[i-1] << " "<<Omg_opt[i-1]<<endl;
  }

  auto end = chrono::steady_clock::now();
  auto diff = end - start;
  cout << chrono::duration <double, milli> (diff).count() << " ms" << endl;
  file.close();
  fvel.close();  

  orunav_generic::Path p_ret = orunav_generic::loadPathTextFile("opt_trajectory_results.txt");
  res.sptl_path2 = orunav_conversions::createPathMsgFromPathInterface(p_ret);
  res.sptl_path1 = req.path1;

  return true;
}
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "get_spatial_deviation_server");
  ros::NodeHandle parameters("~");
  GetSpatialDeviation gsd(parameters);
  //ros::ServiceServer service = nh_.advertiseService("get_spatial_deviation", spatialDeviateCB);
  ros::spin();
}
