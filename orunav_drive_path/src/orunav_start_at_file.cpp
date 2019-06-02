#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include <orunav_conversions/conversions.h>

#include <boost/program_options.hpp>

#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>

#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

namespace po = boost::program_options;
using namespace std;
int main(int argc, char **argv)
{
  int robot_id_1, robot_id_2;
  ros::init(argc, argv, "SettingStartPose");
  ros::NodeHandle nh_;
  nh_.param<int>("robot_id1", robot_id_1, 1);
  nh_.param<int>("robot_id2", robot_id_2, 2);
  string file_name_1, file_name_2;
      po::options_description desc("Allowed options");
      desc.add_options()
          ("help", "produce help message")
          ("debug", "print debug output")
          ("fileName1", po::value<string>(&file_name_1)->required(), "file to be used 1")
          ("fileName2", po::value<string>(&file_name_2)->required(), "file to be used 2")
          ;

     po::variables_map vm;
     po::store(po::parse_command_line(argc, argv, desc), vm);
     po::notify(vm);
    cout << "Loading : " << file_name_1 << endl;
    cout << "Loading : " << file_name_2 << endl;
    orunav_generic::Path loaded_path_1 = orunav_generic::loadPathTextFile(file_name_1);
    orunav_generic::Path loaded_path_2 = orunav_generic::loadPathTextFile(file_name_2);

    if (loaded_path_1.sizePath() == 0) {
        cout << "no points in file 1 - exiting" << endl;
        exit(-1);
    }
    if (loaded_path_2.sizePath() == 0) {
        cout << "no points in file 2 - exiting" << endl;
        exit(-1);
    }

    cout << "First pose file 1: " << loaded_path_1.getPose2d(0) << endl;
    cout << "First pose file 2: " << loaded_path_2.getPose2d(0) << endl;

    ros::Rate loop_rate(0.5);

    geometry_msgs::PoseStamped p1, p2;
    p1.pose = orunav_conversions::createMsgFromPose2d(loaded_path_1.getPose2d(0));
    p2.pose = orunav_conversions::createMsgFromPose2d(loaded_path_2.getPose2d(0));
  ros::Publisher pub_1 = nh_.advertise<geometry_msgs::PoseStamped>(orunav_generic::getRobotTopicName(robot_id_1, "/goal"), 1);
  ros::Publisher pub_2 = nh_.advertise<geometry_msgs::PoseStamped>(orunav_generic::getRobotTopicName(robot_id_2, "/goal"), 1);
    while (ros::ok())
  {
  pub_1.publish(p1);
  pub_2.publish(p2);

  loop_rate.sleep();
}

  return 0;
}