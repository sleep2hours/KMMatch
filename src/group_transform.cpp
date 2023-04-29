#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <vector>
#include <ros/ros.h>
#include "KMMatch.h"
#include <visualization_msgs/Marker.h>
#include <plan_manage/ego_replan_fsm.h>

int main(int argc,char**argv) {
  ros::init(argc,argv,"group_transform");
  ros::NodeHandle nh("~");
  
  KMMatch::KMMatch group_match;
  group_match.init(nh);
  ros::spin();
  return 0;
}
