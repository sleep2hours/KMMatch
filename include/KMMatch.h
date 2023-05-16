#ifndef GROUP_TRANSFORM_H_
#define GROUP_TRANSFORM_H_
#include "hungarian_optimizer.h"
#include "quadrotor_msgs/PositionCommand.h"
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <vector>
using namespace message_filters;
typedef sync_policies::ApproximateTime<
    quadrotor_msgs::PositionCommand, quadrotor_msgs::PositionCommand,
    quadrotor_msgs::PositionCommand, quadrotor_msgs::PositionCommand,
    quadrotor_msgs::PositionCommand>
    MySyncPolicy;
namespace KMMatch {
class KMMatch {
private:
  int drone_num;
  bool target_receive, first_init;
  std::vector<Eigen::Vector3d> present_pos;
  std::vector<Eigen::Vector3d> target_pos;
  std::vector<message_filters::Subscriber<quadrotor_msgs::PositionCommand> *>
      pos_sub;

  HungarianOptimizer<double> optimizer_;

  std::vector<ros::Publisher> goal_puber;
  ros::Subscriber target_suber;
  Synchronizer<MySyncPolicy> *sync;

  void OdomCallback(const quadrotor_msgs::PositionCommandConstPtr &msg1,
                    const quadrotor_msgs::PositionCommandConstPtr &msg2,
                    const quadrotor_msgs::PositionCommandConstPtr &msg3,
                    const quadrotor_msgs::PositionCommandConstPtr &msg4,
                    const quadrotor_msgs::PositionCommandConstPtr &msg5);
  void TargetCallback(const geometry_msgs::PoseStampedPtr &msg);

public:
  KMMatch() {}
  ~KMMatch() {}
  void init(ros::NodeHandle &nh);
  void match();
};
} // namespace KMMatch
#endif
