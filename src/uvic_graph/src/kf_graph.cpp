#include <vector>
#include <ros/ros.h>
#include <uvic_msgs/factor.h>
#include <uvic_msgs/kf_select.h>
#include <uvic_msgs/keyframe.h>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <boost/algorithm/string/split.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <boost/algorithm/string/classification.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace gtsam;

std::vector<uvic_msgs::keyframe> keyframes;

NonlinearFactorGraph graph;
noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr deltaNoise;

uvic_msgs::keyframe get_latest_keyframe() {
  return keyframes[keyframes.size() - 1];
}

void kf_issuer_keyframe_callback(const uvic_msgs::keyframe& input) {
  keyframes.push_back(input);
}

void kf_issuer_factor_callback(const uvic_msgs::factor& input) {
  //#############################
  //#########Insert Code#########
  //#############################

  //#############################
  //########Optimise Graph#######
  //#############################
}

bool kf_select(uvic_msgs::kf_select::Request &req, uvic_msgs::kf_select::Response &res) {
  if(keyframes.size() != 0) {
    res.KF_ref = get_latest_keyframe();
    return true;
  }

  return false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "/graph/kf_graph");
  ros::NodeHandle n;

  ros::Subscriber kf_issuer_factor_sub = n.subscribe("/kf_issuer/factor", 50, kf_issuer_factor_callback);
  ros::Subscriber kf_issuer_keyframe_sub = n.subscribe("/kf_issuer/keyframe", 50, kf_issuer_keyframe_callback);
  ros::ServiceServer kf_select_service = n.advertiseService("/graph/kf_select", kf_select);

  priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.3, 0.3, 0.1));
  deltaNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.1));

  ros::spin();
  return 0;
}
