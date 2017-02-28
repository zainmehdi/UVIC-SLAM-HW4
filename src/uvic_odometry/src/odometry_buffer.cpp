#include <vector>
#include <ros/ros.h>
#include <uvic_msgs/odometry.h>
#include <tf/transform_broadcaster.h>
#include <uvic_msgs/odometry_buffer.h>

ros::Publisher odometry_buffer_pub;
std::vector<uvic_msgs::odometry> odometry_buffer;

geometry_msgs::PoseWithCovariance pose_transform(geometry_msgs::PoseWithCovariance t_start_pose,
						 geometry_msgs::PoseWithCovariance t_end_pose) {
  geometry_msgs::PoseWithCovariance transform;
  double t_start_th = tf::getYaw(t_start_pose.pose.orientation);
  double t_end_th = tf::getYaw(t_start_pose.pose.orientation);
  double cos_th = cos(t_start_th);
  double sin_th = sin(t_start_th);
  double dx = t_end_pose.pose.position.x - t_start_pose.pose.position.x;
  double dy = t_end_pose.pose.position.y - t_start_pose.pose.position.y;
  double dth = t_end_th - t_start_th;
  dth = std::fmod(dth + M_PI, 2 * M_PI) - M_PI;
  transform.pose.position.x = ( cos_th * dx ) + ( sin_th * dy );
  transform.pose.position.y = ( -sin_th * dx ) + ( cos_th *dy );
  transform.pose.orientation = tf::createQuaternionMsgFromYaw(dth);
  
  return transform;
}

void odometry_buffer_callback(const uvic_msgs::odometry& input) {
  if(odometry_buffer.size() > 500) {
    odometry_buffer.clear();
  }
  
  odometry_buffer.push_back(input);
}

bool odometry_buffer_request(uvic_msgs::odometry_buffer::Request &req, uvic_msgs::odometry_buffer::Response &res) {
  int buffer_search_position = 0;
  int t_start_buffer_position = 0;
  int t_end_buffer_position = 0;
  int t_start = (int) req.t_start.toSec();
  int t_end = (int) req.t_end.toSec();
  bool t_start_found = false;
  bool t_end_found = false;
  std::vector<uvic_msgs::odometry> odometry_buffer_frozen = odometry_buffer;
  
  for(int i = 0; i < odometry_buffer.size(); i++) {
    buffer_search_position = (int) odometry_buffer_frozen[i].TS.toSec();
    
    if(buffer_search_position == t_start) {
      t_start_buffer_position = i;
      t_start_found = true;
    }

    if(buffer_search_position == t_end) {
      t_end_buffer_position = i;
      t_end_found = true;
    }
  }

  if(t_start_found && t_end_found) {
    geometry_msgs::PoseWithCovariance t_start_pose = odometry_buffer_frozen[t_start_buffer_position].Delta;
    geometry_msgs::PoseWithCovariance t_end_pose = odometry_buffer_frozen[t_end_buffer_position].Delta;
    res.delta = pose_transform(t_start_pose, t_end_pose);
    return true;
  }
  
  return false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_buffer");
  ros::NodeHandle n;

  ros::Subscriber odometry_sub = n.subscribe("/odometry_integrator/odometry", 50, odometry_buffer_callback);
  ros::ServiceServer odometry_buffer_service = n.advertiseService("odometry_buffer_service", odometry_buffer_request);
  
  ros::spin();
  return 0;
}
