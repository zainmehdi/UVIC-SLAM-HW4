#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

ros::Publisher global_odom_pub;

void odometry_callback(const geometry_msgs::PoseWithCovariance& input) {
  geometry_msgs::TransformStamped odom_trans;
  tf::TransformBroadcaster odom_broadcaster;

  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = input.pose.position.x;
  odom_trans.transform.translation.y = input.pose.position.y;
  odom_trans.transform.translation.z = input.pose.position.z;
  odom_trans.transform.rotation = input.pose.orientation;
  odom_broadcaster.sendTransform(odom_trans);

  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = input.pose.position.x;
  odom.pose.pose.position.y = input.pose.position.y;
  odom.pose.pose.position.z = input.pose.position.z;
  odom.pose.pose.orientation = input.pose.orientation;
  odom.child_frame_id = "base_link";
  global_odom_pub.publish(odom);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_tf");
  ros::NodeHandle n;

  ros::Subscriber global_pose_sub = n.subscribe("/odometry_integrator/odometry", 50, odometry_callback);
  global_odom_pub = n.advertise<nav_msgs::Odometry>("tf/global_pose", 50);

  ros::spin();

  return 0;
}
