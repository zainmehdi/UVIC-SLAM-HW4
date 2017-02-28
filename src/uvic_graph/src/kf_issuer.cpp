#include <ros/ros.h>
#include <uvic_msgs/factor.h>
#include <uvic_msgs/keyframe.h>
#include <uvic_msgs/registration.h>
#include <uvic_msgs/odometry_buffer.h>
#include <geometry_msgs/PoseWithCovariance.h>

ros::Publisher kf_issuer_factor_pub;
ros::Publisher kf_issuer_keyframe_pub;
ros::ServiceClient odometry_buffer_client;

void scanner_callback(const uvic_msgs::registration& input) {
  if(input.KF_flag) {
    uvic_msgs::odometry_buffer odometry_buffer_query;
    odometry_buffer_query.request.t_start = input.TS;
    odometry_buffer_query.request.t_end = ros::Time::now();
  
    bool odometry_buffer_response = odometry_buffer_client.call(odometry_buffer_query);

    if(odometry_buffer_response) {
     geometry_msgs::PoseWithCovariance between = odometry_buffer_query.response.delta;
      geometry_msgs::PoseWithCovariance Delta = odometry_buffer_query.response.Delta;

      uvic_msgs::factor factor;
      factor.KF_ID_ref = input.KF_ID_ref;
      factor.KF_ID_new = input.KF_ID_new;
      factor.Delta = between;
      
      uvic_msgs::keyframe keyframe;
      keyframe.TS = input.TS;
      keyframe.ID = input.KF_ID_new;
      keyframe.scan = input.scan;
      keyframe.pose_odom = Delta;

      kf_issuer_factor_pub.publish(factor);
      kf_issuer_keyframe_pub.publish(keyframe);
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "kf_issuer");
  ros::NodeHandle n;

  odometry_buffer_client = n.serviceClient<uvic_msgs::odometry_buffer>("/odometry_buffer_service");
  ros::Subscriber scan_matching_sub = n.subscribe("/scan_matching/registration", 50, scanner_callback);
  kf_issuer_factor_pub = n.advertise<uvic_msgs::factor>("/kf_issuer/factor", 50);
  kf_issuer_keyframe_pub = n.advertise<uvic_msgs::keyframe>("/kf_issuer/keyframe", 50);

  while(ros::ok()) {
    
    ros::spin();
  }

  return 0;
}
