#include <ros/ros.h>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <uvic_msgs/kf_select.h>
#include <uvic_msgs/covariance.h>
#include <pcl/registration/gicp.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <uvic_msgs/registration.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <uvic_msgs/scan_to_pointcloud.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseWithCovariance.h>

ros::Publisher scan_matching_pub;
ros::ServiceClient scan_converter_client;
ros::ServiceClient kf_select_client;
int new_keyframe_ID;

uvic_msgs::covariance eigen_matrix_to_covariance(Eigen::MatrixXd input) {
  uvic_msgs::covariance output;

  for(int i = 0; i < input.rows(); i++) {
    for(int j = 0; j < input.cols(); j++) {
      output.covariance[( i * input.rows() ) + j] = input(i, j);
    }
  }

  return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_format(sensor_msgs::PointCloud2 input) {
  pcl::PCLPointCloud2 pcl2_pointcloud;
  pcl_conversions::toPCL(input, pcl2_pointcloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl2_pointcloud, *pcl_pointcloud);

  return pcl_pointcloud;
}

geometry_msgs::PoseWithCovariance create_pose_covar_msg(double x, double y, double th, Eigen::MatrixXd m) {
  geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(th);
  geometry_msgs::Pose pose;
  pose.position.x  = x;
  pose.position.y  = y;
  pose.orientation = pose_quat;

  geometry_msgs::PoseWithCovariance pose_covar;
  pose_covar.pose = pose;
  pose_covar.covariance = eigen_matrix_to_covariance(m).covariance;

  return pose_covar;
}

uvic_msgs::registration create_registration_msg(ros::Time TS,
						bool KF_flag,
						bool LC_flag,
						int KF_ID_ref,
						int KF_ID_new,
						sensor_msgs::LaserScan scan,
						geometry_msgs::PoseWithCovariance Delta) {
  uvic_msgs::registration output;
  output.TS = TS;
  output.KF_flag = KF_flag;
  output.LC_flag = LC_flag;
  output.KF_ID_ref = KF_ID_ref;
  output.KF_ID_new = KF_ID_new;
  output.scan = scan;
  output.Delta = Delta;

  return output;
}

void laser_callback(const sensor_msgs::LaserScan& input) {
  uvic_msgs::scan_to_pointcloud new_scan_srv;
  new_scan_srv.request.scan = input;
  bool new_scan_returned = scan_converter_client.call(new_scan_srv);

  if(new_scan_returned & !new_scan_srv.response.empty) {
    uvic_msgs::kf_select kf_trigger_srv;
    bool kf_select_returned = kf_select_client.call(kf_trigger_srv);

    if(kf_select_returned) {
      uvic_msgs::scan_to_pointcloud kf_scan_srv;
      kf_scan_srv.request.scan = kf_trigger_srv.response.KF_ref.scan;
      bool kf__scan_returned = scan_converter_client.call(kf_scan_srv);

      if(kf__scan_returned & !kf_scan_srv.response.empty) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_pointcloud = pointcloud_format(new_scan_srv.response.pointcloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr kf_pointcloud = pointcloud_format(kf_scan_srv.response.pointcloud);

        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

        gicp.setInputSource(new_pointcloud);
        gicp.setInputTarget(kf_pointcloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_transformed_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
        gicp.align(*pcl_transformed_pointcloud);

        double converged_fitness = gicp.getFitnessScore();
        bool converged = gicp.hasConverged();
	Eigen::Matrix4f transform = gicp.getFinalTransformation();
 
        if(converged) {
          double Dx = transform(0 ,3);
          double Dy = transform(1, 3);
          double Dth = atan2( transform(1, 0), transform(0, 0) );

          double k_disp_disp = 0.1;
          double k_rot_disp = 0.1;
          double k_rot_rot = 0.1;

          double Dl = sqrt( pow(Dx, 2) + pow(Dy, 2) );
          double sigma_x_squared = k_disp_disp * Dl;
          double sigma_y_squared = k_disp_disp * Dl;
          double sigma_th_squared = ( k_rot_disp * Dl ) + ( k_rot_rot * Dth );

          Eigen::MatrixXd C_l(6, 6);
          C_l(0, 0) = sigma_x_squared;
          C_l(1, 1) = sigma_y_squared;
          C_l(5, 5) = sigma_th_squared;

          double converged_fitness_threshold = 0.15;
          if(converged_fitness > converged_fitness_threshold) {
	    uvic_msgs::registration output = create_registration_msg( input.header.stamp,
								   true,
								   false,
								   kf_trigger_srv.response.KF_ref.ID,
								   new_keyframe_ID++,
								   input,
								   create_pose_covar_msg(Dx, Dy, Dth, C_l));
            scan_matching_pub.publish(output);
          } else {
	    uvic_msgs::registration output = create_registration_msg( input.header.stamp,
								   false,
								   false,
								   kf_trigger_srv.response.KF_ref.ID,
								   new_keyframe_ID++,
								   input,
								   create_pose_covar_msg(Dx, Dy, Dth, C_l));
            scan_matching_pub.publish(output);
          }
        } 
      }
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_matching");
  ros::NodeHandle n;

  new_keyframe_ID = 0;

  ros::Subscriber laser_sub = n.subscribe("/base_scan", 50, laser_callback);

  kf_select_client = n.serviceClient<uvic_msgs::kf_select>("/graph/kf_select");
  scan_matching_pub = n.advertise<uvic_msgs::registration>("/scan_matching/registration", 50);
  scan_converter_client = n.serviceClient<uvic_msgs::scan_to_pointcloud>("/scan_matching/scan_to_pointcloud");
  
  ros::spin();

  return 0;
}
