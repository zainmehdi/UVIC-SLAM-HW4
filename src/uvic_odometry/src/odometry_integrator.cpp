#include <math.h>
#include <sstream>
#include <string.h>
#include <unistd.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <uvic_msgs/odometry.h>
#include <geometry_msgs/Twist.h>
#include <uvic_msgs/covariance.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovariance.h>

double x, y, th;
double vx, vy, vth;
double Delta_x, Delta_y, Delta_th;
Eigen::Matrix3d C_D;

uvic_msgs::covariance eigen_matrix_to_covariance(Eigen::Matrix3d input) {
  uvic_msgs::covariance output;
  
  for(int i = 0; i < input.rows(); i++) {
    for(int j = 0; j < input.cols(); j++) {
      output.covariance[( i * input.rows() ) + j] = input(i, j);
    }
  }

  return output;
}

geometry_msgs::Pose create_pose_msg(double x, double y, double th) {
  geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(th);
  
  geometry_msgs::Pose pose;
  pose.position.x  = x;
  pose.position.y  = y;
  pose.orientation = pose_quat;

  return pose;
}

geometry_msgs::PoseWithCovariance create_pose_covar_msg(double x, double y, double th, Eigen::Matrix3d m) {
  geometry_msgs::Pose pose = create_pose_msg(x, y, th);

  geometry_msgs::PoseWithCovariance pose_covar;
  pose_covar.pose = pose;
  pose_covar.covariance = eigen_matrix_to_covariance(m).covariance;

  return pose_covar;
}

uvic_msgs::odometry create_buffer_msg(ros::Time TS,
				      geometry_msgs::PoseWithCovariance delta,
				      geometry_msgs::PoseWithCovariance Delta,
				      Eigen::Matrix3d Jac_Pose_delta,
				      Eigen::Matrix3d Jac_Pose_Delta) {
  uvic_msgs::odometry output;
  output.TS = TS;
  delta.covariance = eigen_matrix_to_covariance(Jac_Pose_delta).covariance;
  Delta.covariance = eigen_matrix_to_covariance(Jac_Pose_Delta).covariance;
  output.delta = delta;
  output.Delta = Delta;

  return output;
}

void velocity_callback(const geometry_msgs::Twist::ConstPtr& input) {
  vx  = input->linear.x;
  vy  = input->linear.y;
  vth = input->angular.z;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_integrator");
  ros::NodeHandle n;
  
  ros::Publisher delta_pub = n.advertise<geometry_msgs::Pose>("/odometry_integrator/delta_pose", 50);
  ros::Publisher global_pub = n.advertise<geometry_msgs::Pose>("/odometry_integrator/global_pose", 50);
  ros::Publisher Delta_pub = n.advertise<geometry_msgs::PoseWithCovariance>("/odometry_integrator/Delta_pose", 50);
  ros::Publisher odom_buff_pub = n.advertise<uvic_msgs::odometry>("/odometry_integrator/odometry", 50);
  
  ros::Subscriber vel_sub = n.subscribe("/cmd_vel", 50, velocity_callback);
  
  ros::Time current_time = ros::Time::now();
  ros::Time last_time = current_time;
  ros::Rate loop_rate(100); // 100Hz

  //### Initialize motion data ###
  vx = 0.0;
  vy = 0.0;
  vth = 0.0;
  //### Initial reset: pose ###
  x = 0.0;
  y = 0.0;
  th = 0.0;
  //### Initial reset: factor pose increment ###
  Delta_x = 0.0;
  Delta_y = 0.0;
  Delta_th = 0.0;

  //######################################
  //### SET THESE USER-DEFINED VALUES ####
  double sigma_vx = 0.1; // [m/s/sqrt(s)]
  double sigma_vy = 0.1; // [m/s/sqrt(s)]
  double sigma_vth = 0.1; // [m/s/sqrt(s)]
  //### SET THESE USER-DEFINED VALUES ####
  //######################################
  
  //### Set constant twist covariance ###
  Eigen::Matrix3d C_v = Eigen::Matrix3d::Zero(3, 3);
  C_v(0,0) = pow(sigma_vx, 2);
  C_v(1,1) = pow(sigma_vy, 2);
  C_v(2,2) = pow(sigma_vth, 2);

  //### Initial reset: factor pose increment's covariance ###
  C_D = Eigen::Matrix3d::Zero(3, 3);

  while(ros::ok()) {
    current_time  = ros::Time::now();
    double delta_t = (current_time - last_time).toSec();

    //### Time Integration of Velocity Data ###
    double delta_x = vx  * delta_t;
    double delta_y = vy  * delta_t;
    double delta_th = vth * delta_t;
     
    //### Jacobian stage ###
    Eigen::Matrix3d J_d_v_dt = Eigen::Matrix3d::Zero(3, 3);
    J_d_v_dt(0, 0) = 1;
    J_d_v_dt(1, 1) = 1;
    J_d_v_dt(2, 2) = 1;
            
    Eigen::Matrix3d C_d = J_d_v_dt * C_v * J_d_v_dt.transpose() * delta_t;
      
    //### Integrate Factor ###
    double Delta_th_cos = cos ( Delta_th );
    double Delta_th_sin = sin ( Delta_th );

    Delta_x += ( delta_x * Delta_th_cos ) - ( delta_y * Delta_th_sin );
    Delta_y += ( delta_x * Delta_th_sin ) + ( delta_y * Delta_th_cos );
    Delta_th += delta_th;
    Delta_th = std::fmod( Delta_th + M_PI, 2 * M_PI) - M_PI;
    
    //### Jacobian Stage ###
    Eigen::Matrix3d J_D_D = Eigen::Matrix3d::Zero(3, 3);
    J_D_D(0, 0) = 1;
    J_D_D(0, 2) = ( -1 * delta_x * Delta_th_sin ) - ( delta_y * Delta_th_cos );
    J_D_D(1, 1) = 1;
    J_D_D(1, 2) = ( delta_x * Delta_th_cos ) - ( delta_y * Delta_th_sin );
    J_D_D(2, 2) = 1;

    Eigen::Matrix3d J_D_d = Eigen::Matrix3d::Zero(3, 3);
    J_D_d(0, 0) = Delta_th_cos;
    J_D_d(0, 1) = -1 * Delta_th_sin;
    J_D_d(1, 0) = Delta_th_sin;
    J_D_d(1, 1) = Delta_th_cos;
    J_D_d(2, 2) = 1;
      
    //### Covariance stage ###
    C_D = J_D_D * C_D * J_D_D.transpose() + J_D_d * C_d * J_D_d.transpose();

    //### Integrate Global Pose ###
    x += delta_x * cos ( th ) - delta_y * sin ( th );
    y += delta_x * sin ( th ) + delta_y * cos ( th );
    th += delta_th;
    th = std::fmod( th + M_PI, 2 * M_PI) - M_PI;

    //### Publish d, D and global poses, and covariance C_D ###
    Delta_pub.publish(create_pose_covar_msg(Delta_x, Delta_y, Delta_th, C_D));
    delta_pub.publish(create_pose_msg(delta_x, delta_y, delta_th));
    global_pub.publish(create_pose_msg(x, y, th));

    //### Publish Delta and delta poses to odometry_buffer ###
    geometry_msgs::PoseWithCovariance delta = create_pose_covar_msg(delta_x, delta_y, delta_th, C_d);
    geometry_msgs::PoseWithCovariance Delta = create_pose_covar_msg(Delta_x, Delta_y, Delta_th, C_D);
    uvic_msgs::odometry odometry_buffer_msg = create_buffer_msg(current_time, delta, Delta, J_D_d, J_D_D);
    odom_buff_pub.publish(odometry_buffer_msg);

    last_time = current_time;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
