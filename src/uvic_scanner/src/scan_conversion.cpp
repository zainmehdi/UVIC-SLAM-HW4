#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <uvic_msgs/scan_to_pointcloud.h>
#include <laser_geometry/laser_geometry.h>

laser_geometry::LaserProjection projector;

bool conversion(uvic_msgs::scan_to_pointcloud::Request &req, uvic_msgs::scan_to_pointcloud::Response &res) {
  projector.projectLaser(req.scan, res.pointcloud);
  res.empty = ( res.pointcloud.data.size() == 0 ) ? true : false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_conversion");
  ros::NodeHandle n;
 
  ros::ServiceServer conversion_service = n.advertiseService("/scan_conversion/scan_to_pointcloud", conversion);
    
  ros::spin();
  return 0;
}
