#ifndef _SR_LIDAR_DISTORTION_CORRECTION_H_
#define _SR_LIDAR_DISTORTION_CORRECTION_H_

// Systen libs
#include <cmath>
#include <vector>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <string>

// ROS 
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// PCL 
#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// Define a custom point struct for the HESAI to sort points according to timestamps
namespace SRAMDC {
  struct Point {
    Point(): data{0.f, 0.f, 0.f, 1.f} {}

    PCL_ADD_POINT4D;
    float intensity; // intensity
    double timestamp; // absolute timestamp in seconds
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16; // Memory alignemnt
}
// Register the custom point struct to PCL lib
POINT_CLOUD_REGISTER_POINT_STRUCT(SRAMDC::Point,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (double, timestamp, timestamp))
typedef SRAMDC::Point PointType;

class SRLidarDistortionCorrection {
  
  // Attributes
  bool system_ready_;
  bool node_initialized_;
  double rate_;

  int radius_filter = -1; // cm
  int spin_rate_ = 1000;

  // for imu
  int imu_count = 0;
  double imu_x = 0.0;
  double imu_y = 0.0;
  double imu_z = 0.0;

  pcl::PointCloud<PointType>::Ptr pcd_in_;

  // IMU -> LiDAR static 6DoF transformation
  tf::StampedTransform imu2lidar_tf;

  // Node handle
  ros::NodeHandle node_handle_;
  ros::Timer timer_;

  // Published topics
  ros::Publisher pcd_undistorted_pub_;

  // Subsribed topics
  ros::Subscriber pcd_in_sub_;
  ros::Subscriber imu_in_sub_;

  // Comparator function for points to sort them according to their timestamp
  std::function<bool(const PointType&, const PointType&)> point_time_cmp;

  // Methods
  bool readConfig();

  void process(const ros::TimerEvent &);

  tf::Vector3 return_twist();

  // Callbacks
  void pcdCallback(const sensor_msgs::PointCloud2ConstPtr& pc);

  void vehicleTwistCallback(const sensor_msgs::Imu &imu);

  
public:
    SRLidarDistortionCorrection();

    bool init();

    void run();

};

#endif // _SR_LIDAR_DISTORTION_CORRECTION_H_
