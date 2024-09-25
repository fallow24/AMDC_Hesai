// Lidar Distortion Correction for spherical robots

#include "sr_lidar_distortion_correction.h"

using ::ros::NodeHandle;
using namespace std;


SRLidarDistortionCorrection::SRLidarDistortionCorrection()
        : node_initialized_(false),
          system_ready_(false),
          spin_rate_(1000.0)
{
}

// Spins the callback queue
void SRLidarDistortionCorrection::run()
{
    if (!node_initialized_)
    {
        ROS_FATAL("SRLidarDistortionCorrection is not initialized. Shutdown.");
        return;
    }
    ros::spin();
}

// Initialize node
bool SRLidarDistortionCorrection::init() {
  
  // Read from rosparam server
  if ( readConfig())
  {
    node_initialized_ = true;
  }
  // create publishers
  pcd_undistorted_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("points_out", 10);

  // create subscribers
  pcd_in_sub_ = node_handle_.subscribe("points_in", 10,
      &SRLidarDistortionCorrection::pcdCallback,
      this, ros::TransportHints().tcpNoDelay(true));
  imu_in_sub_ = node_handle_.subscribe("imu_in", 10,
      &SRLidarDistortionCorrection::vehicleTwistCallback,
      this, ros::TransportHints().tcpNoDelay(true));
  
  // Set spin rate (the old fashioned way)
  timer_ = node_handle_.createTimer(ros::Duration(1.0 / spin_rate_),
                                        &SRLidarDistortionCorrection::process, this);

  // Comparator function to sort points according to timestamp
  point_time_cmp = [](const PointType& p1, const PointType& p2)
      { return p1.timestamp < p2.timestamp; }; // which is ascending order

  // Get the imu -> lidar transformation
  static tf::TransformListener listener_tf;
  try {
    // FROM Imu TO Pandar frame: Premultiply imu2lidar_tf 
    listener_tf.waitForTransform("/pandar_frame", "/imu_frame", ros::Time(0), ros::Duration(10.0));
    listener_tf.lookupTransform("/pandar_frame", "/imu_frame", ros::Time(0), imu2lidar_tf);
  } catch (tf::TransformException &e) {
    ROS_ERROR("%s", e.what());
    tf::Transform no_tf(tf::Quaternion(0,0,0,1));
    imu2lidar_tf.setData(no_tf);
  }

  return node_initialized_;
}

bool SRLidarDistortionCorrection::readConfig()
{
    ros::NodeHandle priv_nh("~");

    // Config arguments
    priv_nh.param<int>("spin_rate", spin_rate_, 0);
    priv_nh.param<int>("radius_filter", radius_filter, -1);
    return true;
}


void SRLidarDistortionCorrection::process(const ros::TimerEvent &)
{
  ros::Time current_time = ros::Time::now();
}

// Function for returning the mean rotation speed during frame
tf::Vector3 SRLidarDistortionCorrection::return_twist()
{
  tf::Vector3 ret(imu_x / imu_count,
                  imu_y / imu_count,
                  imu_z / imu_count);
  ret = imu2lidar_tf * ret;
  // Reset the accumulated fields
  imu_count = 0;
  imu_x = 0;
  imu_y = 0;
  imu_z = 0;
  return ret;
}

// Callback function for ROS point cloud msgs. 
// Gets the mean vehicle twist during the LiDAR measurement and applies angular motion distortion correction.  
void SRLidarDistortionCorrection::pcdCallback(const sensor_msgs::PointCloud2ConstPtr& pc)
{
  // Convert ROS to PCL (use custom PointType to access timestamp fields of Hesai Lidar)
  pcl::PointCloud<PointType>::Ptr pcd_in (boost::make_shared<pcl::PointCloud<PointType>>());
  pcl::fromROSMsg(*pc, *pcd_in);
  // Point towards private class attribute
  pcd_in_ = pcd_in;

  //overall rotation speed during frame: averaged -> this is before frame!!
  tf::Vector3 rot = return_twist();
  double theta_x = rot.x();
  double theta_y = rot.y();
  double theta_z = rot.z();

  pcl::PointCloud<pcl::PointXYZI> pcl_out;
  double scan_x, scan_y, scan_z;
  for (unsigned int i = 0; i < pcd_in_->size(); ++i) {
    
    // offset time from start of the measurement
    auto dt = pcd_in_->points[i].timestamp - pc->header.stamp.toSec();
    
    //read point coordinates
    scan_x = pcd_in_->points[i].x;
    scan_y = pcd_in_->points[i].y;
    scan_z = pcd_in_->points[i].z;

    // check for validity
    if(std::isnan(scan_x) || std::isnan(scan_y) || std::isnan(scan_z) ){continue;} 
    // optional distance filter
    if (radius_filter != -1 && (scan_x*scan_x+scan_y*scan_y+scan_z*scan_z < 0.0001*radius_filter*radius_filter)){continue;}

    //perform rotation --> add rottion * dt
    // Hesai style: Timestamp corresponds to the start of measurement 
    // -> Forward projection of points in time
    double x_rot = dt * theta_x;
    double y_rot = dt * theta_y;
    double z_rot = dt * theta_z;

    pcl::PointXYZI pt;
    double cx = cos(x_rot), cy = cos(y_rot), cz = cos(z_rot);
    double sx = sin(x_rot), sy = sin(y_rot), sz = sin(z_rot);
    //write corrected values in pc2. Use rotation rpy-rotation matrix
    pt.x =  scan_x*cy*cz +
            scan_y*(sx*sy*cz-cx*sz) +
            scan_z*(cx*sy*cz+sx*sz);
    pt.y =  scan_x*cy*sz +
            scan_y*(sx*sy*sz+cx*cz) +
            scan_z*(cx*sy*sz-sx*cz);
    pt.z = -scan_x*sy +
            scan_y*sx*cy +
            scan_z*cx*cy;

    // check for validity
    if(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z) ){continue;} 
    pt.intensity = pcd_in_->points[i].intensity;
    pcl_out.push_back(pt);
  }

  // Construct new ROS msg
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_out));
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*scan_ptr, msg);
  msg.header = pc->header;
  
  // publish
  pcd_undistorted_pub_.publish(msg);
}

void SRLidarDistortionCorrection::vehicleTwistCallback(const sensor_msgs::Imu &imu)
{
  // whenever we read the twist restart process of averaging
  // needs to be alligned to lidar!! 
  // HERE specific for our spherical robot -> axes of LiDAR and IMU need to be alligned
  imu_x += imu.angular_velocity.x;
  imu_y += imu.angular_velocity.y;
  imu_z += imu.angular_velocity.z;
  imu_count += 1;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "SR_lidar_distortion_correction_node");

  SRLidarDistortionCorrection sr_lidar_distortion_correction_node;
  if (sr_lidar_distortion_correction_node.init()) {
    sr_lidar_distortion_correction_node.run(); 
  }
  else {
    ROS_FATAL_STREAM("SR_lidar_distortion_correction_node initialization failed. Shutdown.");
  }
  return 0;
}
