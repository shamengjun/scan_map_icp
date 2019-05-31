//-----------------------------------------------------------------------------
// Copyright(c) 2019-2024 roboticplus
// All rights reserved.
//
// File: scan_map_icp.h
// Version: 1.0.00
//
// Create:  Apr.22 2019
// Author:  zhaoyz
// Summary: scan and laser icp
//-----------------------------------------------------------------------------

#ifndef BASE_PERCEPTION_SCAN_MAP_ICP_H_
#define BASE_PERCEPTION_SCAN_MAP_ICP_H_

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/thread/mutex.hpp>

namespace scan_map_icp
{
typedef struct
{
  double inlier_percent;
  double dist;
  double angle_dist;
}IcpResultData;

class ScanMapIcp
{
public:
  ScanMapIcp();
  ~ScanMapIcp();
private:
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > map_cloud_xyz_;
  boost::shared_ptr<sensor_msgs::PointCloud2> map_cloud_;
  boost::shared_ptr<sensor_msgs::PointCloud2> scan_cloud_;
  boost::mutex scan_callback_mutex_;
  pcl::KdTree<pcl::PointXYZ>::Ptr map_tree_;
  void updateParamsLoop();

private:
  ros::NodeHandle private_nh_;
  ros::NodeHandle global_nh_;
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber base_motion_type_sub_;
  ros::Publisher initial_pose_pub_;
  ros::Publisher map_to_cloud_point_pub_;
  ros::Publisher scan_to_cloud_point_pub_;
  ros::Publisher scan_after_transform_pub_;
  ros::Publisher icp_run_info_pub_;
  std::string base_laser_frame_;
  std::string odom_frame_;
  laser_geometry::LaserProjection *projector_;
  tf::TransformListener *listener_;
  sensor_msgs::PointCloud2 cloud2_;
  sensor_msgs::PointCloud2 cloud2transformed_;
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
  ros::Time params_were_updated_;
  ros::Time last_proccessed_scan_;
  ros::Time scan_in_time_;
  sensor_msgs::PointCloud cloud_in_map_;
  sensor_msgs::PointCloud cloud_in_scan_;
  tf::StampedTransform old_pose_;
  tf::StampedTransform base_at_laser_;
  // bool status_loop_thread_;
  bool has_map_;
  bool has_scan_;
  bool has_scan_transform_;
  bool use_sim_time_;
  int last_scan_;
  int act_scan_;
  int last_time_sent_;
  bool got_transform_;
  bool publish_initial_pose_;
  //底盘当前是走直线还是转圈,"straight" or "turn" or "arc"
  std::string base_motion_type_;
  IcpResultData icp_result_;
  ros::Duration scan_age_;
  double fitness_score_;
  double icp_converge_;
  tf::Vector3 rot_axis_;
  tf::Transform t_;
private:
  // transform convert matrix form
  inline void matrixAsTransform(const Eigen::Matrix4f &out_mat, tf::Transform &bt)
  {
    double mv[12];
    mv[0] = out_mat(0, 0);
    mv[4] = out_mat(0, 1);
    mv[8] = out_mat(0, 2);
    mv[1] = out_mat(1, 0);
    mv[5] = out_mat(1, 1);
    mv[9] = out_mat(1, 2);
    mv[2] = out_mat(2, 0);
    mv[6] = out_mat(2, 1);
    mv[10] = out_mat(2, 2);
    tf::Matrix3x3 basis;
    basis.setFromOpenGLSubMatrix(mv);
    tf::Vector3 origin(out_mat(0, 3),out_mat(1, 3),out_mat(2, 3));
    ROS_DEBUG("origin %f %f %f", origin.x(), origin.y(), origin.z());
    bt = tf::Transform(basis,origin);
  }
  // create one kdtree
  pcl::KdTree<pcl::PointXYZ>::Ptr getTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  // laser scan subcallback function
  void laserScanSubCallback(const sensor_msgs::LaserScanConstPtr &msg);
  // map subcallback function
  void mapSubCallback(const nav_msgs::OccupancyGridConstPtr &data);
  // get tf from two frames
  bool getTransform(tf::StampedTransform &trans, const std::string parent_frame,\
                    const std::string child_frame, const ros::Time stamp);
  // initial icp params
  void initialParams();
  // update icp params
  void updateParams();
  // run icp
  void runIcp();
  void projectLaser();
  void computeInlierPecent(PointCloudT &pc);
  void publishIcpInfo();
 // void publishRelocationPose();
  void transformScanToPointcloud();
  // 当前是走直线还是转弯
  void baseMotionTypeSubCallback(const std_msgs::String::ConstPtr &data);
private:
  /**
   * @brief icp_fitness_threshold_ ,defines how good the match has to be to create
   *        a candidate for publishing a pose
   */
  double icp_fitness_threshold_;
  /**
   * @brief dist_threshold_,minimum position change that will trigger a
   *        re-initialisation of amcl
   */
  double dist_threshold_;
  /**
   * @brief angle_threshold_,minimum angle change that will trigger a
   *        re-initialisation of amcl
   */
  double angle_threshold_;
  /**
   * @brief angle_upper_threshold_,maximum icp transform angle that will
   *         still be considered a sane match published as an initial pose
   */
  double angle_upper_threshold_;
  /**
   * @brief age_threshold_,accept only scans one second old or younger, describes how old
   *        laser scans can be before they get sorted out by icp
   */
  double age_threshold_;
  /**
   * @brief update_age_threshold_, how long before another new initial pose
   *        will be sent after one was sent
   */
  double update_age_threshold_;
  /**
   * @brief icp_inlier_threshold_,percentage of scan points that must match withing icp_inlier_dist
   *        so that the match is considered good and a initial pose can be sent
   */
  double icp_inlier_threshold_;
  /**
   * @brief icp_inlier_dist_,distance a point can have to its nearest neighbor
   *        in map to be still considered as inlier
   */
  double icp_inlier_dist_;
  /**
   * @brief pose_covariance_trans_,translational pose covariance the initial pose gets sent with
   */
  double pose_covariance_trans_;
  /**
   * @brief icp_num_iter_,number of iterations in ICP
   */
  int icp_num_iter_;
  /**
   * @brief scan_rate_,over 1/scan_rate_ ,we consider the scan is too old
   */
  int scan_rate_;
};
}
#endif
