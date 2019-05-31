//-----------------------------------------------------------------------------
// Copyright(c) 2019-2024 roboticplus
// All rights reserved.
//
// File: scan_map_icp.cpp
// Version: 1.0.00
//
// Create:  Apr.22 2019
// Author:  zhaoyz
// Summary: scan and laser icp
//-----------------------------------------------------------------------------

#include "scan_map_icp/scan_map_icp.h"

namespace scan_map_icp
{
ScanMapIcp::ScanMapIcp():
  private_nh_("~")
{
  initialParams();
  global_nh_.param("base_laser_frame", base_laser_frame_, std::string("base_laser_link"));
  global_nh_.param("odom_frame", odom_frame_, std::string("odom_combined"));
  private_nh_.param("use_sim_time", use_sim_time_, true);
  if (use_sim_time_)
  {
    ROS_INFO("use_sim_time true!");
  }
  else
  {
     ROS_INFO("use_sim_time false!");
  }
  last_proccessed_scan_ = ros::Time::now();
  projector_ = new laser_geometry::LaserProjection();
  tf::TransformListener listener;
  listener_ = &listener;
  laser_scan_sub_ = private_nh_.subscribe<sensor_msgs::LaserScan>("scan", 1, &ScanMapIcp::laserScanSubCallback, this);
  map_sub_ = private_nh_.subscribe<nav_msgs::OccupancyGrid>("map", 1, &ScanMapIcp::mapSubCallback, this);
  //base_motion_type_sub_ = global_nh_.subscribe<std_msgs::String>("base_motion_type", 1, &ScanMapIcp::baseMotionTypeSubCallback, this);
  call_icp_sub_ = global_nh_.subscribe<std_msgs::String>("call_icp", 1, &ScanMapIcp::callIcpSubCallback, this);
  //base_motion_type_ = std::string("static");
  call_icp_msg_ = std::string("finish");
  initial_pose_pub_ = private_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initial_pose", 1);
  icp_run_info_pub_ = private_nh_.advertise<std_msgs::String>("icp_run_info", 1);
  map_to_cloud_point_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("map_point", 1);
  scan_to_cloud_point_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("scan_point", 1);
  scan_after_transform_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("scan_point_transformed", 1);
  map_cloud_xyz_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>());
  map_cloud_ = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2());
  scan_cloud_ = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2());
  listener_->waitForTransform("/base_link", "map", ros::Time(0), ros::Duration(30.0));
  listener_->waitForTransform(base_laser_frame_, "/map", ros::Time(0), ros::Duration(30.0));
  // ros async thread spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();
  updateParams();
  ROS_INFO("ScanMapIcp running");
  
 //调试时开辟一个线程更新参数
 //updateParamsThread_ = new boost::thread(boost::bind(&ScanMapIcp::updateParamsThreadFun,this));
  updateParamsLoop();
}

ScanMapIcp::~ScanMapIcp()
{
// 调试用的时候，析构函数删除线程函数
/*  if(statusLoopThread_)
//  {
//    updateParamsThread_->join();
//    delete updateParamsThread_;
//    statusLoopThread_ = false;
 }*/
}

void ScanMapIcp::initialParams()
{
  icp_fitness_threshold_ = 100.1;
  dist_threshold_ = 0.05;
  angle_threshold_ = 0.01;
  angle_upper_threshold_ = M_PI / 6;
  age_threshold_ = 1;
  update_age_threshold_ = 1;
  icp_inlier_dist_ = 0.1;
  icp_inlier_threshold_ = 0.9;
  pose_covariance_trans_ = 1.5;
  icp_num_iter_ = 250;
  scan_rate_ = 2;
 // status_loop_thread_ = true;
  last_time_sent_ = -1000;
  act_scan_ = 0;
  last_scan_ = 0;
}
void ScanMapIcp::updateParamsLoop()
{
  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    if (act_scan_ > last_scan_)
    {
      last_scan_ = act_scan_;
//      if (has_map_)
//      {
//        map_to_cloud_point_pub_.publish(map_cloud_);
//       // has_map_ = false;
//      }
//      if (has_scan_)
//      {
//        scan_to_cloud_point_pub_.publish(scan_cloud_);
//        has_scan_ = false;
//      }
      if (has_scan_transform_)
      {
        scan_after_transform_pub_.publish(cloud2transformed_);
 //      has_scan_transform_ = false;
      }
    }
    loop_rate.sleep();
    ros::spinOnce();
    if(ros::Time::now() - params_were_updated_ > ros::Duration(1.0))
    {
      updateParams();
    }
  }
}
/**
 * @brief ScanMapIcp::getTree , get cloud search tree
 * @param cloud ,input cloud
 * @return result tree
 */
pcl::KdTree<pcl::PointXYZ>::Ptr ScanMapIcp::getTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::KdTree<pcl::PointXYZ>::Ptr tree;
  tree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  tree->setInputCloud(cloud);
  return tree;
}
/**
 * @brief ScanMapIcp::mapSubCallback, map subscribe callback function, convert map to pointcloud
 * @param data
 */
void ScanMapIcp::mapSubCallback(const nav_msgs::OccupancyGridConstPtr &data)
{
  ROS_INFO("I hear frame_id:[%s]", data->header.frame_id.c_str());
  float resolution = data->info.resolution;
  float width = data->info.width;
  float height = data->info.height;
  float origin_pose_x = data->info.origin.position.x;
  float origin_pose_y = data->info.origin.position.y;

  //map_cloud_xyz_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>());
  map_cloud_xyz_->height = 1; // pointcloud is unordered
  map_cloud_xyz_->is_dense = false; // if has invalid data(NAN or inf)
  std_msgs::Header header;
  header.stamp = ros::Time(0);
  header.frame_id = "/map";
  map_cloud_xyz_->header = pcl_conversions::toPCL(header);

  pcl::PointXYZ point_xyz;
  // fill with pointcloud
  ROS_INFO("111111");
  for(int y = 0; y < height; ++y)
  {
    for(int x = 0; x < width; ++x)
    {
      if(data->data[x + y*width] == 100)
      {
        point_xyz.x = (.5f + x) * resolution + origin_pose_x;
        point_xyz.y = (.5f + y) * resolution + origin_pose_y;
        point_xyz.z = 0;
        map_cloud_xyz_->points.push_back(point_xyz);
      }
    }
  }
  map_cloud_xyz_->width = map_cloud_xyz_->points.size(); // pointcloud's number
  ROS_INFO("222222");
  map_tree_ = getTree(map_cloud_xyz_);
  pcl::toROSMsg(*map_cloud_xyz_,*map_cloud_);
  
  ROS_INFO("Publishing PointXYZ cloud with %ld points in frame %s", map_cloud_xyz_->points.size(), map_cloud_->header.frame_id.c_str());
  has_map_ = true;
}
/**
 * @brief ScanMapIcp::getTransform, get two frames' translation
 * @param trans, save translation variable
 * @param parent_frame, parent frame
 * @param child_frame, child frame
 * @param stamp, time stamp
 * @return transform state
 */
bool ScanMapIcp::getTransform(tf::StampedTransform &trans, const std::string parent_frame,\
                              const std::string child_frame, const ros::Time stamp)
{
  bool gotTransform = false;
  ros::Time before = ros::Time::now();
  if (!listener_->waitForTransform(parent_frame, child_frame, stamp, ros::Duration(0.5)))
  {
    ROS_ERROR("DIDNOT GET TRANSFORM %s %s at %f", parent_frame.c_str(), child_frame.c_str(), stamp.toSec());
    return false;
  }
  try
  {
    gotTransform = true;
    listener_->lookupTransform(parent_frame, child_frame, stamp, trans);
  }
  catch (tf::TransformException &ex)
  {
    gotTransform = false;
    ROS_ERROR("DIDNOT GET TRANSFORM %s %s,reason:%s", parent_frame.c_str(), child_frame.c_str(), ex.what()); // exception inherite from runtime_error,const char *
  }
  return gotTransform;
}
/**
 * @brief ScanMapIcp::laserScanSubCallback, laser callback function
 * @param msg
 */
void ScanMapIcp::laserScanSubCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
  if (!has_map_)
  {
    ROS_WARN("waiting for map to be published......");
    return;
  }
  scan_in_time_ = msg->header.stamp;
  ros::Time time_received = ros::Time::now();
  if (scan_in_time_ - last_proccessed_scan_ < ros::Duration(1 / scan_rate_))
  {
    ROS_WARN("rejected scan,last %f,this %f", last_proccessed_scan_.toSec(), scan_in_time_.toSec());
    return;
  }
  if (!scan_callback_mutex_.try_lock())
    return;
  scan_age_ = ros::Time::now() - scan_in_time_;
  ROS_DEBUG("judge whether scan to old!");
  if (!use_sim_time_)
  {
    if (scan_age_.toSec() > age_threshold_)
    {
      ROS_WARN("SCAN SEEMS TO OLD (%f seconds %f threshold) scan time:%f now:%f", scan_age_.toSec(), age_threshold_,scan_in_time_.toSec(), ros::Time::now().toSec());
      scan_callback_mutex_.unlock();
      return;
    }
  }
  ROS_DEBUG("judge whether get base pose at laser scan time!");
  //tf::StampedTransform base_at_laser;
  if (!getTransform(base_at_laser_, odom_frame_, "base_link", scan_in_time_))
  {
    ROS_WARN("Did not get base pose at laser scan time");
    scan_callback_mutex_.unlock();
    return;
  }
  // if it satisfies below conditions , start to convert scan data to pointcloud
  ROS_DEBUG("project laser!");
  //sensor_msgs::PointCloud cloud;
  //sensor_msgs::PointCloud cloudInMap;
  //Project a sensor_msgs::LaserScan into a sensor_msgs::PointCloud
  projector_->projectLaser(*msg, cloud_in_scan_);
  has_scan_ = false;
  //bool gotTransform = false;
  got_transform_ = false;
  ROS_DEBUG("judge whether map to cloud transform found!");
  if (!listener_->waitForTransform("/map", cloud_in_scan_.header.frame_id,\
                                   cloud_in_scan_.header.stamp, ros::Duration(0.05)))
  {
    ROS_WARN("no map to cloud transform found!");
    scan_callback_mutex_.unlock();
    return;
  }
  ROS_DEBUG("judge whether map to base transform found!");
  if (!listener_->waitForTransform("/map", "/base_link", cloud_in_scan_.header.stamp, ros::Duration(0.05)))
  {
    ROS_WARN("no map to base transform found!");
    scan_callback_mutex_.unlock();
    return;
  }
  // transform scan pointcloud
  transformScanToPointcloud();
  // when got base and map transform , and convert map and scan pointcloud ,we start to run icp
  runIcp();
  scan_callback_mutex_.unlock();
}

/**
 * @brief ScanMapIcp::transformScanToPointcloud,把激光数据转换为点云
 */
void ScanMapIcp::transformScanToPointcloud()
{
  while (!got_transform_ && (ros::ok()))
  {
    try
    {
      got_transform_ = true;
      listener_->transformPointCloud("/map", cloud_in_scan_, cloud_in_map_);
      ROS_DEBUG("transform cloud to map!");
    }
    catch(...)
    {
      got_transform_ = false;
      ROS_WARN("DIDNT GET TRANSFORM");
    }
  }

  for(size_t i = 0; i < cloud_in_map_.points.size(); ++i)
  {
    cloud_in_map_.points[i].z = 0;
  }
  got_transform_ = false;
 // tf::StampedTransform oldPose;
 // get base and map transform
  while (!got_transform_ && ros::ok())
  {
    try
    {
      got_transform_ = true;
      listener_->lookupTransform("/map", "/base_link", cloud_in_scan_.header.stamp, old_pose_);
      ROS_DEBUG("map->base_link's transform!");
    }
    catch(tf::TransformException &ex)
    {
      ROS_WARN("DIDNT GET TRANSFORM,%s", ex.what());
      got_transform_ = false;
    }
  }
}
/**
 * @brief ScanMapIcp::runIcp，运行pcl中的icp
 */
void ScanMapIcp::runIcp()
{
  if (has_map_ && got_transform_)
  {
    sensor_msgs::convertPointCloudToPointCloud2(cloud_in_map_, cloud2_);
    has_scan_ = true;
    act_scan_ ++;
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> reg;
    /*Set the transformation epsilon (maximum allowable difference between two consecutive
     * transformations) in order for an optimization to be considered as having converged to the final
     * solution.
     */
    reg.setTransformationEpsilon(1e-6);
    /*Set the maximum distance threshold between two correspondent points in source <-> target. If the
     *distance is larger than this threshold, the points will be ignored in the alignment process.
     */
    reg.setMaxCorrespondenceDistance(0.5);
    //Set the maximum number of iterations the internal optimization should run for
    reg.setMaximumIterations(icp_num_iter_);

    PointCloudT::Ptr myMapCloud(new PointCloudT());
    PointCloudT::Ptr myScanCloud(new PointCloudT());
    //Convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object
    pcl::fromROSMsg(*map_cloud_, *myMapCloud);
    pcl::fromROSMsg(cloud2_, *myScanCloud);

    reg.setInputCloud(myScanCloud);
    reg.setInputTarget(myMapCloud);

    PointCloudT unused;
    int i = 0;
    //Call the registration algorithm which estimates the transformation and returns the transformed source
    reg.align(unused);
    //Get the final transformation matrix estimated by the registration method
    const Eigen::Matrix4f &transf = reg.getFinalTransformation();
   // tf::Transform t;
    matrixAsTransform(transf, t_);
    has_scan_transform_ = false;
    PointCloudT transformedCloud;
    pcl::transformPointCloud(*myScanCloud, transformedCloud, reg.getFinalTransformation());
    //compute inlier_percent of icp
    computeInlierPecent(transformedCloud);

    last_proccessed_scan_ = scan_in_time_ ;

    pcl::toROSMsg(transformedCloud, cloud2transformed_);
    has_scan_transform_ = true;
    // compute real transform info, actually R and t in icp math model
    icp_result_.dist = sqrt((t_.getOrigin().x() * t_.getOrigin().x()) + (t_.getOrigin().y() * t_.getOrigin().y()));
    icp_result_.angle_dist = t_.getRotation().getAngle();
    rot_axis_ = t_.getRotation().getAxis();
    t_ = t_ * old_pose_;

    tf::StampedTransform base_after_icp;
    if(!getTransform(base_after_icp, odom_frame_, "base_link", ros::Time(0)))
    {
      ROS_WARN("DIDNT get base pose at now");
      scan_callback_mutex_.unlock();
      return;
    }
    else
    {
      // final compute's result
      tf::Transform rel = base_at_laser_.inverseTimes(base_after_icp); // transform's times
      ROS_DEBUG("relative motion of robot while doing icp:%fcm %fdeg", rel.getOrigin().length(), rel.getRotation().getAngle() * 180 / M_PI);
      t_ = t_ * rel;
    }
    fitness_score_ = reg.getFitnessScore();
    icp_converge_ = reg.hasConverged();
    // publish icp result info
    publishIcpInfo();
 }
}

void ScanMapIcp::computeInlierPecent(PointCloudT &pc)
{
  icp_result_.inlier_percent = 0.0;
  std::vector<int> nn_indices(1); // save search results' index
  std::vector<float> nn_sqr_dists(1); // save search results' distance
  size_t inlier_num = 0;
  for (size_t k = 0; k < pc.points.size(); ++k)
  {
    //Search for all the nearest neighbors of the query point in a given radius
    if (map_tree_->radiusSearch(pc.points[k], icp_inlier_dist_, nn_indices, nn_sqr_dists, 1) != 0)
    {
      inlier_num += 1;
    }
  }
  if (pc.points.size() > 0)
  {
    icp_result_.inlier_percent = (double) inlier_num / (double) pc.points.size();
  }
}

void ScanMapIcp::publishIcpInfo()
{
  char msg_c_str[2048];
  sprintf(msg_c_str, "inliers:%f(%f) scan_age %f (%f age_threshold) dist %f ang_dist %f axis(%f %f %f) \
         fitting %f(icp_fitness_threshold %f)", icp_result_.inlier_percent, icp_inlier_threshold_,\
          scan_age_.toSec(), age_threshold_, icp_result_.dist, icp_result_.angle_dist, rot_axis_.x(),\
          rot_axis_.y(), rot_axis_.z(), fitness_score_, icp_fitness_threshold_);

  std_msgs::String str;
  str.data = msg_c_str;
  double cov = pose_covariance_trans_;
  //judge whether satisfy relocation
  if ((act_scan_ - last_time_sent_) > update_age_threshold_ && (icp_result_.dist > dist_threshold_ || icp_result_.angle_dist > angle_threshold_) \
     && (icp_result_.inlier_percent > icp_inlier_threshold_) && (icp_result_.angle_dist < angle_upper_threshold_))
  {
    last_time_sent_ = act_scan_;
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = t_.getOrigin().x();
    pose.pose.pose.position.y = t_.getOrigin().y();
    tf::Quaternion quat = t_.getRotation();
    tf::quaternionTFToMsg(quat,pose.pose.pose.orientation);
    float poseFactor = 0.03;
    float rotFactor = 0.1;
    pose.pose.covariance[6*0 + 0] = (cov * cov) * poseFactor;
    pose.pose.covariance[6*1 + 1] = (cov * cov) * poseFactor;
    pose.pose.covariance[6*5 + 5] = (M_PI /12.0 * M_PI /12.0) * rotFactor;
    // ROS_INFO("i %i converged %i score: %f", icp_converge_, fitness_score_);
    ROS_INFO("publish a new initial pose, dist %f angle_dist %f,setting pose:%.3f %.3f [frame=%s] ", \
             icp_result_.dist, icp_result_.angle_dist, pose.pose.pose.position.x, pose.pose.pose.position.y,\
             pose.header.frame_id.c_str());
    if (call_icp_msg_ == "start")
    {
      ROS_WARN("publish a new initial pose, dist %f angle_dist %f,setting pose:%.3f %.3f [frame=%s] ", \
               icp_result_.dist, icp_result_.angle_dist, pose.pose.pose.position.x, pose.pose.pose.position.y,\
               pose.header.frame_id.c_str());
      initial_pose_pub_.publish(pose);
    }
    str.data += "<< sent";
  }
  if (call_icp_msg_ == "start")
  {
    icp_run_info_pub_.publish(str);
  }
}
void ScanMapIcp::updateParams()
{
  params_were_updated_ = ros::Time::now();
  private_nh_.param("use_sim_time", use_sim_time_, false);
  private_nh_.param("icp_fitness_threshold", icp_fitness_threshold_, 100.0);
  private_nh_.param("age_threshold", age_threshold_, 1.0);
  private_nh_.param("angle_upper_threshold", angle_upper_threshold_, 1.0);
  private_nh_.param("angle_threshold", angle_threshold_, 0.01);
  private_nh_.param("update_age_threshold", update_age_threshold_, 1.0);
  private_nh_.param("dist_threshold", dist_threshold_, 0.01);
  private_nh_.param("icp_inlier_threshold", icp_inlier_threshold_, 0.88);
  private_nh_.param("icp_inlier_dist", icp_inlier_dist_, 0.1);
  private_nh_.param("icp_num_iter", icp_num_iter_, 250);
  private_nh_.param("pose_covariance_trans", pose_covariance_trans_, 0.5);
  private_nh_.param("scan_rate", scan_rate_, 2);
  if(scan_rate_ < .001)
  {
    scan_rate_ = .001;
  }
}
/*
void ScanMapIcp::baseMotionTypeSubCallback(const std_msgs::String::ConstPtr &data)
{
  if (data->data == "straight")
  {
    ROS_INFO("base is now go straight");
  }
  else if (data->data == "turn")
  {
    ROS_INFO("base is now turn!");
  }
  else if(data->data == "arc")
  {
    ROS_INFO("base is now go arc!");
  }
  else if (data->data == "static")
  {
    ROS_INFO("base is now static!");
  }
  else
  {}
  base_motion_type_ = data->data;
}*/
void ScanMapIcp::callIcpSubCallback(const std_msgs::String::ConstPtr &data)
{
  if (data->data == "start")
  {
    ROS_INFO("NOW, START TO CALL ICP!");
    call_icp_msg_ = data->data;
  }
  else if (data->data == "finish")
  {
    ROS_INFO("Now, FINSH CALL ICP!");
    call_icp_msg_ = data->data;
  }
  else
  {
    ROS_ERROR("INVALID MSG!");
  }
}
}
