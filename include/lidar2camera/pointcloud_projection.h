/**
 * @file pointcloud_projection.h
 * @brief Project 3D points onto image
 * @author Parker Lusk <plusk@mit.edu>
 * @date 15 April 2023
 */

#pragma once

#include <algorithm>
#include <memory>
#include <tuple>

// #include <vector>
// #include <map>
// #include <string>
// #include <unordered_map>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/static_transform_broadcaster.h>
// #include <tf2_eigen/tf2_eigen.h>
// #include <eigen_conversions/eigen_msg.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
// #include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/opencv.hpp>

namespace l2c {

  class PointCloudProjection
  {
  public:
    PointCloudProjection(const ros::NodeHandle& nh, const ros::NodeHandle& nhp);
    ~PointCloudProjection() = default;

  private:
    ros::NodeHandle nh_, nhp_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_img_;
    // ros::Subscriber sub_odom_;
    // ros::Publisher pub_gt_odom_;

    tf2_ros::Buffer tfbuf_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_cinfo_;
    message_filters::Subscriber<sensor_msgs::Image> sub_img_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pcd_;

    //\brief Camera model
    image_geometry::PinholeCameraModel cam_model_;

    //\brief TimeSynchronizer
    typedef message_filters::sync_policies::ApproximateTime<
          sensor_msgs::CameraInfo, sensor_msgs::Image,
          sensor_msgs::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::unique_ptr<Sync> sync_;


    // std::map<double, Eigen::Affine3d> gtposes_;
    // Eigen::Affine3d T_mo_;

    // // tf broadcaster
    // tf2_ros::TransformBroadcaster tf_;
    // tf2_ros::StaticTransformBroadcaster tf_static_;

    // void init_tf_tree(const Eigen::Affine3d& T_mo);

    // std::map<double, Eigen::Affine3d> read_gt(const std::filesystem::path& datapath,
    //                                       const std::string& run,
    //                                       const std::string& robot);


    // void odom_cb(const nav_msgs::OdometryConstPtr& msg);


    void img_pcd_cb(const sensor_msgs::CameraInfoConstPtr& msg_cinfo,
                    const sensor_msgs::ImageConstPtr& msg_img,
                    const sensor_msgs::PointCloud2ConstPtr& msg_pcd);


    std::tuple<double, double, double> colormap_jet(double t);

    double map_range(double t, double il, double iu, double ol, double ou);

    cv::Scalar get_range_color(double range2);
    
  };

} // ns l2c
