/**
 * @file manual_calibrator_ros.h
 * @brief Manual lidar2camera calibration refinement
 * @author Parker Lusk <plusk@mit.edu>
 * @date 17 April 2023
 */

#pragma once

#include <algorithm>
#include <memory>
#include <tuple>
#include <thread>

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
// #include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3d);

namespace l2c {

  class ManualCalibrator
  {
  public:
    ManualCalibrator(const ros::NodeHandle& nh, const ros::NodeHandle& nhp);
    ~ManualCalibrator();

  private:
    ros::NodeHandle nh_, nhp_;
    // image_transport::ImageTransport it_;
    // image_transport::Publisher pub_img_;
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


    void img_pcd_cb(const sensor_msgs::CameraInfoConstPtr& msg_cinfo,
                    const sensor_msgs::ImageConstPtr& msg_img,
                    const sensor_msgs::PointCloud2ConstPtr& msg_pcd);


    std::tuple<double, double, double> colormap_jet(double t);

    double map_range(double t, double il, double iu, double ol, double ou);

    cv::Scalar get_range_color(double range2);


    Eigen::Affine3d T_CL_; ///< lidar w.r.t camera transform
    Eigen::Affine3d T_refined_; ///< left multiplied T_CL_ refinement
    std::vector<Eigen::Affine3d> T_modifiers_; ///< small changes for T_refined

    sensor_msgs::PointCloud2ConstPtr msg_pcd_; ///< latest received point cloud

    cv::Mat img_;
    int width_, height_; ///< image size
    bool initialized_ = false;

    std::thread renderloop_;

    double st_, sr_; /// trans [m] and rot [deg] refinement scale factor

    void run();

    void initialize();
    void project_points_onto_image(cv::Mat& img,
                                const image_geometry::PinholeCameraModel& cam,
                                const sensor_msgs::PointCloud2ConstPtr& msg_pcd,
                                const Eigen::Affine3d& T_CP,
                                bool image_rectified);

    void build_modifier_transforms();

    void print_transform(const std::string& name, const Eigen::Affine3d& T);

    
  };

} // ns l2c
