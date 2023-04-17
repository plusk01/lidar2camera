/**
 * @file pointcloud_projection.cpp
 * @brief Project 3D points onto image
 * @author Parker Lusk <plusk@mit.edu>
 * @date 15 April 2023
 */

#include <iostream>

#include "lidar2camera/pointcloud_projection.h"

namespace l2c {

PointCloudProjection::PointCloudProjection(const ros::NodeHandle& nh, const ros::NodeHandle& nhp)
: nh_(nh), nhp_(nhp), it_(nh)
{
  // // Check namespace to find name of robot -- leverage fs module
  // fs::path ns(ros::this_node::getNamespace());
  // robot_ = ns.filename();
  // if (robot_.empty()) {
  //   ROS_FATAL("Required robot namespace is missing. Hint: use launch file "
  //             "with node namespacing or `rosrun ... __ns:=robot_name`\n");
  //   ros::shutdown();
  //   return;
  // }
  // ROS_INFO_STREAM("Robot: " << robot_);


  // double min_range;
  // if (!nhp_.getParam("dataroot", dataroot_)) {
  //   ROS_FATAL("Must specify Kimera-Multi Data root in param `dataroot`");
  //   ros::shutdown();
  //   return;
  // }

  // datapath_ = dataroot_;
  // if (!fs::exists(datapath_)) {
  //   ROS_FATAL_STREAM("data root `" << dataroot_ << "` does not exist");
  //   ros::shutdown();
  //   return; 
  // }

  // int seq;
  // if (!nhp_.getParam("sequence", seq)) {
  //   ROS_FATAL("Must specify which sequence in param `sequence` (1014, 1207, or 1208).");
  //   ros::shutdown();
  //   return;
  // } else {
  //   if (seq != 1014 && seq != 1207 && seq != 1208) {
  //     ROS_FATAL("The `sequence` param can only be 1014, 1207, or 1208.");
  //     ros::shutdown();
  //     return;
  //   }
  //   sequence_ = std::to_string(seq);
  // }
  // 


  tf_listener_.reset(new tf2_ros::TransformListener(tfbuf_));

  pub_img_ = it_.advertise("image_pts_raw", 1);

  sub_cinfo_.subscribe(nh_, "camera_info", 1);
  sub_img_.subscribe(nh_, "image_raw", 1);
  sub_pcd_.subscribe(nh_, "points", 1);

  sync_.reset(new Sync(MySyncPolicy(30), sub_cinfo_, sub_img_, sub_pcd_));
  sync_->registerCallback(boost::bind(&PointCloudProjection::img_pcd_cb, this, _1, _2, _3));
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

std::tuple<double, double, double> PointCloudProjection::colormap_jet(double t)
{
  // JET colors
  // https://stackoverflow.com/a/46628410/2392520
  const double red   = std::clamp(1.5 - std::abs(2.0 * t - 1.0), 0.0, 1.0);
  const double green = std::clamp(1.5 - std::abs(2.0 * t), 0.0, 1.0);
  const double blue  = std::clamp(1.5 - std::abs(2.0 * t + 1.0), 0.0, 1.0);

  return std::make_tuple(red, green, blue);
}

// ----------------------------------------------------------------------------

double PointCloudProjection::map_range(double t,
                                        double il, double iu,
                                        double ol, double ou)
{
  t = std::clamp(t, il, iu);
  const double slope = (ou - ol) / (iu - il);
  return ol + slope * (t - il);
}

// ----------------------------------------------------------------------------

cv::Scalar PointCloudProjection::get_range_color(double range2)
{
  const double t = map_range(range2, 1.5*1.5, 15*15, -1, 1);

  const auto [red, green, blue] = colormap_jet(t);
  
  const int R = static_cast<int>(red * 255);
  const int G = static_cast<int>(green * 255);
  const int B = static_cast<int>(blue * 255);

  return cv::Scalar(B, G, R);
}

// ----------------------------------------------------------------------------
// ROS Callbacks
// ----------------------------------------------------------------------------

void PointCloudProjection::img_pcd_cb(
  const sensor_msgs::CameraInfoConstPtr& msg_cinfo,
  const sensor_msgs::ImageConstPtr& msg_img,
  const sensor_msgs::PointCloud2ConstPtr& msg_pcd)
{
  // cv::Mat image;
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
    // image = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8)->image;
  } catch (cv_bridge::Exception& ex) {
    ROS_ERROR("[pointcloud_projection] Failed to convert image");
    return;
  }

  cam_model_.fromCameraInfo(msg_cinfo);

  // don't do the work if no one is listening
  if (pub_img_.getNumSubscribers() == 0) return;

  //
  // Express point cloud in camera frame (optical frame)
  //

  sensor_msgs::PointCloud2 msg_pcd_transformed;
  try {
    const geometry_msgs::TransformStamped T = tfbuf_.lookupTransform(
                                                msg_img->header.frame_id,
                                                msg_pcd->header.frame_id,
                                                ros::Time(0));

    tf2::doTransform(*msg_pcd, msg_pcd_transformed, T);

  } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
  }

  //
  // Project points onto camera
  //

  const cv::Size sz = cam_model_.fullResolution();

  sensor_msgs::PointCloud2ConstIterator<float> it(msg_pcd_transformed, "x");
  for (; it != it.end(); ++it) {
    const cv::Point3d xyz{it[0], it[1], it[2]};

    if (xyz.z < 0) continue;

    const cv::Point2d uv_rect = cam_model_.project3dToPixel(xyz);

    if ((uv_rect.x >= 0 && uv_rect.x < sz.width) && (uv_rect.y >= 0 && uv_rect.y < sz.height)) {
      const cv::Point2d uv_raw = cam_model_.unrectifyPoint(uv_rect);
      const double range2 = xyz.dot(xyz);
      cv::circle(cv_ptr->image, uv_raw, 1, get_range_color(range2), -1);
    }
  }

  pub_img_.publish(cv_ptr->toImageMsg());
}

} // ns l2c
