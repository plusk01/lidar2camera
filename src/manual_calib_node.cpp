/**
 * @file manual_calib_node.cpp
 * @brief Manual lidar2camera calibration refinement
 * @author Parker Lusk <plusk@mit.edu>
 * @date 17 April 2023
 */

#include <ros/ros.h>

#include "lidar2camera/manual_calibrator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manual_lidar2camera_calib");
  ros::NodeHandle nhtopics("");
  ros::NodeHandle nhparams("~");
  l2c::ManualCalibrator node(nhtopics, nhparams);
  ros::spin();
  return 0;
}
