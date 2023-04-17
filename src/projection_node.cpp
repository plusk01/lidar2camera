/**
 * @file projection_node.cpp
 * @brief Project points onto image
 * @author Parker Lusk <plusk@mit.edu>
 * @date 15 April 2023
 */

#include <ros/ros.h>

#include "lidar2camera/pointcloud_projection.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "projection");
  ros::NodeHandle nhtopics("");
  ros::NodeHandle nhparams("~");
  l2c::PointCloudProjection node(nhtopics, nhparams);
  ros::spin();
  return 0;
}
