lidar2camera
============

Visualize `sensor_msgs/PointCloud2` messages (e.g., from a lidar sensor) projected onto images. Use [`pangolin`](https://github.com/stevenlovegrove/Pangolin) GUI for fine-tuning of extrinsic sensor calibration. Inspired by the non-ROS sensor calibration toolbox, [OpenCalib](https://github.com/PJLab-ADG/SensorsCalibration).

## Getting Started

First install dependencies (e.g., `sudo apt install ros-noetic-tf2-sensor-msgs`) and build and source your ROS workspace.

### Manual Calibration

Run the manual calibration tool using

```bash
roslaunch lidar2camera manual_calib.launch
```

Note that you may need to change the `camera` or `points` arguments to ensure proper topic remapping. Also note that this code assumes that `image_raw`, `camera_info`, and `lidar_points` topics are published and sufficiently time synchronized.

### Calibration Visualization

Replace the `manual_calib.launch` above with `projection.launch` to publish a topic called `image_pts_raw` with the lidar points projected onto the image.