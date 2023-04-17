/**
 * @file manual_calibrator_ros.cpp
 * @brief Manual lidar2camera calibration refinement
 * @author Parker Lusk <plusk@mit.edu>
 * @date 17 April 2023
 */

#include <iostream>

#include <pangolin/pangolin.h>

#include <eigen_conversions/eigen_msg.h>

#include "lidar2camera/manual_calibrator.h"

namespace l2c {

static const std::string window_name = "lidar2camera";

ManualCalibrator::ManualCalibrator(const ros::NodeHandle& nh, const ros::NodeHandle& nhp)
: nh_(nh), nhp_(nhp), T_CL_(Eigen::Affine3d::Identity()),
  T_refined_(Eigen::Affine3d::Identity()), sr_(0.3), st_(0.01)
{
  tf_listener_.reset(new tf2_ros::TransformListener(tfbuf_));

  sub_cinfo_.subscribe(nh_, "camera_info", 1);
  sub_img_.subscribe(nh_, "image_raw", 1);
  sub_pcd_.subscribe(nh_, "points", 1);

  sync_.reset(new Sync(MySyncPolicy(30), sub_cinfo_, sub_img_, sub_pcd_));
  sync_->registerCallback(boost::bind(&ManualCalibrator::img_pcd_cb, this, _1, _2, _3));
}

// ----------------------------------------------------------------------------

ManualCalibrator::~ManualCalibrator()
{
  pangolin::Quit();
  if (renderloop_.joinable()) renderloop_.join();
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

std::tuple<double, double, double> ManualCalibrator::colormap_jet(double t)
{
  // JET colors
  // https://stackoverflow.com/a/46628410/2392520
  const double red   = std::clamp(1.5 - std::abs(2.0 * t - 1.0), 0.0, 1.0);
  const double green = std::clamp(1.5 - std::abs(2.0 * t), 0.0, 1.0);
  const double blue  = std::clamp(1.5 - std::abs(2.0 * t + 1.0), 0.0, 1.0);

  return std::make_tuple(red, green, blue);
}

// ----------------------------------------------------------------------------

double ManualCalibrator::map_range(double t,
                                        double il, double iu,
                                        double ol, double ou)
{
  t = std::clamp(t, il, iu);
  const double slope = (ou - ol) / (iu - il);
  return ol + slope * (t - il);
}

// ----------------------------------------------------------------------------

cv::Scalar ManualCalibrator::get_range_color(double range2)
{
  const double t = map_range(range2, 1.5*1.5, 15*15, -1, 1);

  const auto [red, green, blue] = colormap_jet(t);
  
  const int R = static_cast<int>(red * 255);
  const int G = static_cast<int>(green * 255);
  const int B = static_cast<int>(blue * 255);

  return cv::Scalar(B, G, R);
}

// ----------------------------------------------------------------------------

void ManualCalibrator::initialize()
{
  width_ = img_.cols;
  height_ = img_.rows;
  pangolin::CreateWindowAndBind(window_name, width_, height_);

  glEnable(GL_DEPTH_TEST);

  // unset the current context from the main thread
  pangolin::GetBoundWindow()->RemoveCurrent();

  renderloop_ = std::thread(&ManualCalibrator::run, this);

  initialized_ = true;
}

// ----------------------------------------------------------------------------

void ManualCalibrator::project_points_onto_image(cv::Mat& img,
                                const image_geometry::PinholeCameraModel& cam,
                                const sensor_msgs::PointCloud2ConstPtr& msg_pcd,
                                const Eigen::Affine3d& T_CP,
                                bool image_rectified)
{
  const cv::Size sz = img.size();
  sensor_msgs::PointCloud2ConstIterator<float> it(*msg_pcd, "x");
  for (; it != it.end(); ++it) {

    // express point in camera frame
    const Eigen::Vector3d p = T_CP * Eigen::Vector3d(it[0], it[1], it[2]);

    // ensure that point is in front of camera
    const cv::Point3d xyz{p.x(), p.y(), p.z()};
    if (xyz.z < 0) continue;

    // project 3D point as rectified (i.e., undistorted) 2D point
    const cv::Point2d uv_rect = cam.project3dToPixel(xyz);

    if ((uv_rect.x >= 0 && uv_rect.x < sz.width) && (uv_rect.y >= 0 && uv_rect.y < sz.height)) {
      const cv::Point2d uv = (image_rectified) ? uv_rect : cam.unrectifyPoint(uv_rect);
      const double range2 = xyz.dot(xyz);
      cv::circle(img, uv, 1, get_range_color(range2), -1);
    }
  }
}

// ----------------------------------------------------------------------------

void ManualCalibrator::build_modifier_transforms()
{
  static constexpr int NUM_DOF = 6; ///< each trans and rot DOF
  static constexpr int NUM_MODIFIERS = 2 * NUM_DOF; ///< +/- for each DOF

  T_modifiers_.resize(NUM_MODIFIERS);
  for (size_t i=0; i<NUM_MODIFIERS; i++) {

    // select which degree of freedom will be altered (and if + or -)
    std::vector<int> basis(6, 0);
    basis[i / 2] = (i % 2) ? -1 : 1;

    // compute the delta change in each DOF
    const double roll = basis[0] * sr_ * M_PI / 180.;
    const double pitch = basis[1] * sr_ * M_PI / 180.;
    const double yaw = basis[2] * sr_ * M_PI / 180.;
    const double x = basis[3] * st_;
    const double y = basis[4] * st_;
    const double z = basis[5] * st_;

    // create modifier SE3 matrix
    T_modifiers_[i] = Eigen::Affine3d::Identity();
    T_modifiers_[i].translation() = Eigen::Vector3d(x, y, z);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    T_modifiers_[i].linear() = R;
  }

}

// ----------------------------------------------------------------------------

void ManualCalibrator::run()
{
  // fetch the context and bind it to this thread
  pangolin::BindToContext(window_name);

  // we manually need to restore the properties of the context
  glEnable(GL_DEPTH_TEST);

  // pangolin::OpenGlRenderState s_cam(
  //     pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
  //     pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, 0.0, 1.0, 0.0));

  pangolin::View &project_image = pangolin::Display("project")
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(150), 1.0,
                     -1.0 * (width_ - 200) / height_)
          .SetLock(pangolin::LockLeft, pangolin::LockTop);

  unsigned char *imageArray = new unsigned char[3 * width_ * height_];
  pangolin::GlTexture imageTexture(width_, height_, GL_RGB, false, 0, GL_RGB,
                                   GL_UNSIGNED_BYTE);

  // control panel
  pangolin::CreatePanel("cp").SetBounds(pangolin::Attach::Pix(30), 1.0, 0.0,
                                        pangolin::Attach::Pix(150));
  // pangolin::Var<bool> displayMode("cp.Intensity Color", false, true);
  // pangolin::Var<bool> filterMode("cp.Overlap Filter", false, true);
  pangolin::Var<bool> rectifyMode("cp.Rectify Image", false, true);
  pangolin::Var<double> degreeStep("cp.deg step", sr_, 0, 1);
  pangolin::Var<double> tStep("cp.t step(cm)", st_ * 1e2, 0, 15);
  pangolin::Var<double> fxfyScale("cp.fxfy scale", 1.005, 1, 1.1);
  pangolin::Var<int> pointSize("cp.point size", 2, 1, 5);

  pangolin::Var<bool> addXdegree("cp.+ x degree", false, false);
  pangolin::Var<bool> minusXdegree("cp.- x degree", false, false);
  pangolin::Var<bool> addYdegree("cp.+ y degree", false, false);
  pangolin::Var<bool> minusYdegree("cp.- y degree", false, false);
  pangolin::Var<bool> addZdegree("cp.+ z degree", false, false);
  pangolin::Var<bool> minusZdegree("cp.- z degree", false, false);
  pangolin::Var<bool> addXtrans("cp.+ x trans", false, false);
  pangolin::Var<bool> minusXtrans("cp.- x trans", false, false);
  pangolin::Var<bool> addYtrans("cp.+ y trans", false, false);
  pangolin::Var<bool> minusYtrans("cp.- y trans", false, false);
  pangolin::Var<bool> addZtrans("cp.+ z trans", false, false);
  pangolin::Var<bool> minusZtrans("cp.- z trans", false, false);

  // pangolin::Var<bool> addFx("cp.+ fx", false, false);
  // pangolin::Var<bool> minusFx("cp.- fx", false, false);
  // pangolin::Var<bool> addFy("cp.+ fy", false, false);
  // pangolin::Var<bool> minusFy("cp.- fy", false, false);

  pangolin::Var<bool> resetButton("cp.Reset", false, false);
  pangolin::Var<bool> printTransform("cp.Print Transform", false, false);

  std::vector<pangolin::Var<bool>> refinement_modifiers;
  refinement_modifiers.push_back(addXdegree);
  refinement_modifiers.push_back(minusXdegree);
  refinement_modifiers.push_back(addYdegree);
  refinement_modifiers.push_back(minusYdegree);
  refinement_modifiers.push_back(addZdegree);
  refinement_modifiers.push_back(minusZdegree);
  refinement_modifiers.push_back(addXtrans);
  refinement_modifiers.push_back(minusXtrans);
  refinement_modifiers.push_back(addYtrans);
  refinement_modifiers.push_back(minusYtrans);
  refinement_modifiers.push_back(addZtrans);
  refinement_modifiers.push_back(minusZtrans);


  build_modifier_transforms();
  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    cv::Mat current_frame;
    if (rectifyMode) {
      cam_model_.rectifyImage(img_, current_frame);
    } else {
      current_frame = img_.clone();
    }
    project_points_onto_image(current_frame, cam_model_, msg_pcd_, T_refined_ * T_CL_, rectifyMode);

    if (degreeStep.GuiChanged()) {
      sr_ = degreeStep.Get();
      build_modifier_transforms();
    }

    if (tStep.GuiChanged()) {
      st_ = tStep.Get() * 1e-2;
      build_modifier_transforms();
    }

    if (fxfyScale.GuiChanged()) {
      // cali_scale_fxfy_ = fxfyScale.Get();
      // std::cout << "fxfy calib scale changed to " << cali_scale_fxfy_ << std::endl;
    }
    if (pointSize.GuiChanged()) {
      int ptsize = pointSize.Get();
      // projector.setPointSize(ptsize);
      // current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist, calibration_matrix_);
      // std::cout << "point size changed to " << ptsize << std::endl;
    }

    for (size_t i=0; i<refinement_modifiers.size(); i++) {
      if (pangolin::Pushed(refinement_modifiers[i])) {
        T_refined_ = T_refined_ * T_modifiers_[i];
      }
    }

    // if (pangolin::Pushed(addFx)) {
    //   // intrinsic_matrix_(0, 0) *= cali_scale_fxfy_;
    //   // current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist, calibration_matrix_);
    //   // std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
    // }
    // if (pangolin::Pushed(minusFx)) {
    //   // intrinsic_matrix_(0, 0) /= cali_scale_fxfy_;
    //   // current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist, calibration_matrix_);
    //   // std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
    // }
    // if (pangolin::Pushed(addFy)) {
    //   // intrinsic_matrix_(1, 1) *= cali_scale_fxfy_;
    //   // current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist, calibration_matrix_);
    //   // std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
    // }
    // if (pangolin::Pushed(minusFy)) {
    //   // intrinsic_matrix_(1, 1) /= cali_scale_fxfy_;
    //   // current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist, calibration_matrix_);
    //   // std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
    // }

    if (pangolin::Pushed(resetButton)) {
      T_refined_ = Eigen::Affine3d::Identity();
    }
    if (pangolin::Pushed(printTransform)) {
      print_transform("Refinement", T_refined_);
      print_transform("Total", T_refined_ * T_CL_);
    }

    // if (kbhit()) {
    //   int c = getchar();
    //   // if (ManualCalibration(c)) {
    //     // Eigen::Matrix4d transform = calibration_matrix_;
    //     // cout << "\nTransfromation Matrix:\n" << transform << std::endl;
    //   // }
    //   // current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist, calibration_matrix_);
    // }

    imageArray = current_frame.data;
    imageTexture.Upload(imageArray, GL_BGR, GL_UNSIGNED_BYTE);

    project_image.Activate();
    glColor3f(1.0, 1.0, 1.0);
    imageTexture.RenderToViewportFlipY();

    pangolin::FinishFrame();
    glFinish();
  }
}

// ----------------------------------------------------------------------------

void ManualCalibrator::print_transform(const std::string& name,
                                        const Eigen::Affine3d& T)
{
  std::cout << std::endl << std::endl;
  std::cout << name << ": " << std::endl;
  std::cout << T.matrix() << std::endl << std::endl;

  Eigen::Quaterniond q;
  q = T.rotation();

  std::cout << "x y z qw qx qy qz" << std::endl;
  std::cout << T.translation().x() << " "
            << T.translation().y() << " "
            << T.translation().z() << " "
            << q.w() << " "
            << q.x() << " "
            << q.y() << " "
            << q.z() << std::endl << std::endl;

}

// ----------------------------------------------------------------------------
// ROS Callbacks
// ----------------------------------------------------------------------------

void ManualCalibrator::img_pcd_cb(
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
  // const cv::Size sz = cam_model_.fullResolution();
  img_ = cv_ptr->image.clone();
  msg_pcd_ = msg_pcd;

  //
  // Express point cloud in camera frame (optical frame)
  //

  try {
    const geometry_msgs::TransformStamped T = tfbuf_.lookupTransform(
                                                msg_img->header.frame_id,
                                                msg_pcd->header.frame_id,
                                                ros::Time(0));

    // tf2::doTransform(*msg_pcd, msg_pcd_transformed_, T);

    tf::transformMsgToEigen(T.transform, T_CL_);

  } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
  }

  if (!initialized_) initialize();
}

} // ns l2c
