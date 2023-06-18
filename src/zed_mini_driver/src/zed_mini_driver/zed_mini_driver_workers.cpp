/**
 * Worker threads of the ZED Mini driver node.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 13, 2023
 */

#include <cmath>

#include <zed_mini_driver/zed_mini_driver.hpp>

namespace ZEDMiniDriver
{

/**
 * @brief Camera sampling routine: gets and publishes data from all sensors.
 *
 * @throws RuntimeError if the camera cannot be opened.s
 */
void ZEDMiniDriverNode::camera_routine()
{
  // Open camera
  if (!open_camera()) {
    throw std::runtime_error("ZEDMiniDriverNode::camera_routine: Failed to open camera");
  }

  // Prepare positional tracking data
  sl::Pose camera_pose;
  sl::POSITIONAL_TRACKING_STATE tracking_state;

  // Prepare sensors data
  sl::SensorsData sensor_data;

  // Prepare image sampling data
  sl::Resolution camera_res = zed_.getCameraInformation().camera_configuration.resolution;
  sl::Resolution sd_res(sd_width_, sd_height_);
  sl::Mat left_frame(camera_res, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
  sl::Mat right_frame(camera_res, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
  sl::Mat left_frame_sd(sd_res, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
  sl::Mat right_frame_sd(sd_res, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
  cv::Mat left_frame_cv = slMat2cvMat(left_frame);
  cv::Mat right_frame_cv = slMat2cvMat(right_frame);
  cv::Mat left_frame_cv_sd = slMat2cvMat(left_frame_sd);
  cv::Mat right_frame_cv_sd = slMat2cvMat(right_frame_sd);
  cv::Mat left_frame_cv_bgr(left_frame_cv.size(), CV_8UC3);
  cv::Mat right_frame_cv_bgr(left_frame_cv.size(), CV_8UC3);
  cv::Mat left_frame_cv_bgr_sd(left_frame_cv_sd.size(), CV_8UC3);
  cv::Mat right_frame_cv_bgr_sd(right_frame_cv_sd.size(), CV_8UC3);

  // Prepare depth sampling data
  sl::Mat depth_map_view;
  sl::Mat point_cloud;

  // Run until stopped
  sl::ERROR_CODE err;
  sl::RuntimeParameters runtime_params;
  runtime_params.measure3D_reference_frame = sl::REFERENCE_FRAME::WORLD;
  runtime_params.enable_depth = true;
  runtime_params.enable_fill_mode = false;
  runtime_params.remove_saturated_areas = true;
  while (running_.load(std::memory_order_acquire)) {
    // Grab data
    runtime_params.confidence_threshold = static_cast<int>(confidence_);
    runtime_params.texture_confidence_threshold = static_cast<int>(texture_confidence_);
    err = zed_.grab(runtime_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_FATAL(
        this->get_logger(),
        "ZEDMiniDriverNode::camera_routine: Failed to grab data (%d): %s",
        static_cast<int>(err),
        sl::toString(err).c_str());
      running_.store(false, std::memory_order_release);
      break;
    }

    // Get positional tracking data
    tracking_state = zed_.getPosition(camera_pose, sl::REFERENCE_FRAME::WORLD);
    if (verbose_) {
      switch (tracking_state) {
        case sl::POSITIONAL_TRACKING_STATE::OFF:
          RCLCPP_ERROR(this->get_logger(), "Positional tracking OFF");
          break;
        case sl::POSITIONAL_TRACKING_STATE::FPS_TOO_LOW:
          RCLCPP_ERROR(this->get_logger(), "FPS too low");
          break;
        case sl::POSITIONAL_TRACKING_STATE::SEARCHING:
          RCLCPP_WARN(this->get_logger(), "Track lost, relocalizing...");
          break;
        default:
          break;
      }
    }

    // Publish positional tracking data
    if (camera_pose.valid) {
      positional_tracking(camera_pose);
    }

    // Publish sensor data
    if (zed_.getSensorsData(sensor_data, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS) {
      sensor_sampling(sensor_data);
    }

    // Retrieve frames
    zed_.retrieveImage(left_frame, sl::VIEW::LEFT, sl::MEM::CPU, camera_res);
    zed_.retrieveImage(right_frame, sl::VIEW::RIGHT, sl::MEM::CPU, camera_res);
    zed_.retrieveImage(left_frame_sd, sl::VIEW::LEFT, sl::MEM::CPU, sd_res);
    zed_.retrieveImage(right_frame_sd, sl::VIEW::RIGHT, sl::MEM::CPU, sd_res);

    // Convert frames to BGR8 in OpenCV
    cv::cvtColor(left_frame_cv, left_frame_cv_bgr, cv::COLOR_BGRA2BGR);
    cv::cvtColor(right_frame_cv, right_frame_cv_bgr, cv::COLOR_BGRA2BGR);
    cv::cvtColor(left_frame_cv_sd, left_frame_cv_bgr_sd, cv::COLOR_BGRA2BGR);
    cv::cvtColor(right_frame_cv_sd, right_frame_cv_bgr_sd, cv::COLOR_BGRA2BGR);

    // Allocate Image messages
    Image::SharedPtr left_frame_msg = frame_to_msg(left_frame_cv_bgr);
    Image::SharedPtr right_frame_msg = frame_to_msg(right_frame_cv_bgr);
    Image::SharedPtr left_frame_msg_sd = frame_to_msg(left_frame_cv_bgr_sd);
    Image::SharedPtr right_frame_msg_sd = frame_to_msg(right_frame_cv_bgr_sd);

    // Set Image messages headers and metadata
    left_frame_msg->header.set__frame_id(link_namespace_ + "zedm_left_link");
    left_frame_msg->header.stamp.set__sec(static_cast<int32_t>(left_frame.timestamp.getSeconds()));
    left_frame_msg->header.stamp.set__nanosec(
      static_cast<uint32_t>(left_frame.timestamp.getNanoseconds() % uint64_t(1e9)));
    left_frame_msg->set__encoding(sensor_msgs::image_encodings::BGR8);
    left_frame_msg_sd->header.set__frame_id(link_namespace_ + "zedm_left_link");
    left_frame_msg_sd->header.stamp.set__sec(static_cast<int32_t>(left_frame.timestamp.getSeconds()));
    left_frame_msg_sd->header.stamp.set__nanosec(
      static_cast<uint32_t>(left_frame.timestamp.getNanoseconds() % uint64_t(1e9)));
    left_frame_msg_sd->set__encoding(sensor_msgs::image_encodings::BGR8);
    right_frame_msg->header.set__frame_id(link_namespace_ + "zedm_right_link");
    right_frame_msg->header.stamp.set__sec(static_cast<int32_t>(right_frame.timestamp.getSeconds()));
    right_frame_msg->header.stamp.set__nanosec(
      static_cast<uint32_t>(right_frame.timestamp.getNanoseconds() % uint64_t(1e9)));
    right_frame_msg->set__encoding(sensor_msgs::image_encodings::BGR8);
    right_frame_msg_sd->header.set__frame_id(link_namespace_ + "zedm_right_link");
    right_frame_msg_sd->header.stamp.set__sec(
      static_cast<int32_t>(right_frame.timestamp.getSeconds()));
    right_frame_msg_sd->header.stamp.set__nanosec(
      static_cast<uint32_t>(right_frame.timestamp.getNanoseconds() % uint64_t(1e9)));
    right_frame_msg_sd->set__encoding(sensor_msgs::image_encodings::BGR8);

    // Stamp camera_infos
    left_info_.header.stamp.set__sec(static_cast<int32_t>(left_frame.timestamp.getSeconds()));
    left_info_.header.stamp.set__nanosec(
      static_cast<uint32_t>(left_frame.timestamp.getNanoseconds() % uint64_t(1e9)));
    left_sd_info_.header.stamp.set__sec(static_cast<int32_t>(left_frame.timestamp.getSeconds()));
    left_sd_info_.header.stamp.set__nanosec(
      static_cast<uint32_t>(left_frame.timestamp.getNanoseconds() % uint64_t(1e9)));
    right_info_.header.stamp.set__sec(static_cast<int32_t>(left_frame.timestamp.getSeconds()));
    right_info_.header.stamp.set__nanosec(
      static_cast<uint32_t>(left_frame.timestamp.getNanoseconds() % uint64_t(1e9)));
    right_sd_info_.header.stamp.set__sec(static_cast<int32_t>(left_frame.timestamp.getSeconds()));
    right_sd_info_.header.stamp.set__nanosec(
      static_cast<uint32_t>(left_frame.timestamp.getNanoseconds() % uint64_t(1e9)));

    // Publish images
    if (left_rect_pub_->getNumSubscribers()) {
      left_rect_pub_->publish(*left_frame_msg, left_info_);
    }
    if (left_rect_sd_pub_->getNumSubscribers()) {
      left_rect_sd_pub_->publish(*left_frame_msg_sd, left_sd_info_);
    }
    if (right_rect_pub_->getNumSubscribers()) {
      right_rect_pub_->publish(*right_frame_msg, right_info_);
    }
    if (right_rect_sd_pub_->getNumSubscribers()) {
      right_rect_sd_pub_->publish(*right_frame_msg_sd, right_sd_info_);
    }
    if (left_stream_pub_->getNumSubscribers()) {
      if (stream_hd_) {
        left_stream_pub_->publish(*left_frame_msg);
      } else {
        left_stream_pub_->publish(*left_frame_msg_sd);
      }
    }
    if (right_stream_pub_->getNumSubscribers()) {
      if (stream_hd_) {
        right_stream_pub_->publish(*right_frame_msg);
      } else {
        right_stream_pub_->publish(*right_frame_msg_sd);
      }
    }

    // Get and process depth data
    err = zed_.retrieveImage(depth_map_view, sl::VIEW::DEPTH);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to retrieve depth map: %s",
        sl::toString(err).c_str());
      continue;
    }
    err = zed_.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to retrieve point cloud: %s",
        sl::toString(err).c_str());
      continue;
    }
    depth_sampling(depth_map_view, point_cloud);
  }

  // Close camera
  close_camera();
}

/**
 * @brief Processes positional tracking data.
 *
 * @param zed_pose ZED positional tracking data.
 */
void ZEDMiniDriverNode::positional_tracking(sl::Pose & camera_pose)
{
  // Parse pose data
  Header pose_header{};
  pose_header.set__frame_id(link_namespace_ + "zedm_odom");
  pose_header.stamp.set__sec(
    static_cast<int32_t>(camera_pose.timestamp.getNanoseconds() /
    uint64_t(1e9)));
  pose_header.stamp.set__nanosec(
    static_cast<uint32_t>(camera_pose.timestamp.getNanoseconds() %
    uint64_t(1e9)));
  Eigen::Vector3d position(
    static_cast<double>(camera_pose.getTranslation().tx),
    static_cast<double>(camera_pose.getTranslation().ty),
    static_cast<double>(camera_pose.getTranslation().tz));
  Eigen::Quaterniond orientation(
    static_cast<double>(camera_pose.getOrientation().ow),
    static_cast<double>(camera_pose.getOrientation().ox),
    static_cast<double>(camera_pose.getOrientation().oy),
    static_cast<double>(camera_pose.getOrientation().oz));
  std::array<double, 36> pose_covariance{};
  for (size_t i = 0; i < 36; i++) {
    pose_covariance[i] = static_cast<double>(camera_pose.pose_covariance[i]);
  }
  PoseKit::Pose zed_pose(
    position,
    orientation,
    pose_header,
    pose_covariance);

  // Parse twist data (it's in body frame, so left camera frame)
  Header twist_header{};
  twist_header.set__frame_id(link_namespace_ + "zedm_link");
  twist_header.stamp.set__sec(
    static_cast<int32_t>(camera_pose.timestamp.getSeconds()));
  twist_header.stamp.set__nanosec(
    static_cast<uint32_t>(camera_pose.timestamp.getNanoseconds() %
    uint64_t(1e9)));
  Eigen::Vector3d linear_velocity(
    static_cast<double>(camera_pose.twist[0]),
    static_cast<double>(camera_pose.twist[1]),
    static_cast<double>(camera_pose.twist[2]));
  Eigen::Vector3d angular_velocity(
    static_cast<double>(camera_pose.twist[3]),
    static_cast<double>(camera_pose.twist[4]),
    static_cast<double>(camera_pose.twist[5]));
  std::array<double, 36> twist_covariance{};
  for (size_t i = 0; i < 36; i++) {
    twist_covariance[i] = static_cast<double>(camera_pose.twist_covariance[i]);
  }
  PoseKit::KinematicPose zed_twist(
    Eigen::Vector3d::Zero(),
    Eigen::Quaterniond::Identity(),
    linear_velocity,
    angular_velocity,
    twist_header,
    std::array<double, 36>{},
    twist_covariance);

  // Get base_link pose
  PoseKit::Pose base_link_pose = zed_pose;
  try {
    tf_lock_.lock();
    base_link_pose.track_parent(odom_to_camera_odom_);
    tf_lock_.unlock();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "ZEDMiniDriverNode::camera_routine: base_link_pose::track_parent: %s",
      e.what());
    tf_lock_.unlock();
  }

  // Get base_link twist
  PoseKit::KinematicPose base_link_twist = zed_twist;
  try {
    tf_lock_.lock();
    base_link_twist.track_parent(base_link_to_camera_);
    tf_lock_.unlock();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "ZEDMiniDriverNode::camera_routine: base_link_twist::track_parent: %s",
      e.what());
    tf_lock_.unlock();
  }

  // Build odometry messages
  Odometry camera_odom_msg{}, base_link_odom_msg{};
  camera_odom_msg.set__header(zed_pose.get_header());
  camera_odom_msg.set__child_frame_id(link_namespace_ + "zedm_link");
  camera_odom_msg.set__pose(zed_pose.to_pose_with_covariance_stamped().pose);
  camera_odom_msg.set__twist(zed_twist.to_twist_with_covariance_stamped().twist);
  base_link_odom_msg.set__header(base_link_pose.get_header());
  base_link_odom_msg.set__child_frame_id(link_namespace_ + "base_link");
  base_link_odom_msg.set__pose(base_link_pose.to_pose_with_covariance_stamped().pose);
  base_link_odom_msg.set__twist(base_link_twist.to_twist_with_covariance_stamped().twist);

  // Publish odometry messages
  base_link_odom_pub_->publish(base_link_odom_msg);
  camera_odom_pub_->publish(camera_odom_msg);
  rviz_base_link_odom_pub_->publish(base_link_odom_msg);
  rviz_camera_odom_pub_->publish(camera_odom_msg);

  // Publish pose messages
  base_link_pose_pub_->publish(base_link_pose.to_pose_with_covariance_stamped());
  camera_pose_pub_->publish(zed_pose.to_pose_with_covariance_stamped());
  rviz_base_link_pose_pub_->publish(base_link_pose.to_pose_with_covariance_stamped());
  rviz_camera_pose_pub_->publish(zed_pose.to_pose_with_covariance_stamped());
}

/**
 * @brief Publishes onboard sensors data.
 *
 * @param sensors_data Sensors data.
 */
void ZEDMiniDriverNode::sensor_sampling(sl::SensorsData & sensors_data)
{
  // Process IMU data
  sl::SensorsData::IMUData imu_data = sensors_data.imu;
  if (imu_data.is_available) {
    Imu imu_msg{};
    imu_msg.header.stamp.set__sec(static_cast<int32_t>(imu_data.timestamp.getSeconds()));
    imu_msg.header.stamp.set__nanosec(
      static_cast<uint32_t>(imu_data.timestamp.getNanoseconds() % uint64_t(1e9)));
    imu_msg.header.set__frame_id(link_namespace_ + "zedm_imu_link");

    imu_msg.orientation.set__w(static_cast<double>(imu_data.pose.getOrientation().ow));
    imu_msg.orientation.set__x(static_cast<double>(imu_data.pose.getOrientation().ox));
    imu_msg.orientation.set__y(static_cast<double>(imu_data.pose.getOrientation().oy));
    imu_msg.orientation.set__z(static_cast<double>(imu_data.pose.getOrientation().oz));

    for (int i = 0; i < 9; ++i) {
      imu_msg.orientation_covariance[i] = static_cast<double>(imu_data.pose_covariance.r[i]);
    }

    imu_msg.angular_velocity.set__x(
      static_cast<double>((M_PI / 180.0) * imu_data.angular_velocity.x));
    imu_msg.angular_velocity.set__y(
      static_cast<double>((M_PI / 180.0) * imu_data.angular_velocity.y));
    imu_msg.angular_velocity.set__z(
      static_cast<double>((M_PI / 180.0) * imu_data.angular_velocity.z));

    for (int i = 0; i < 9; ++i) {
      imu_msg.angular_velocity_covariance[i] =
        static_cast<double>((M_PI / 180.0) * imu_data.angular_velocity_covariance.r[i]);
    }

    imu_msg.linear_acceleration.set__x(static_cast<double>(imu_data.linear_acceleration.x));
    imu_msg.linear_acceleration.set__y(static_cast<double>(imu_data.linear_acceleration.y));
    imu_msg.linear_acceleration.set__z(static_cast<double>(imu_data.linear_acceleration.z));

    for (int i = 0; i < 9; ++i) {
      imu_msg.linear_acceleration_covariance[i] =
        static_cast<double>(imu_data.linear_acceleration_covariance.r[i]);
    }

    imu_pub_->publish(imu_msg);
  }
}

/**
 * @brief Processes depth data.
 *
 * @param depth_map_view Depth map for visualization.
 * @param point_cloud Point cloud.
 */
void ZEDMiniDriverNode::depth_sampling(sl::Mat & depth_map_view, sl::Mat & point_cloud)
{
  // Publish depth map image
  cv::Mat depth_map_view_cv = slMat2cvMatDepth(depth_map_view);
  Image::SharedPtr depth_msg = frame_to_msg(depth_map_view_cv);
  depth_msg->header.set__frame_id(link_namespace_ + "zedm_link");
  depth_msg->header.stamp.set__sec(static_cast<int32_t>(depth_map_view.timestamp.getSeconds()));
  depth_msg->header.stamp.set__nanosec(
    static_cast<uint32_t>(depth_map_view.timestamp.getNanoseconds() % uint64_t(1e9)));
  depth_msg->set__encoding(sensor_msgs::image_encodings::BGRA8);
  if (depth_pub_->getNumSubscribers()) {
    depth_pub_->publish(*depth_msg);
  }
}

} // namespace ZEDMiniDriver
