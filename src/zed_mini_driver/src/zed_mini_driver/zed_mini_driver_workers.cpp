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

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

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
  Image::SharedPtr left_frame_msg;
  Image::SharedPtr left_frame_msg_sd;
  Image::SharedPtr right_frame_msg;
  Image::SharedPtr right_frame_msg_sd;

  // Prepare stopwatches
  rclcpp::Time curr_ts = this->get_clock()->now();
  rclcpp::Duration depth_period(
    std::chrono::nanoseconds(int(1.0 / double(depth_rate_ > 0 ? depth_rate_ : fps_) * 1e9)));
  rclcpp::Duration video_period(
    std::chrono::nanoseconds(int(1.0 / double(video_rate_ > 0 ? video_rate_ : fps_) * 1e9)));
  last_depth_ts_ = curr_ts;
  last_video_ts_ = curr_ts;

  // Spawn depth processing thread
  depth_thread_ = std::thread{
    &ZEDMiniDriverNode::depth_routine,
    this};

  RCLCPP_INFO(this->get_logger(), "Camera sampling thread started");

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
    PoseKit::Pose curr_pose{};
    if (camera_pose.valid) {
      curr_pose = positional_tracking(camera_pose);
    }

    // Publish sensor data
    if (zed_.getSensorsData(sensor_data, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS) {
      sensor_sampling(sensor_data);
    }

    curr_ts = this->get_clock()->now();

    if (camera_pose.valid &&
      (((curr_ts - last_depth_ts_) >= depth_period) ||
      (depth_rate_ == 0) ||
      (depth_rate_ >= fps_)) &&
      (sem_trywait(&depth_sem_1_) == 0))
    {
      // Get depth data and post it for processing
      err = zed_.retrieveImage(depth_map_view_, sl::VIEW::DEPTH);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(
          this->get_logger(),
          "ZEDMiniDriverNode::camera_routine: Failed to retrieve depth map: %s",
          sl::toString(err).c_str());
        sem_post(&depth_sem_1_);
        continue;
      }
      err = zed_.retrieveMeasure(depth_point_cloud_, sl::MEASURE::XYZBGRA);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(
          this->get_logger(),
          "ZEDMiniDriverNode::camera_routine: Failed to retrieve point cloud: %s",
          sl::toString(err).c_str());
        sem_post(&depth_sem_1_);
        continue;
      }
      depth_curr_pose_ = curr_pose;
      last_depth_ts_ = curr_ts;
      sem_post(&depth_sem_2_);
    }

    if (((curr_ts - last_video_ts_) >= video_period) ||
      (video_rate_ == 0) ||
      (video_rate_ >= fps_))
    {
      // Process and publish RGB frames
      // This is the sequence, for each stream:
      // - Retrieve frames
      // - Convert frames to BGR8 in OpenCV
      // - Allocate Image messages
      // - Set Image messages headers and metadata
      // - Stamp camera_infos
      // - Publish images

      // Left streams
      if (left_rect_pub_->getNumSubscribers() ||
        left_rect_sd_pub_->getNumSubscribers() ||
        left_stream_pub_->getNumSubscribers())
      {
        // HD streams
        if (left_rect_pub_->getNumSubscribers() ||
          (left_stream_pub_->getNumSubscribers() && stream_hd_))
        {
          zed_.retrieveImage(left_frame, sl::VIEW::LEFT, sl::MEM::CPU, camera_res);

          cv::cvtColor(left_frame_cv, left_frame_cv_bgr, cv::COLOR_BGRA2BGR);

          left_frame_msg = frame_to_msg(left_frame_cv_bgr);

          left_frame_msg->header.set__frame_id(link_namespace_ + "zedm_left_link");
          left_frame_msg->header.stamp.set__sec(
            static_cast<int32_t>(left_frame.timestamp.getSeconds()));
          left_frame_msg->header.stamp.set__nanosec(
            static_cast<uint32_t>(left_frame.timestamp.getNanoseconds() % uint64_t(1e9)));
          left_frame_msg->set__encoding(sensor_msgs::image_encodings::BGR8);

          left_info_.header.stamp.set__sec(static_cast<int32_t>(left_frame.timestamp.getSeconds()));
          left_info_.header.stamp.set__nanosec(
            static_cast<uint32_t>(left_frame.timestamp.getNanoseconds() % uint64_t(1e9)));

          // left_rect
          if (left_rect_pub_->getNumSubscribers()) {
            left_rect_pub_->publish(*left_frame_msg, left_info_);
          }

          // left_stream
          if (left_stream_pub_->getNumSubscribers() && stream_hd_) {
            left_stream_pub_->publish(*left_frame_msg);
          }
        }

        // SD streams
        if (left_rect_sd_pub_->getNumSubscribers() ||
          (left_stream_pub_->getNumSubscribers() && !stream_hd_))
        {
          zed_.retrieveImage(left_frame_sd, sl::VIEW::LEFT, sl::MEM::CPU, sd_res);

          cv::cvtColor(left_frame_cv_sd, left_frame_cv_bgr_sd, cv::COLOR_BGRA2BGR);

          left_frame_msg_sd = frame_to_msg(left_frame_cv_bgr_sd);

          left_frame_msg_sd->header.set__frame_id(link_namespace_ + "zedm_left_link");
          left_frame_msg_sd->header.stamp.set__sec(
            static_cast<int32_t>(left_frame_sd.timestamp.getSeconds()));
          left_frame_msg_sd->header.stamp.set__nanosec(
            static_cast<uint32_t>(left_frame_sd.timestamp.getNanoseconds() % uint64_t(1e9)));
          left_frame_msg_sd->set__encoding(sensor_msgs::image_encodings::BGR8);

          left_sd_info_.header.stamp.set__sec(
            static_cast<int32_t>(left_frame_sd.timestamp.getSeconds()));
          left_sd_info_.header.stamp.set__nanosec(
            static_cast<uint32_t>(left_frame_sd.timestamp.getNanoseconds() % uint64_t(1e9)));

          // left_rect_sd
          if (left_rect_sd_pub_->getNumSubscribers()) {
            left_rect_sd_pub_->publish(*left_frame_msg_sd, left_sd_info_);
          }

          // left_stream
          if (left_stream_pub_->getNumSubscribers() && !stream_hd_) {
            left_stream_pub_->publish(*left_frame_msg_sd);
          }
        }
      }

      // Right streams
      if (right_rect_pub_->getNumSubscribers() ||
        right_rect_sd_pub_->getNumSubscribers() ||
        right_stream_pub_->getNumSubscribers())
      {
        // HD streams
        if (right_rect_pub_->getNumSubscribers() ||
          (right_stream_pub_->getNumSubscribers() && stream_hd_))
        {
          zed_.retrieveImage(right_frame, sl::VIEW::RIGHT, sl::MEM::CPU, camera_res);

          cv::cvtColor(right_frame_cv, right_frame_cv_bgr, cv::COLOR_BGRA2BGR);

          right_frame_msg = frame_to_msg(right_frame_cv_bgr);

          right_frame_msg->header.set__frame_id(link_namespace_ + "zedm_right_link");
          right_frame_msg->header.stamp.set__sec(
            static_cast<int32_t>(right_frame.timestamp.getSeconds()));
          right_frame_msg->header.stamp.set__nanosec(
            static_cast<uint32_t>(right_frame.timestamp.getNanoseconds() % uint64_t(1e9)));
          right_frame_msg->set__encoding(sensor_msgs::image_encodings::BGR8);

          right_info_.header.stamp.set__sec(static_cast<int32_t>(right_frame.timestamp.getSeconds()));
          right_info_.header.stamp.set__nanosec(
            static_cast<uint32_t>(right_frame.timestamp.getNanoseconds() % uint64_t(1e9)));

          // right_rect
          if (right_rect_pub_->getNumSubscribers()) {
            right_rect_pub_->publish(*right_frame_msg, right_info_);
          }

          // right_stream
          if (right_stream_pub_->getNumSubscribers() && stream_hd_) {
            right_stream_pub_->publish(*right_frame_msg);
          }
        }

        // SD streams
        if (right_rect_sd_pub_->getNumSubscribers() ||
          (right_stream_pub_->getNumSubscribers() && !stream_hd_))
        {
          zed_.retrieveImage(right_frame_sd, sl::VIEW::RIGHT, sl::MEM::CPU, sd_res);

          cv::cvtColor(right_frame_cv_sd, right_frame_cv_bgr_sd, cv::COLOR_BGRA2BGR);

          right_frame_msg_sd = frame_to_msg(right_frame_cv_bgr_sd);

          right_frame_msg_sd->header.set__frame_id(link_namespace_ + "zedm_right_link");
          right_frame_msg_sd->header.stamp.set__sec(
            static_cast<int32_t>(right_frame_sd.timestamp.getSeconds()));
          right_frame_msg_sd->header.stamp.set__nanosec(
            static_cast<uint32_t>(right_frame_sd.timestamp.getNanoseconds() % uint64_t(1e9)));
          right_frame_msg_sd->set__encoding(sensor_msgs::image_encodings::BGR8);

          right_sd_info_.header.stamp.set__sec(
            static_cast<int32_t>(right_frame_sd.timestamp.getSeconds()));
          right_sd_info_.header.stamp.set__nanosec(
            static_cast<uint32_t>(right_frame_sd.timestamp.getNanoseconds() % uint64_t(1e9)));

          // right_rect_sd
          if (right_rect_sd_pub_->getNumSubscribers()) {
            right_rect_sd_pub_->publish(*right_frame_msg_sd, right_sd_info_);
          }

          // right_stream
          if (right_stream_pub_->getNumSubscribers() && !stream_hd_) {
            right_stream_pub_->publish(*right_frame_msg_sd);
          }
        }
      }

      last_video_ts_ = curr_ts;
    }
  }

  // Join depth processing thread
  depth_thread_.join();
  RCLCPP_INFO(this->get_logger(), "Depth processing thread joined");

  // Close camera
  close_camera();
}

/**
 * @brief Processes positional tracking data.
 *
 * @param zed_pose ZED positional tracking data.
 *
 * @return Current camera pose in zedm_odom frame.
 */
PoseKit::Pose ZEDMiniDriverNode::positional_tracking(sl::Pose & camera_pose)
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

  return zed_pose;
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
 * @throws RuntimeError if clock_gettime fails.
 */
void ZEDMiniDriverNode::depth_routine()
{
  RCLCPP_INFO(this->get_logger(), "Depth processing thread started");

  while (running_.load(std::memory_order_acquire)) {
    struct timespec timeout;
    if (clock_gettime(CLOCK_REALTIME, &timeout) == -1) {
      RCLCPP_FATAL(
        this->get_logger(),
        "ZEDMiniDriver::depth_routine: clock_gettime failed");
      throw std::runtime_error(
              "ZEDMiniDriver::depth_routine: clock_gettime failed");
    }
    timeout.tv_sec += 1;

    if (sem_timedwait(&depth_sem_2_, &timeout) == 0) {
      // Publish depth map image
      cv::Mat depth_map_view_cv = slMat2cvMatDepth(depth_map_view_);
      Image::SharedPtr depth_msg = frame_to_msg(depth_map_view_cv);
      depth_msg->header.set__frame_id(link_namespace_ + "zedm_left_link");
      depth_msg->header.stamp.set__sec(static_cast<int32_t>(depth_map_view_.timestamp.getSeconds()));
      depth_msg->header.stamp.set__nanosec(
        static_cast<uint32_t>(depth_map_view_.timestamp.getNanoseconds() % uint64_t(1e9)));
      depth_msg->set__encoding(sensor_msgs::image_encodings::BGRA8);
      if (depth_pub_->getNumSubscribers()) {
        depth_pub_->publish(*depth_msg);
      }

      // Get latest map -> zedm_odom transform, then compute map -> zedm_link
      tf_lock_.lock();
      Eigen::Isometry3d map_to_camera_odom_iso = tf2::transformToEigen(map_to_camera_odom_);
      tf_lock_.unlock();
      Eigen::Isometry3d camera_odom_to_camera_iso = depth_curr_pose_.get_isometry();
      Eigen::Isometry3f map_to_camera_iso =
        Eigen::Isometry3d(map_to_camera_odom_iso * camera_odom_to_camera_iso).cast<float>();

      uint32_t pc_length = static_cast<uint32_t>(depth_point_cloud_.getWidth() * depth_point_cloud_.getHeight());

      // Fill and publish point cloud messages
      PointCloud2 pc_msg{};
      sensor_msgs::PointCloud2Modifier pc_modifier(pc_msg);
      pc_msg.header.set__frame_id("map");
      pc_msg.header.stamp.set__sec(static_cast<int32_t>(depth_point_cloud_.timestamp.getSeconds()));
      pc_msg.header.stamp.set__nanosec(
        static_cast<uint32_t>(depth_point_cloud_.timestamp.getNanoseconds() % uint64_t(1e9)));
      pc_msg.set__height(1);
      pc_msg.set__width(pc_length);
      pc_msg.set__is_bigendian(false);
      pc_msg.set__is_dense(true);
      pc_modifier.setPointCloud2Fields(
        4,
        "x", 1, PointField::FLOAT32,
        "y", 1, PointField::FLOAT32,
        "z", 1, PointField::FLOAT32,
        "rgba", 1, PointField::FLOAT32);
      sensor_msgs::PointCloud2Iterator<float> iter_pc_x(pc_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_pc_y(pc_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_pc_z(pc_msg, "z");
      sensor_msgs::PointCloud2Iterator<float> iter_pc_rgba(pc_msg, "rgba");

      // Fill point cloud matrix
      Eigen::MatrixXf pc_mat(4, pc_length);
      uint32_t mat_col_idx = 0;
      for (uint64_t i = 0; i < depth_point_cloud_.getHeight(); ++i) {
        for (uint64_t j = 0; j < depth_point_cloud_.getWidth(); ++j) {
          // Extract point position w.r.t. the camera from ZED data
          sl::float4 point3D;
          if (depth_point_cloud_.getValue(j, i, &point3D) == sl::ERROR_CODE::FAILURE) {
            pc_mat.block<4, 1>(0, mat_col_idx) = Eigen::Vector4f::Zero();
            continue;
          }
          pc_mat.block<4, 1>(0, mat_col_idx) = Eigen::Vector4f(
            point3D.x,
            point3D.y,
            point3D.z,
            1.0f);
          mat_col_idx++;

          // Write color information
          *iter_pc_rgba = point3D.w;
          ++iter_pc_rgba;
        }
      }

      // Express point cloud in map frame
      pc_mat = map_to_camera_iso.matrix() * pc_mat;

      // Fill point cloud messages
      for (uint32_t i = 0; i < pc_length; ++i) {
        *iter_pc_x = pc_mat(0, i);
        *iter_pc_y = pc_mat(1, i);
        *iter_pc_z = pc_mat(2, i);

        // Advance iterators
        ++iter_pc_x;
        ++iter_pc_y;
        ++iter_pc_z;
      }

      point_cloud_pub_->publish(pc_msg);
      rviz_point_cloud_pub_->publish(pc_msg);

      sem_post(&depth_sem_1_);
    }
  }
}

}   // namespace ZEDMiniDriver