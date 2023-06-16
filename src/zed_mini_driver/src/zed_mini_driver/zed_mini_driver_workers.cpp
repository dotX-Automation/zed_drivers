/**
 * Worker threads of the ZED Mini driver node.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 13, 2023
 */

#define NOOP ((void)0)

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

  // Run until stopped
  sl::ERROR_CODE err;
  sl::Pose camera_pose;
  sl::POSITIONAL_TRACKING_STATE tracking_state;
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
      twist_header.set__frame_id(link_namespace_ + "zedm_left_link");
      twist_header.stamp.set__sec(
        static_cast<int32_t>(camera_pose.timestamp.getNanoseconds() /
        uint64_t(1e9)));
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
      camera_odom_msg.set__child_frame_id(link_namespace_ + "zedm_left_link");
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
      rviz_base_link_pose_pub_->publish(base_link_pose.to_pose_stamped());
      rviz_camera_pose_pub_->publish(zed_pose.to_pose_stamped());
    }
  }

  // Close camera
  close_camera();
}

/**
 * @brief Listens for incoming TFs.
 */
void ZEDMiniDriverNode::tf_thread_routine()
{
  // Initialize TF buffers and listeners
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  std::string map_frame = "map";
  std::string odom_frame = link_namespace_ + "odom";
  std::string zedm_odom_frame = link_namespace_ + "zedm_odom";
  odom_to_camera_odom_.header.set__frame_id(odom_frame);
  odom_to_camera_odom_.set__child_frame_id(zedm_odom_frame);
  map_to_camera_odom_.header.set__frame_id(map_frame);
  map_to_camera_odom_.set__child_frame_id(zedm_odom_frame);
  base_link_to_camera_.header.set__frame_id(link_namespace_ + "base_link");
  base_link_to_camera_.set__child_frame_id(link_namespace_ + "zedm_left_link");
  TransformStamped odom_to_camera_odom{}, map_to_camera_odom{};

  // Start listening
  while (tf_listening_.load(std::memory_order_acquire)) {
    // odom -> zedm_odom, base_link -> zedm_left_link (it's rigid)
    try {
      odom_to_camera_odom = tf_buffer_->lookupTransform(
        odom_frame,
        zedm_odom_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(1.0));

      tf_lock_.lock();
      odom_to_camera_odom_ = odom_to_camera_odom;
      base_link_to_camera_ = odom_to_camera_odom;
      base_link_to_camera_.header.set__frame_id(link_namespace_ + "base_link");
      base_link_to_camera_.set__child_frame_id(link_namespace_ + "zedm_left_link");
      tf_lock_.unlock();
    } catch (const tf2::TimeoutException & e) {
      NOOP;
    } catch (const tf2::TransformException & e) {
      RCLCPP_INFO(this->get_logger(), "TF exception: %s", e.what());
    }

    // map -> zedm_odom
    try {
      map_to_camera_odom = tf_buffer_->lookupTransform(
        map_frame,
        zedm_odom_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(1.0));

      tf_lock_.lock();
      map_to_camera_odom_ = map_to_camera_odom;
      tf_lock_.unlock();
    } catch (const tf2::TimeoutException & e) {
      NOOP;
    } catch (const tf2::TransformException & e) {
      RCLCPP_INFO(this->get_logger(), "TF exception: %s", e.what());
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // Stop listening
  tf_listener_.reset();
  tf_buffer_.reset();
}

} // namespace ZEDMiniDriver
