/**
 * Positional tracking routines.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * August 29, 2023
 */

#include <zed_driver/zed_driver.hpp>

namespace ZEDDriver
{

/**
 * @brief Processes positional tracking data.
 *
 * @param camera_pose ZED positional tracking data.
 */
void ZEDDriverNode::positional_tracking(sl::Pose & camera_pose)
{
  // Parse pose data
  Header pose_header{};
  pose_header.set__frame_id(camera_odom_frame_);
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
  twist_header.set__frame_id(camera_frame_);
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

  // Get latest base_link -> camera transform
  TransformStamped base_link_to_camera{};
  try {
    base_link_to_camera = tf_buffer_->lookupTransform(
      link_namespace_ + "base_link",
      camera_frame_,
      zed_pose.get_header().stamp,
      tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "ZEDDriverNode::positional_tracking: TF exception: %s",
      e.what());
  }

  // Get base_link pose
  PoseKit::Pose base_link_pose = zed_pose;
  base_link_pose.rigid_transform(base_link_to_camera, odom_frame_);

  // Get base_link twist
  PoseKit::KinematicPose base_link_twist = zed_twist;
  base_link_twist.rigid_transform(base_link_to_camera);

  // Build odometry messages
  Odometry camera_odom_msg{}, base_link_odom_msg{};
  camera_odom_msg.set__header(zed_pose.get_header());
  camera_odom_msg.set__child_frame_id(camera_frame_);
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

  // Publish odom -> base_link transform
  if (publish_tf_) {
    TransformStamped odom_to_base_link = tf2::eigenToTransform(base_link_pose.get_isometry());
    odom_to_base_link.set__header(base_link_pose.get_header());
    odom_to_base_link.set__child_frame_id(link_namespace_ + "base_link");
    tf_broadcaster_->sendTransform(odom_to_base_link);
  }
}

} // namespace ZEDDriver
