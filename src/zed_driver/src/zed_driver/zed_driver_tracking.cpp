/**
 * Positional tracking routines.
 *
 * June 24, 2024
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <zed_driver/zed_driver.hpp>

namespace zed_drivers
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
  pose_header.set__frame_id(camera_local_frame_);
  pose_header.stamp.set__sec(
    static_cast<int32_t>(camera_pose.timestamp.getSeconds()));
  pose_header.stamp.set__nanosec(
    static_cast<uint32_t>(camera_pose.timestamp.getNanoseconds() % uint64_t(1e9)));
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

  // Here we must differentiate computations based on possible tracking settings.
  // Note that we have a different frame for the camera in the TF tree, and that frame might be
  // rotated with respect to the base frame, as per the static TF that describes this part of the robot.
  // The tracking_set_gravity_as_origin parameter enables the camera to estimate its orientation
  // with respect to the world, or with respect to its initial orientation.
  // To be coherent with the static TF, depending on the value of the parameter, we must either:
  // - leave the camera pose as it is (if tracking_set_gravity_as_origin is false);
  // - rotate the camera pose by the inverse of the (roll, pitch) rotation given by the static TF.
  // In the latter case, if there's a non-zero orientation left, it will be given by a real mounting
  // or positioning error.
  // As for the sensor data, the IMU orientation will still be expressed with respect to the world.
  if (tracking_set_gravity_as_origin_) {
    // Get (roll, pitch) rotation matrix of odom -> camera_odom transform
    TransformStamped odom_to_camera_odom{};
    rclcpp::Time tf_time(pose_header.stamp);
    while (true) {
      try {
        odom_to_camera_odom = tf_buffer_->lookupTransform(
          local_frame_,
          camera_local_frame_,
          tf_time,
          tf2::durationFromSec(0.1));
        break;
      } catch (const tf2::ExtrapolationException & e) {
        // Just get the latest
        tf_time = rclcpp::Time{};
      } catch (const tf2::TransformException & e) {
        RCLCPP_ERROR(
          this->get_logger(),
          "ZEDDriverNode::positional_tracking: TF exception: %s",
          e.what());
        return;
      }
    }
    tf2Scalar camera_roll, camera_pitch, camera_yaw;
    tf2::Matrix3x3 odom_to_camera_odom_mat(
      tf2::Quaternion{
        odom_to_camera_odom.transform.rotation.x,
        odom_to_camera_odom.transform.rotation.y,
        odom_to_camera_odom.transform.rotation.z,
        odom_to_camera_odom.transform.rotation.w});
    odom_to_camera_odom_mat.getEulerYPR(camera_yaw, camera_pitch, camera_roll);
    Eigen::Matrix3d camera_odom_rp(
      Eigen::AngleAxisd(camera_roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(camera_pitch, Eigen::Vector3d::UnitY()));
    Eigen::Matrix<double, 6, 6> camera_odom_rp_covariance = Eigen::Matrix<double, 6, 6>::Zero();
    camera_odom_rp_covariance.block<3, 3>(0, 0) = camera_odom_rp;
    camera_odom_rp_covariance.block<3, 3>(3, 3) = camera_odom_rp;
    std::array<double, 36> pose_covariance_in = pose_covariance;
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> pose_covariance_map(
      pose_covariance.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> pose_covariance_in_map(
      pose_covariance_in.data());

    // Build corrected camera pose
    position = camera_odom_rp.inverse() * position;
    orientation = camera_odom_rp.inverse() * orientation;
    pose_covariance_map =
      camera_odom_rp_covariance.transpose() * pose_covariance_in_map * camera_odom_rp_covariance;
  }

  // Apply corrections to VIO position data
  position.x() *= x_correction_;
  position.y() *= y_correction_;

  pose_kit::Pose zed_pose(
    position,
    orientation,
    pose_header,
    pose_covariance);

  // Parse twist data (it's in body frame a.k.a. left camera frame, so no need for corrections)
  Header twist_header{};
  twist_header.set__frame_id(camera_frame_);
  twist_header.set__stamp(pose_header.stamp);
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
  pose_kit::KinematicPose zed_twist(
    Eigen::Vector3d::Zero(),
    Eigen::Quaterniond::Identity(),
    linear_velocity,
    angular_velocity,
    twist_header,
    std::array<double, 36>{},
    twist_covariance);

  // Get latest base_link -> camera transform
  TransformStamped base_link_to_camera{};
  rclcpp::Time tf_time(pose_header.stamp);
  while (true) {
    try {
      base_link_to_camera = tf_buffer_->lookupTransform(
        frame_prefix_ + base_link_name_,
        camera_frame_,
        tf_time,
        tf2::durationFromSec(0.1));
      break;
    } catch (const tf2::ExtrapolationException & e) {
      // Just get the latest
      tf_time = rclcpp::Time{};
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "ZEDDriverNode::positional_tracking: TF exception: %s",
        e.what());
      return;
    }
  }

  // Get base_link pose
  pose_kit::Pose base_link_pose = zed_pose;
  base_link_pose.rigid_transform(base_link_to_camera, local_frame_);

  // Get base_link twist
  pose_kit::KinematicPose base_link_twist = zed_twist;
  base_link_twist.rigid_transform(base_link_to_camera);

  // Build odometry messages
  Odometry camera_odom_msg{}, base_link_odom_msg{};
  camera_odom_msg.set__header(zed_pose.get_header());
  camera_odom_msg.set__child_frame_id(camera_frame_);
  camera_odom_msg.set__pose(zed_pose.to_pose_with_covariance_stamped().pose);
  camera_odom_msg.set__twist(zed_twist.to_twist_with_covariance_stamped().twist);
  base_link_odom_msg.set__header(base_link_pose.get_header());
  base_link_odom_msg.set__child_frame_id(frame_prefix_ + base_link_name_);
  base_link_odom_msg.set__pose(base_link_pose.to_pose_with_covariance_stamped().pose);
  base_link_odom_msg.set__twist(base_link_twist.to_twist_with_covariance_stamped().twist);

  // Publish odometry messages
  base_link_odom_pub_->publish(base_link_odom_msg);
  camera_odom_pub_->publish(camera_odom_msg);

  // Publish pose messages
  base_link_pose_pub_->publish(base_link_pose.to_pose_with_covariance_stamped());
  camera_pose_pub_->publish(zed_pose.to_pose_with_covariance_stamped());

  // Publish local_frame -> base_link transform
  if (publish_tf_) {
    TransformStamped local_frame_to_base_link = tf2::eigenToTransform(base_link_pose.get_isometry());
    local_frame_to_base_link.set__header(base_link_pose.get_header());
    local_frame_to_base_link.set__child_frame_id(frame_prefix_ + base_link_name_);
    tf_broadcaster_->sendTransform(local_frame_to_base_link);
  }
}

} // namespace zed_drivers
