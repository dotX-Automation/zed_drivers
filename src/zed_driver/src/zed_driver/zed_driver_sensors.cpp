/**
 * Sensors data processing routines.
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
 * @brief Thread to sample high-rate onboard sensors.
 */
void ZEDDriverNode::sensors_routine()
{
  // Prepare sensors data
  sl::SensorsData sensor_data;

  RCLCPP_INFO(this->get_logger(), "Sensors processing thread started");

  // Run until stopped
  sl::ERROR_CODE err;
  sl::Timestamp last_imu_ts_{};
  sensors_start_time_ = this->get_clock()->now();
  while (true) {
    // Check if thread should stop
    if (!running_.load(std::memory_order_acquire)) {
      break;
    }

    // The rationale for this is that we want to sample sensors data at the highest possible rate, but:
    // - we must check that the data we get from the camera is not stale, i.e., the timestamp is different;
    // - we can do this only if a physical camera is connected, i.e., this is not a stream or an SVO file.
    // In the latter case, this thread won't be activated: only a small subset of the data will be
    // available at TIME_REFERENCE::IMAGE, so it's useless to sample it.
    err = zed_.getSensorsData(sensor_data, sl::TIME_REFERENCE::CURRENT);

    if (err == sl::ERROR_CODE::SUCCESS) {
      // Check that the data is not stale
      if (sensor_data.imu.timestamp == last_imu_ts_) {
        continue;
      }
      last_imu_ts_ = sensor_data.imu.timestamp;
      sensors_processing(sensor_data);
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "ZEDDriverNode::sensors_routine: Failed to get sensors data (%d): %s",
        static_cast<int>(err),
        sl::toString(err).c_str());
      continue;
    }

    // Enforce a specific sampling rate
    if (imu_sampling_time_ > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(imu_sampling_time_));
    }
  }
}

/**
 * @brief Post-processes and publishes onboard sensors data.
 *
 * @param sensors_data Sensors data.
 */
void ZEDDriverNode::sensors_processing(sl::SensorsData & sensors_data)
{
  // Process IMU data, translating it from ZED to ROS format
  // Refer to the ZED SDK documentation for more information on these operations
  sl::SensorsData::IMUData imu_data = sensors_data.imu;
  if (imu_data.is_available) {
    Imu imu_msg{}, imu_filtered_msg{};
    imu_msg.header.stamp.set__sec(static_cast<int32_t>(imu_data.timestamp.getSeconds()));
    imu_msg.header.stamp.set__nanosec(
      static_cast<uint32_t>(imu_data.timestamp.getNanoseconds() % uint64_t(1e9)));
    imu_msg.header.set__frame_id(camera_imu_frame_);
    imu_filtered_msg.header.set__stamp(imu_msg.header.stamp);
    imu_filtered_msg.header.set__frame_id(camera_imu_frame_);

    imu_msg.orientation.set__w(static_cast<double>(imu_data.pose.getOrientation().ow));
    imu_msg.orientation.set__x(static_cast<double>(imu_data.pose.getOrientation().ox));
    imu_msg.orientation.set__y(static_cast<double>(imu_data.pose.getOrientation().oy));
    imu_msg.orientation.set__z(static_cast<double>(imu_data.pose.getOrientation().oz));
    imu_filtered_msg.orientation.set__w(static_cast<double>(imu_data.pose.getOrientation().ow));
    imu_filtered_msg.orientation.set__x(static_cast<double>(imu_data.pose.getOrientation().ox));
    imu_filtered_msg.orientation.set__y(static_cast<double>(imu_data.pose.getOrientation().oy));
    imu_filtered_msg.orientation.set__z(static_cast<double>(imu_data.pose.getOrientation().oz));

    for (int i = 0; i < 9; ++i) {
      imu_msg.orientation_covariance[i] = static_cast<double>(imu_data.pose_covariance.r[i]);
      imu_filtered_msg.orientation_covariance[i] =
        static_cast<double>(imu_data.pose_covariance.r[i]);
    }

    imu_msg.angular_velocity.set__x(
      static_cast<double>((M_PIf32 / 180.0f) * imu_data.angular_velocity.x));
    imu_msg.angular_velocity.set__y(
      static_cast<double>((M_PIf32 / 180.0f) * imu_data.angular_velocity.y));
    imu_msg.angular_velocity.set__z(
      static_cast<double>((M_PIf32 / 180.0f) * imu_data.angular_velocity.z));
    imu_filtered_msg.angular_velocity.set__x(
      gyro_filters_[0].evolve(
        static_cast<double>((M_PIf32 / 180.0f) * imu_data.angular_velocity.x))(0, 0));
    imu_filtered_msg.angular_velocity.set__y(
      gyro_filters_[1].evolve(
        static_cast<double>((M_PIf32 / 180.0f) * imu_data.angular_velocity.y))(0, 0));
    imu_filtered_msg.angular_velocity.set__z(
      gyro_filters_[2].evolve(
        static_cast<double>((M_PIf32 / 180.0f) * imu_data.angular_velocity.z))(0, 0));

    for (int i = 0; i < 9; ++i) {
      imu_msg.angular_velocity_covariance[i] =
        static_cast<double>((M_PIf32 / 180.0f) * imu_data.angular_velocity_covariance.r[i]);
      imu_filtered_msg.angular_velocity_covariance[i] =
        static_cast<double>((M_PIf32 / 180.0f) * imu_data.angular_velocity_covariance.r[i]);
    }

    imu_msg.linear_acceleration.set__x(static_cast<double>(imu_data.linear_acceleration.x));
    imu_msg.linear_acceleration.set__y(static_cast<double>(imu_data.linear_acceleration.y));
    imu_msg.linear_acceleration.set__z(static_cast<double>(imu_data.linear_acceleration.z));
    imu_filtered_msg.linear_acceleration.set__x(
      accel_filters_[0].evolve(static_cast<double>(imu_data.linear_acceleration.x))(0, 0));
    imu_filtered_msg.linear_acceleration.set__y(
      accel_filters_[1].evolve(static_cast<double>(imu_data.linear_acceleration.y))(0, 0));
    imu_filtered_msg.linear_acceleration.set__z(
      accel_filters_[2].evolve(static_cast<double>(imu_data.linear_acceleration.z))(0, 0));

    for (int i = 0; i < 9; ++i) {
      imu_msg.linear_acceleration_covariance[i] =
        static_cast<double>(imu_data.linear_acceleration_covariance.r[i]);
      imu_filtered_msg.linear_acceleration_covariance[i] =
        static_cast<double>(imu_data.linear_acceleration_covariance.r[i]);
    }

    // Publish IMU data
    imu_pub_->publish(imu_msg);

    // Publish filtered IMU data if the settling time has elapsed
    if ((imu_filters_settling_time_ > 0.0) && !imu_filters_settling_time_elapsed_) {
      rclcpp::Duration elapsed = this->get_clock()->now() - sensors_start_time_;
      if (elapsed.seconds() * 1e9 + elapsed.nanoseconds() > imu_filters_settling_time_ * 1e9) {
        imu_filters_settling_time_elapsed_ = true;
        RCLCPP_INFO(this->get_logger(), "IMU filters settling time elapsed");
      }
    }
    if (!(imu_filters_settling_time_ > 0.0) || imu_filters_settling_time_elapsed_) {
      imu_filtered_pub_->publish(imu_filtered_msg);
    }
  }
}

} // namespace ZEDDriver
