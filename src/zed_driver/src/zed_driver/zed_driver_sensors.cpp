/**
 * Sensors data processing routines.
 *
 * Roberto Masocco <r.masocco@dotxautomation.com>
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
    Imu imu_msg{};
    imu_msg.header.stamp.set__sec(static_cast<int32_t>(imu_data.timestamp.getSeconds()));
    imu_msg.header.stamp.set__nanosec(
      static_cast<uint32_t>(imu_data.timestamp.getNanoseconds() % uint64_t(1e9)));
    imu_msg.header.set__frame_id(camera_imu_frame_);

    imu_msg.orientation.set__w(static_cast<double>(imu_data.pose.getOrientation().ow));
    imu_msg.orientation.set__x(static_cast<double>(imu_data.pose.getOrientation().ox));
    imu_msg.orientation.set__y(static_cast<double>(imu_data.pose.getOrientation().oy));
    imu_msg.orientation.set__z(static_cast<double>(imu_data.pose.getOrientation().oz));

    for (int i = 0; i < 9; ++i) {
      imu_msg.orientation_covariance[i] = static_cast<double>(imu_data.pose_covariance.r[i]);
    }

    imu_msg.angular_velocity.set__x(
      static_cast<double>((M_PIf32 / 180.0f) * imu_data.angular_velocity.x));
    imu_msg.angular_velocity.set__y(
      static_cast<double>((M_PIf32 / 180.0f) * imu_data.angular_velocity.y));
    imu_msg.angular_velocity.set__z(
      static_cast<double>((M_PIf32 / 180.0f) * imu_data.angular_velocity.z));

    for (int i = 0; i < 9; ++i) {
      imu_msg.angular_velocity_covariance[i] =
        static_cast<double>((M_PIf32 / 180.0f) * imu_data.angular_velocity_covariance.r[i]);
    }

    imu_msg.linear_acceleration.set__x(static_cast<double>(imu_data.linear_acceleration.x));
    imu_msg.linear_acceleration.set__y(static_cast<double>(imu_data.linear_acceleration.y));
    imu_msg.linear_acceleration.set__z(static_cast<double>(imu_data.linear_acceleration.z));

    for (int i = 0; i < 9; ++i) {
      imu_msg.linear_acceleration_covariance[i] =
        static_cast<double>(imu_data.linear_acceleration_covariance.r[i]);
    }

    // Publish IMU data
    imu_pub_->publish(imu_msg);
  }
}

} // namespace zed_drivers
