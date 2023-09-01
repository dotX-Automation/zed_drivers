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
 * @brief Publishes onboard sensors data.
 *
 * @param sensors_data Sensors data.
 */
void ZEDDriverNode::sensor_sampling(sl::SensorsData & sensors_data)
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

    imu_pub_->publish(imu_msg);
    imu_filtered_pub_->publish(imu_filtered_msg);
  }
}

} // namespace ZEDDriver
