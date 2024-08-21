/**
 * Depth data processing routines.
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
 * @brief Processes depth data.
 */
void ZEDDriverNode::depth_routine()
{
  RCLCPP_INFO(this->get_logger(), "Depth processing thread started");

  while (true) {
    // Wait for depth data
    sem_wait(&depth_sem_2_);

    // Check if the thread should stop
    if (!running_.load(std::memory_order_acquire)) {
      break;
    }

    // Publish depth map image
    cv::Mat depth_map_view_cv = sl_to_cv_depth(depth_map_view_);
    Image::SharedPtr depth_msg = frame_to_msg(depth_map_view_cv);
    depth_msg->header.set__frame_id(camera_frame_);
    depth_msg->header.stamp.set__sec(static_cast<int32_t>(depth_map_view_.timestamp.getSeconds()));
    depth_msg->header.stamp.set__nanosec(
      static_cast<uint32_t>(depth_map_view_.timestamp.getNanoseconds() % uint64_t(1e9)));
    depth_msg->set__encoding(sensor_msgs::image_encodings::BGRA8);
    depth_pub_->publish(*depth_msg);

    // Prepare point cloud message
    PointCloud2 pc_msg{};
    sensor_msgs::PointCloud2Modifier pc_modifier(pc_msg);
    uint32_t pc_length =
      static_cast<uint32_t>(depth_point_cloud_.getWidth() * depth_point_cloud_.getHeight());
    pc_msg.header.set__frame_id(camera_frame_);
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

    // Prepare depth distances message and buffer
    Image depth_distances_msg{};
    depth_distances_msg.header.set__frame_id(camera_frame_);
    depth_distances_msg.header.stamp.set__sec(
      static_cast<int32_t>(depth_point_cloud_.timestamp.getSeconds()));
    depth_distances_msg.header.stamp.set__nanosec(
      static_cast<uint32_t>(depth_point_cloud_.timestamp.getNanoseconds() % uint64_t(1e9)));
    depth_distances_msg.set__width(depth_point_cloud_.getWidth());
    depth_distances_msg.set__height(depth_point_cloud_.getHeight());
    depth_distances_msg.set__encoding(sensor_msgs::image_encodings::TYPE_32FC1);
    depth_distances_msg.set__is_bigendian(false);
    depth_distances_msg.set__step(depth_point_cloud_.getWidth() * sizeof(float));
    cv::Mat depth_distances_cv = cv::Mat::zeros(
      depth_point_cloud_.getHeight(),
      depth_point_cloud_.getWidth(),
      CV_32FC1);
    cv::Mat nan_mask = cv::Mat::zeros(depth_distances_cv.size(), CV_8U);

    // Fill and publish point cloud and depth distances messages
    uint32_t valid_points = 0U;
    float nan = std::numeric_limits<float>::quiet_NaN();
    for (uint64_t i = 0; i < depth_point_cloud_.getHeight(); ++i) {
      for (uint64_t j = 0; j < depth_point_cloud_.getWidth(); ++j) {
        // Extract point position w.r.t. the camera from ZED data
        sl::float4 point3D;
        if ((depth_point_cloud_.getValue(j, i, &point3D) == sl::ERROR_CODE::FAILURE) ||
          ((!isValidMeasure(point3D.x) || !isValidMeasure(point3D.y) || !isValidMeasure(point3D.z))))
        {
          depth_distances_cv.at<float>(i, j) = nan;
          nan_mask.at<unsigned char>(i, j) = 255;
          continue;
        }
        valid_points++;

        // Fill point cloud message
        *iter_pc_x = point3D.x;
        *iter_pc_y = point3D.y;
        *iter_pc_z = point3D.z;
        *iter_pc_rgba = point3D.w;

        // Fill depth distances matrix
        Eigen::Vector3f point(float(point3D.x), float(point3D.y), float(point3D.z));
        float distance = point.norm();
        depth_distances_cv.at<float>(i, j) = distance;

        // Advance iterators
        ++iter_pc_x;
        ++iter_pc_y;
        ++iter_pc_z;
        ++iter_pc_rgba;
      }
    }

    if (valid_points > 0U) {
      // Apply inpainting to fill holes in the distances matrix
      cv::Mat depth_distances_inpaint;
      cv::inpaint(
        depth_distances_cv,
        nan_mask,
        depth_distances_inpaint,
        depth_distances_inpainting_radius_,
        cv::INPAINT_TELEA);

      // Fill depth distances message
      depth_distances_msg.set__step(
        depth_distances_inpaint.cols * depth_distances_inpaint.elemSize());
      size_t size = depth_distances_msg.step * depth_distances_inpaint.rows;
      depth_distances_msg.data.resize(size);
      ::memcpy(depth_distances_msg.data.data(), depth_distances_inpaint.data, size);

      pc_modifier.resize(valid_points);
      point_cloud_pub_->publish(pc_msg);
      depth_distances_pub_->publish(depth_distances_msg);
    }

    sem_post(&depth_sem_1_);
  }
}

} // namespace zed_drivers
