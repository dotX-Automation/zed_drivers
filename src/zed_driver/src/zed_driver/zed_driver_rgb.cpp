/**
 * RGB data processing routines.
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
 * @brief Publishes RGB data.
 */
void ZEDDriverNode::rgb_routine()
{
  rclcpp::Duration video_period(
    std::chrono::nanoseconds(
      int(1.0 /
      double(video_stream_rate_ > 0 ? video_stream_rate_ : camera_fps_) * 1e9)));
  rclcpp::Time last_video_ts(0L, RCL_SYSTEM_TIME);

  RCLCPP_INFO(this->get_logger(), "RGB processing thread started");

  while (true) {
    // Wait for RGB data
    sem_wait(&rgb_sem_2_);

    // Check if the thread should stop
    if (!running_.load(std::memory_order_acquire)) {
      break;
    }

    // This is the sequence, for each stream:
    // - Retrieve frames (the camera sampling thread already did this for us)
    // - Convert frames to BGR8 in OpenCV (we discard the Alpha channel to save bandwidth)
    // - Allocate Image messages
    // - Set Image messages headers and metadata
    // - Stamp camera_infos
    // - Publish images and camera_infos
    // - Record frames, if requested

    // Prepare local data
    cv::Mat left_frame_cv_bgr(left_frame_cv_.size(), CV_8UC3);
    cv::Mat right_frame_cv_bgr(right_frame_cv_.size(), CV_8UC3);
    cv::Mat left_frame_sd_cv_bgr(left_frame_sd_cv_.size(), CV_8UC3);
    cv::Mat right_frame_sd_cv_bgr(right_frame_sd_cv_.size(), CV_8UC3);
    Image::SharedPtr left_frame_msg, right_frame_msg;
    Image::SharedPtr left_frame_msg_sd, right_frame_msg_sd;

    // Convert frames to BGR8 in OpenCV
    cv::cvtColor(left_frame_cv_, left_frame_cv_bgr, cv::COLOR_BGRA2BGR);
    cv::cvtColor(right_frame_cv_, right_frame_cv_bgr, cv::COLOR_BGRA2BGR);
    cv::cvtColor(left_frame_sd_cv_, left_frame_sd_cv_bgr, cv::COLOR_BGRA2BGR);
    cv::cvtColor(right_frame_sd_cv_, right_frame_sd_cv_bgr, cv::COLOR_BGRA2BGR);

    // Allocate Image messages
    left_frame_msg = frame_to_msg(left_frame_cv_bgr);
    right_frame_msg = frame_to_msg(right_frame_cv_bgr);
    left_frame_msg_sd = frame_to_msg(left_frame_sd_cv_bgr);
    right_frame_msg_sd = frame_to_msg(right_frame_sd_cv_bgr);

    // Set Image messages headers and metadata (left)
    left_frame_msg->header.set__frame_id(camera_left_frame_);
    left_frame_msg->header.set__stamp(curr_rgb_ts_);
    left_frame_msg->set__encoding(sensor_msgs::image_encodings::BGR8);

    // Set Image message headers and metadata (right)
    right_frame_msg->header.set__frame_id(camera_right_frame_);
    right_frame_msg->header.set__stamp(curr_rgb_ts_);
    right_frame_msg->set__encoding(sensor_msgs::image_encodings::BGR8);

    // Set Image messages headers and metadata (left, SD)
    left_frame_msg_sd->header.set__frame_id(camera_left_frame_);
    left_frame_msg_sd->header.set__stamp(curr_rgb_ts_);
    left_frame_msg_sd->set__encoding(sensor_msgs::image_encodings::BGR8);

    // Set Image message headers and metadata (right, SD)
    right_frame_msg_sd->header.set__frame_id(camera_right_frame_);
    right_frame_msg_sd->header.set__stamp(curr_rgb_ts_);
    right_frame_msg_sd->set__encoding(sensor_msgs::image_encodings::BGR8);

    // Stamp camera_infos
    left_info_.header.set__stamp(curr_rgb_ts_);
    right_info_.header.set__stamp(curr_rgb_ts_);
    left_sd_info_.header.set__stamp(curr_rgb_ts_);
    right_sd_info_.header.set__stamp(curr_rgb_ts_);

    // Publish images and camera_infos (left, right, HD streams)
    left_info_pub_->publish(left_info_);
    left_rect_pub_->publish(*left_frame_msg);
    right_info_pub_->publish(right_info_);
    right_rect_pub_->publish(*right_frame_msg);
    if (video_stream_hd_) {
      left_stream_pub_->publish(*left_frame_msg);
      right_stream_pub_->publish(*right_frame_msg);
    }

    // Publish images and camera_infos (left, right, SD streams)
    if (((curr_rgb_ts_ - last_video_ts) >= video_period) ||
      (video_stream_rate_ == 0) ||
      (video_stream_rate_ >= camera_fps_))
    {
      left_sd_info_pub_->publish(left_sd_info_);
      left_rect_sd_pub_->publish(*left_frame_msg_sd);
      right_sd_info_pub_->publish(right_sd_info_);
      right_rect_sd_pub_->publish(*right_frame_msg_sd);
      if (!video_stream_hd_) {
        left_stream_pub_->publish(*left_frame_msg_sd);
        right_stream_pub_->publish(*right_frame_msg_sd);
      }

      last_video_ts = curr_rgb_ts_;
    }

    // Record frames, if requested
    if (!video_stream_record_path_.empty()) {
      std::string frame_name = std::to_string(uint64_t(curr_rgb_ts_.nanoseconds())) + ".jpg";
      std::string left_frame_name = video_stream_record_path_ + "/left_" + frame_name;
      std::string right_frame_name = video_stream_record_path_ + "/right_" + frame_name;
      cv::imwrite(left_frame_name, left_frame_cv_bgr);
      cv::imwrite(right_frame_name, right_frame_cv_bgr);
    }
    sem_post(&rgb_sem_1_);
  }
}

} // namespace zed_drivers
