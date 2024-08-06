/**
 * Implementation of the main worker thread of the ZED Driver node.
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
 * @brief Camera sampling routine: gets and publishes data from all sensors.
 *
 * @throws RuntimeError if the camera cannot be opened.
 */
void ZEDDriverNode::camera_routine()
{
  // Open camera
  if (!open_camera()) {
    throw std::runtime_error("ZEDDriverNode::camera_routine: Failed to open camera");
  }

  // Initialize semaphores
  if ((sem_init(&depth_sem_1_, 0, 1) != 0) ||
    (sem_init(&depth_sem_2_, 0, 0) != 0) ||
    (sem_init(&rgb_sem_1_, 0, 1) != 0) ||
    (sem_init(&rgb_sem_2_, 0, 0) != 0))
  {
    RCLCPP_FATAL(
      this->get_logger(),
      "ZEDDriverNode::camera_routine: Failed to initialize semaphores (%d)", errno);
    throw std::runtime_error(
            "ZEDDriverNode::camera_routine: Failed to initialize semaphores (" +
            std::to_string(errno) + ")");
  }

  // Prepare positional tracking data
  sl::Pose camera_pose;
  sl::POSITIONAL_TRACKING_STATE tracking_state;

  // Prepare RGB sampling data
  sl::Resolution camera_res = zed_.getCameraInformation().camera_configuration.resolution;
  sl::Resolution sd_res(video_sd_resolution_[0], video_sd_resolution_[1]);
  sl::Mat left_frame(camera_res, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
  sl::Mat right_frame(camera_res, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
  sl::Mat left_frame_sd(sd_res, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
  sl::Mat right_frame_sd(sd_res, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
  left_frame_cv_ = sl_to_cv(left_frame);
  right_frame_cv_ = sl_to_cv(right_frame);
  left_frame_sd_cv_ = sl_to_cv(left_frame_sd);
  right_frame_sd_cv_ = sl_to_cv(right_frame_sd);

  // Prepare depth processing data
  sl::Resolution depth_res(depth_resolution_[0], depth_resolution_[1]);

  // Prepare stopwatches
  rclcpp::Time curr_ts(0L, RCL_SYSTEM_TIME);
  rclcpp::Time last_depth_ts(0L, RCL_SYSTEM_TIME);
  rclcpp::Duration depth_period(
    std::chrono::nanoseconds(int(1.0 / double(depth_rate_ > 0 ? depth_rate_ : camera_fps_) * 1e9)));

  // Spawn depth processing thread
  depth_thread_ = std::thread(
    &ZEDDriverNode::depth_routine,
    this);

  // Spawn RGB processing thread
  rgb_thread_ = std::thread(
    &ZEDDriverNode::rgb_routine,
    this);

  // Spawn sensors processing thread (see zed_driver_sensors.cpp for more information)
  if (physical_camera_ && (sensors_sampling_time_ > -1)) {
    sensors_thread_ = std::thread(
      &ZEDDriverNode::sensors_routine,
      this);
  }

  RCLCPP_INFO(this->get_logger(), "Camera sampling thread started");

  // Run until stopped
  sl::ERROR_CODE err;
  sl::RuntimeParameters runtime_params;
  runtime_params.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
  runtime_params.enable_depth = true;
  runtime_params.enable_fill_mode = depth_fill_;
  runtime_params.remove_saturated_areas = true;
  while (running_.load(std::memory_order_acquire)) {
    // Grab data
    runtime_params.confidence_threshold = static_cast<int>(depth_confidence_);
    runtime_params.texture_confidence_threshold = static_cast<int>(depth_texture_confidence_);
    err = zed_.grab(runtime_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
      if (err == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
        // We reached the end of the recording, so just close the camera instance
        RCLCPP_WARN(this->get_logger(), "End of SVO file reached, closing camera instance");
        break;
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "ZEDDriverNode::camera_routine: Failed to grab data (%d): %s",
          static_cast<int>(err),
          sl::toString(err).c_str());
        continue;
      }
    }

    // Get RGB data and post it for processing (this goes on while we do stuff)
    sem_wait(&rgb_sem_1_);
    zed_.retrieveImage(left_frame, sl::VIEW::LEFT, sl::MEM::CPU, camera_res);
    zed_.retrieveImage(left_frame_sd, sl::VIEW::LEFT, sl::MEM::CPU, sd_res);
    zed_.retrieveImage(right_frame, sl::VIEW::RIGHT, sl::MEM::CPU, camera_res);
    zed_.retrieveImage(right_frame_sd, sl::VIEW::RIGHT, sl::MEM::CPU, sd_res);
    curr_rgb_ts_ = rclcpp::Time(left_frame.timestamp.getNanoseconds(), RCL_SYSTEM_TIME);
    curr_ts = curr_rgb_ts_;
    sem_post(&rgb_sem_2_);

    // Process and publish depth data iff:
    // - Depth is enabled at runtime (i.e. now)
    // - Depth mode is not NONE
    // - Depth rate is not negative, so the user did not disable depth data processing
    // - Undersampling is enabled and the period has elapsed, or it is disabled
    if (runtime_params.enable_depth &&
      (depth_mode_ != sl::DEPTH_MODE::NONE) &&
      (depth_rate_ >= 0) &&
      (((curr_ts - last_depth_ts) >= depth_period) ||
      (depth_rate_ == 0) ||
      (depth_rate_ >= camera_fps_)))
    {
      sem_wait(&depth_sem_1_);

      // Get depth data and post it for processing
      err = zed_.retrieveImage(depth_map_view_, sl::VIEW::DEPTH, sl::MEM::CPU, depth_res);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(
          this->get_logger(),
          "ZEDDriverNode::camera_routine: Failed to retrieve depth map: %s",
          sl::toString(err).c_str());
        sem_post(&depth_sem_1_);
        continue;
      }
      err = zed_.retrieveMeasure(depth_point_cloud_, sl::MEASURE::XYZBGRA, sl::MEM::CPU, depth_res);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(
          this->get_logger(),
          "ZEDDriverNode::camera_routine: Failed to retrieve point cloud: %s",
          sl::toString(err).c_str());
        sem_post(&depth_sem_1_);
        continue;
      }
      last_depth_ts = curr_ts;

      sem_post(&depth_sem_2_);
    }

    // Get positional tracking data
    if (tracking_enable_) {
      tracking_state = zed_.getPosition(camera_pose, sl::REFERENCE_FRAME::WORLD);
      if (verbose_) {
        switch (tracking_state) {
          case sl::POSITIONAL_TRACKING_STATE::OFF:
            RCLCPP_ERROR(this->get_logger(), "Positional tracking OFF");
            break;
          case sl::POSITIONAL_TRACKING_STATE::FPS_TOO_LOW:
            RCLCPP_ERROR(this->get_logger(), "FPS too low");
            break;
          case sl::POSITIONAL_TRACKING_STATE::SEARCHING_FLOOR_PLANE:
            RCLCPP_INFO(this->get_logger(), "Searching floor plane");
            break;
          case sl::POSITIONAL_TRACKING_STATE::UNAVAILABLE:
            RCLCPP_ERROR(this->get_logger(), "Positional tracking unavailable");
            break;
          default:
            break;
        }
      }

      // Publish positional tracking data
      if (camera_pose.valid) {
        positional_tracking(camera_pose);
      }
    }
  }

  // Join depth processing thread
  sem_post(&depth_sem_2_);
  depth_thread_.join();
  RCLCPP_INFO(this->get_logger(), "Depth processing thread joined");

  // Join RGB processing thread
  sem_post(&rgb_sem_2_);
  rgb_thread_.join();
  RCLCPP_INFO(this->get_logger(), "RGB processing thread joined");

  // Join sensors processing thread
  if (physical_camera_ && (sensors_sampling_time_ > -1)) {
    sensors_thread_.join();
    RCLCPP_INFO(this->get_logger(), "Sensors processing thread joined");
  }

  // Release memory of shared resources
  depth_map_view_.free(sl::MEM::CPU);
  depth_point_cloud_.free(sl::MEM::CPU);
  left_frame.free(sl::MEM::CPU);
  right_frame.free(sl::MEM::CPU);
  left_frame_sd.free(sl::MEM::CPU);
  right_frame_sd.free(sl::MEM::CPU);

  // Destroy semaphores
  sem_destroy(&depth_sem_1_);
  sem_destroy(&depth_sem_2_);
  sem_destroy(&rgb_sem_1_);
  sem_destroy(&rgb_sem_2_);

  // Close camera
  close_camera();
}

} // namespace zed_drivers
