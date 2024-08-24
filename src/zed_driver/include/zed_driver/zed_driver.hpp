/**
 * ZED Driver node definition.
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

#ifndef ZED_DRIVERS__ZED_DRIVER_HPP
#define ZED_DRIVERS__ZED_DRIVER_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <errno.h>
#include <semaphore.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <sl/Camera.hpp>

#include <rclcpp/rclcpp.hpp>

#include <pose_kit/pose.hpp>
#include <pose_kit/kinematic_pose.hpp>

#include <dua_node/dua_node.hpp>
#include <dua_qos_cpp/dua_qos.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <theora_wrappers/publisher.hpp>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <std_srvs/srv/set_bool.hpp>

using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;

using namespace std_srvs::srv;

namespace zed_drivers
{

/**
 * Drives a ZED camera.
 */
class ZEDDriverNode : public dua_node::NodeBase
{
public:
  ZEDDriverNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());
  virtual ~ZEDDriverNode();

private:
  /* Node initialization routines. */
  void init_parameters();
  void init_publishers();
  void init_services();
  void init_tf2();

  /* TF listeners, broadcaster, and related data. */
  std::string camera_name_;
  std::string camera_frame_;
  std::string camera_local_frame_;
  std::string camera_left_frame_;
  std::string camera_right_frame_;
  std::string camera_imu_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /* Topic publishers. */
  rclcpp::Publisher<Odometry>::SharedPtr base_link_odom_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr base_link_pose_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr camera_odom_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr camera_pose_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr depth_map_pub_;

  /* image_transport, camera_info publishers. */
  std::shared_ptr<image_transport::Publisher> left_rect_pub_;
  std::shared_ptr<image_transport::Publisher> right_rect_pub_;
  std::shared_ptr<image_transport::Publisher> left_rect_sd_pub_;
  std::shared_ptr<image_transport::Publisher> right_rect_sd_pub_;
  std::shared_ptr<TheoraWrappers::Publisher> left_stream_pub_;
  std::shared_ptr<TheoraWrappers::Publisher> right_stream_pub_;
  std::shared_ptr<image_transport::Publisher> depth_pub_;
  std::shared_ptr<image_transport::Publisher> depth_distances_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr right_info_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr left_sd_info_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr right_sd_info_pub_;

  /* Service servers. */
  rclcpp::Service<SetBool>::SharedPtr enable_srv_;

  /* Service callbacks. */
  void enable_callback(
    const SetBool::Request::SharedPtr req,
    const SetBool::Response::SharedPtr resp);

  /* Internal state and data. */
  sl::Camera zed_;
  sl::MODEL camera_model_;
  std::atomic<bool> running_{false};
  camera_info_manager::CameraInfo left_info_{};
  camera_info_manager::CameraInfo left_sd_info_{};
  camera_info_manager::CameraInfo right_info_{};
  camera_info_manager::CameraInfo right_sd_info_{};
  bool physical_camera_ = false;

  /* Camera sampling thread and routine. */
  std::thread camera_thread_;
  void camera_routine();

  /* Depth processing thread, routine, and buffers. */
  std::thread depth_thread_;
  void depth_routine();
  sem_t depth_sem_1_;
  sem_t depth_sem_2_;
  sl::Mat depth_map_view_;
  sl::Mat depth_point_cloud_;

  /* RGB processing thread, routine, and buffers. */
  std::thread rgb_thread_;
  void rgb_routine();
  rclcpp::Time curr_rgb_ts_{};
  sem_t rgb_sem_1_;
  sem_t rgb_sem_2_;
  cv::Mat left_frame_cv_, right_frame_cv_;
  cv::Mat left_frame_sd_cv_, right_frame_sd_cv_;

  /* Sensors processing thread and routine. */
  std::thread sensors_thread_;
  rclcpp::Time sensors_start_time_{};
  bool imu_filters_settling_time_elapsed_ = false;
  void sensors_routine();

  /* Node parameters. */
  bool autostart_;
  std::string base_link_name_ = "";
  int camera_fps_ = 15;
  sl::RESOLUTION camera_resolution_ = sl::RESOLUTION::HD720;
  int64_t depth_confidence_ = 50;
  bool depth_fill_ = false;
  bool depth_map_filled_ = false;
  sl::DEPTH_MODE depth_mode_ = sl::DEPTH_MODE::QUALITY;
  int64_t depth_rate_ = 0;
  std::vector<int64_t> depth_resolution_ = {0, 0};
  int64_t depth_texture_confidence_ = 100;
  std::string frame_prefix_ = "";
  std::string local_frame_ = "";
  bool publish_tf_ = false;
  int64_t sensors_sampling_time_ = 0;
  sl::STREAMING_CODEC streaming_codec_ = sl::STREAMING_CODEC::H264;
  sl::SVO_COMPRESSION_MODE svo_compression = sl::SVO_COMPRESSION_MODE::H264;
  bool tracking_enable_ = false;
  bool tracking_set_gravity_as_origin_ = false;
  std::vector<int64_t> video_sd_resolution_ = {0, 0};
  bool video_stream_hd_ = false;
  int64_t video_stream_rate_ = 0;
  std::string video_stream_record_path_ = "";
  bool verbose_ = false;

  /* Node parameters validators. */
  bool validate_camera_fps(const rclcpp::Parameter & p);
  bool validate_camera_resolution(const rclcpp::Parameter & p);
  bool validate_depth_mode(const rclcpp::Parameter & p);
  bool validate_depth_sensing_mode(const rclcpp::Parameter & p);
  bool validate_streaming_codec(const rclcpp::Parameter & p);
  bool validate_svo_compression(const rclcpp::Parameter & p);
  bool validate_tracking_enable(const rclcpp::Parameter & p);

  /* Auxiliary routines. */
  bool open_camera();
  void close_camera();
  void init_camera_info(
    const sl::CameraParameters & zed_params,
    camera_info_manager::CameraInfo & info,
    const std::string & frame_id,
    double stereo_baseline = 0.0);
  void positional_tracking(sl::Pose & camera_pose);
  void sensors_processing(sl::SensorsData & sensors_data);
  cv::Mat sl_to_cv(sl::Mat & input);
  cv::Mat sl_to_cv_depth(sl::Mat & input);
  Image::SharedPtr frame_to_msg(cv::Mat & frame);
};

} // namespace zed_drivers

#endif // ZED_DRIVERS__ZED_DRIVER_HPP
