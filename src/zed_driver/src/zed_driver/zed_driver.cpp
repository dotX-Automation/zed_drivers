/**
 * ZED Driver node implementation.
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
 * @brief ZEDDriverNode constructor.
 *
 * @param opts Node options for the base node.
 */
ZEDDriverNode::ZEDDriverNode(const rclcpp::NodeOptions & opts)
: NodeBase("zed_driver", opts, true)
{
  init_parameters();
  init_publishers();
  init_services();
  init_tf2();

  RCLCPP_INFO(this->get_logger(), "Node initialized");

  // Start camera sampling thread, if requested
  if (autostart_) {
    running_.store(true, std::memory_order_release);
    camera_thread_ = std::thread(
      &ZEDDriverNode::camera_routine,
      this);
  }
}

/**
 * @brief ZEDDriverNode destructor.
 */
ZEDDriverNode::~ZEDDriverNode()
{
  // Stop the camera thread if it is active
  bool is_running = true;
  if (running_.compare_exchange_strong(
      is_running,
      false,
      std::memory_order_release,
      std::memory_order_acquire))
  {
    camera_thread_.join();
    RCLCPP_INFO(this->get_logger(), "Camera thread joined");
  }

  // Destroy image_transport publishers
  left_rect_pub_->shutdown();
  left_rect_pub_.reset();
  left_rect_sd_pub_->shutdown();
  left_rect_sd_pub_.reset();
  right_rect_pub_->shutdown();
  right_rect_pub_.reset();
  right_rect_sd_pub_->shutdown();
  right_rect_sd_pub_.reset();
  left_stream_pub_.reset();
  right_stream_pub_.reset();
  depth_pub_->shutdown();
  depth_pub_.reset();

  // Stop TF listeners
  tf_listener_.reset();
  tf_broadcaster_.reset();
  tf_buffer_.reset();
}

/**
 * @brief Initializes publishers.
 */
void ZEDDriverNode::init_publishers()
{
  bool video_sd_reliable = this->get_parameter("video_sd_reliable").as_bool();

  // base_link_odometry
  base_link_odom_pub_ = this->create_publisher<Odometry>(
    "~/base_link_odometry",
    dua_qos::Reliable::get_datum_qos());

  // base_link_pose
  base_link_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/base_link_pose",
    dua_qos::Reliable::get_datum_qos());

  // camera_odometry
  camera_odom_pub_ = this->create_publisher<Odometry>(
    "~/camera_odometry",
    dua_qos::Reliable::get_datum_qos());

  // camera_pose
  camera_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/camera_pose",
    dua_qos::Reliable::get_datum_qos());

  // imu
  imu_pub_ = this->create_publisher<Imu>(
    "~/imu",
    dua_qos::Reliable::get_datum_qos());

  // point_cloud
  point_cloud_pub_ = this->create_publisher<PointCloud2>(
    "~/point_cloud",
    dua_qos::Reliable::get_scan_qos());

  // left/camera_info
  left_info_pub_ = this->create_publisher<CameraInfo>(
    "~/left/camera_info",
    dua_qos::Reliable::get_datum_qos());

  // left/image_rect_color
  left_rect_pub_ = std::make_shared<image_transport::Publisher>(
    image_transport::create_publisher(
      this,
      "~/left/image_rect_color",
      dua_qos::Reliable::get_image_qos().get_rmw_qos_profile()));

  // left/sd/camera_info
  left_sd_info_pub_ = this->create_publisher<CameraInfo>(
    "~/left/sd/camera_info",
    video_sd_reliable ? dua_qos::Reliable::get_datum_qos() : dua_qos::BestEffort::get_datum_qos());

  // left/sd/image_rect_color
  left_rect_sd_pub_ = std::make_shared<image_transport::Publisher>(
    image_transport::create_publisher(
      this,
      "~/left/sd/image_rect_color",
      video_sd_reliable ? dua_qos::Reliable::get_image_qos().get_rmw_qos_profile() :
      dua_qos::BestEffort::get_image_qos().get_rmw_qos_profile()));

  // right/camera_info
  right_info_pub_ = this->create_publisher<CameraInfo>(
    "~/right/camera_info",
    dua_qos::Reliable::get_datum_qos());

  // right/image_rect_color
  right_rect_pub_ = std::make_shared<image_transport::Publisher>(
    image_transport::create_publisher(
      this,
      "~/right/image_rect_color",
      dua_qos::Reliable::get_image_qos().get_rmw_qos_profile()));

  // right/sd/camera_info
  right_sd_info_pub_ = this->create_publisher<CameraInfo>(
    "~/right/sd/camera_info",
    video_sd_reliable ? dua_qos::Reliable::get_datum_qos() : dua_qos::BestEffort::get_datum_qos());

  // right/sd/image_rect_color
  right_rect_sd_pub_ = std::make_shared<image_transport::Publisher>(
    image_transport::create_publisher(
      this,
      "~/right/sd/image_rect_color",
      video_sd_reliable ? dua_qos::Reliable::get_image_qos().get_rmw_qos_profile() :
      dua_qos::BestEffort::get_image_qos().get_rmw_qos_profile()));

  // left stream
  left_stream_pub_ = std::make_shared<TheoraWrappers::Publisher>(
    this,
    "~/left/image_rect_color",
    dua_qos::BestEffort::get_image_qos().get_rmw_qos_profile());

  // right stream
  right_stream_pub_ = std::make_shared<TheoraWrappers::Publisher>(
    this,
    "~/right/image_rect_color",
    dua_qos::BestEffort::get_image_qos().get_rmw_qos_profile());

  // depth
  depth_pub_ = std::make_shared<image_transport::Publisher>(
    image_transport::create_publisher(
      this,
      "~/depth",
      dua_qos::Reliable::get_image_qos().get_rmw_qos_profile()));
}

/**
 * @brief Initializes services.
 */
void ZEDDriverNode::init_services()
{
  // enable
  enable_srv_ = this->create_service<SetBool>(
    "~/enable",
    std::bind(
      &ZEDDriverNode::enable_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

/**
 * @brief Initializes tf2 data.
 */
void ZEDDriverNode::init_tf2()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

} // namespace zed_drivers

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(zed_drivers::ZEDDriverNode)
