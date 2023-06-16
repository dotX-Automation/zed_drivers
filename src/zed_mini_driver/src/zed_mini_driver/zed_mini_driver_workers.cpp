/**
 * Worker threads of the ZED Mini driver node.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 13, 2023
 */

#define NOOP ((void)0)

#include <zed_mini_driver/zed_mini_driver.hpp>

namespace ZEDMiniDriver
{

/**
 * @brief Camera sampling routine: gets and publishes data from all sensors.
 *
 * @throws RuntimeError if the camera cannot be opened.s
 */
void ZEDMiniDriverNode::camera_routine()
{
  // Open camera
  if (!open_camera()) {
    throw std::runtime_error("ZEDMiniDriverNode::camera_routine: Failed to open camera");
  }

  // Run until stopped
  while (running_.load(std::memory_order_acquire))
  {
    // TODO
  }

  // Close camera
  close_camera();
}

/**
 * @brief Listens for incoming TFs.
 */
void ZEDMiniDriverNode::tf_thread_routine()
{
  // Initialize TF buffers and listeners
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  std::string map_frame = "map";
  std::string odom_frame = link_namespace_ + "odom";
  std::string zedm_odom_frame = link_namespace_ + "zedm_odom";
  TransformStamped odom_to_camera_odom{}, map_to_camera_odom{};

  // Start listening
  while (tf_listening_.load(std::memory_order_acquire))
  {
    // odom -> zedm_odom
    try {
      odom_to_camera_odom = tf_buffer_->lookupTransform(
        zedm_odom_frame,
        odom_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(1.0));

      tf_lock_.lock();
      odom_to_camera_odom_ = odom_to_camera_odom;
      tf_lock_.unlock();
    } catch (const tf2::TimeoutException & e) {
      NOOP;
    } catch (const tf2::TransformException & e) {
      RCLCPP_INFO(this->get_logger(), "TF exception: %s", e.what());
    }

    // map -> zedm_odom
    try {
      map_to_camera_odom = tf_buffer_->lookupTransform(
        zedm_odom_frame,
        map_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(1.0));

      tf_lock_.lock();
      map_to_camera_odom_ = map_to_camera_odom;
      tf_lock_.unlock();
    } catch (const tf2::TimeoutException & e) {
      NOOP;
    } catch (const tf2::TransformException & e) {
      RCLCPP_INFO(this->get_logger(), "TF exception: %s", e.what());
    }
  }

  // Stop listening
  tf_listener_.reset();
  tf_buffer_.reset();
}

} // namespace ZEDMiniDriver
