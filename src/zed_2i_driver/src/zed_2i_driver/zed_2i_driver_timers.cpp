/**
 * ZED 2i Driver node timer callbacks.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 20, 2023
 */

#define NOOP ((void)0)

#include <zed_2i_driver/zed_2i_driver.hpp>

namespace ZED2iDriver
{

/**
 * @brief Updates tf2 transforms.
 */
void ZED2iDriverNode::tf_timer_callback()
{
  TransformStamped odom_to_camera_odom{}, map_to_camera_odom{};

  // Start listening
  // odom -> zedm_odom, base_link -> zed2i_left_link (it's rigid)
  try {
    odom_to_camera_odom = tf_buffer_->lookupTransform(
      odom_frame_,
      zed2i_odom_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(1.0));

    tf_lock_.lock();
    odom_to_camera_odom_ = odom_to_camera_odom;
    base_link_to_camera_ = odom_to_camera_odom;
    base_link_to_camera_.header.set__frame_id(link_namespace_ + "base_link");
    base_link_to_camera_.set__child_frame_id(link_namespace_ + "zed2i_link");
    tf_lock_.unlock();
  } catch (const tf2::TimeoutException & e) {
    NOOP;
  } catch (const tf2::TransformException & e) {
    RCLCPP_INFO(this->get_logger(), "TF exception: %s", e.what());
  }

  // map -> zedm_odom
  try {
    map_to_camera_odom = tf_buffer_->lookupTransform(
      map_frame_,
      zed2i_odom_frame_,
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

} // namespace ZED2iDriver
