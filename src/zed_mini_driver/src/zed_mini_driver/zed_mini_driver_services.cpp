/**
 * ZED Mini Driver node service callbacks.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 16, 2023
 */

#include <zed_mini_driver/zed_mini_driver.hpp>

namespace ZEDMiniDriver
{

/**
 * @brief Enable service callback.
 *
 * @param req Service request.
 * @param resp Service response.
 */
void ZEDMiniDriverNode::enable_callback(
  const SetBool::Request::SharedPtr req,
  const SetBool::Response::SharedPtr resp)
{
  if (req->data) {
    bool expected = false;
    if (running_.compare_exchange_strong(
        expected,
        true,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      // Start camera sampling thread
      camera_thread_ = std::thread(
        &ZEDMiniDriverNode::camera_routine,
        this);
    }
  } else {
    bool expected = true;
    if (running_.compare_exchange_strong(
        expected,
        false,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      camera_thread_.join();
      RCLCPP_INFO(this->get_logger(), "Camera sampling thread joined");
    }
  }
  resp->set__success(true);
  resp->set__message("");
}

} // namespace ZEDMiniDriver
