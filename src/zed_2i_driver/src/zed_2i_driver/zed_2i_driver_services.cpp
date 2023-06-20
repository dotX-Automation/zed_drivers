/**
 * ZED 2i Driver node service callbacks.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 20, 2023
 */

#include <zed_2i_driver/zed_2i_driver.hpp>

namespace ZED2iDriver
{

/**
 * @brief Enable service callback.
 *
 * @param req Service request.
 * @param resp Service response.
 */
void ZED2iDriverNode::enable_callback(
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
        &ZED2iDriverNode::camera_routine,
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

} // namespace ZED2iDriver
