/**
 * ZED Mini Driver node implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 13, 2023
 */

#include <zed_mini_driver/zed_mini_driver.hpp>

namespace ZEDMiniDriver
{

/**
 * @brief ZEDMiniDriverNode constructor.
 *
 * @param opts Node options for the base node.
 */
ZEDMiniDriverNode::ZEDMiniDriverNode(const rclcpp::NodeOptions & opts)
: NodeBase("zed_mini_driver", opts, true)
{
  init_atomics();
  init_parameters();
  init_subscriptions();
  init_publishers();
  init_services();

  // Start TF listener thread
  tf_thread_ = std::thread(&ZEDMiniDriverNode::tf_thread_routine, this);

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief ZEDMiniDriverNode destructor.
 */
ZEDMiniDriverNode::~ZEDMiniDriverNode()
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
  }

  // Stop the TF listener thread
  tf_listening_.store(false, std::memory_order_release);
  tf_thread_.join();

  // Destroy image_transport publishers
  left_rect_pub_->shutdown();
  left_rect_pub_.reset();
  right_rect_pub_->shutdown();
  right_rect_pub_.reset();
  depth_pub_->shutdown();
  depth_pub_.reset();

  // Destroy camera_info_managers
  left_info_manager_.reset();
  right_info_manager_.reset();
  depth_info_manager_.reset();
}

/**
 * @brief Initializes atomic variables.
 */
void ZEDMiniDriverNode::init_atomics()
{
  tf_listening_.store(true, std::memory_order_release);
  running_.store(false, std::memory_order_release);
}

} // namespace ZEDMiniDriver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ZEDMiniDriver::ZEDMiniDriverNode)
