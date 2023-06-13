/**
 * ZED Mini Driver node definition.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 13, 2023
 */

#ifndef ZED_DRIVERS__ZED_MINI_DRIVER_HPP
#define ZED_DRIVERS__ZED_MINI_DRIVER_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <sl/Camera.hpp>

#include <rclcpp/rclcpp.hpp>

#include <pose_kit/pose.hpp>

#include <dua_node/dua_node.hpp>
#include <dua_qos/dua_qos.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <std_srvs/srv/set_bool.hpp>

using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;

namespace ZEDMiniDriver
{

/**
 * Drives a ZED Mini camera.
 */
class ZEDMiniDriverNode : public DUANode::NodeBase
{
public:
  ZEDMiniDriverNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());
  virtual ~ZEDMiniDriverNode();

private:
  /* Node initialization routines. */
  void init_atomics();
  void init_parameters();
  void init_subscriptions();
  void init_publishers();
  void init_services();

  /* Topic subscriptions. */

  /* TF subscribers. */

  /* Topic subscription callbacks. */

  /* Topic publishers. */

  /* image_transport publishers. */

  /* Service servers. */

  /* Service callbacks. */

  /* Synchronization primitives. */

  /* Internal state variables. */

  /* Camera instance, sampling thread and routine. */

  /* Node parameters. */

  /* Node parameters validators. */

  /* Auxiliary routines. */
  bool open_camera();
  void close_camera();
  cv::Mat slMat2cvMat(sl::Mat & input);
  Image::SharedPtr frame_to_msg(cv::Mat & frame);
};

} // namespace ZEDMiniDriver

#endif // ZED_DRIVERS__ZED_MINI_DRIVER_HPP
