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
  init_publishers();
  init_services();

  // Initialize camera_info_managers
  left_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this,
    "zedm_left");
  right_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this,
    "zedm_right");
  depth_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this,
    "zedm_depth");
  if (!left_info_manager_->loadCameraInfo(this->get_parameter("left_camera_info_url").as_string())) {
    RCLCPP_ERROR(this->get_logger(), "ZEDMiniDriverNode::ZEDMiniDriverNode: Left camera info not found");
  } else {
    left_info_ = left_info_manager_->getCameraInfo();
  }
  if (!right_info_manager_->loadCameraInfo(this->get_parameter("right_camera_info_url").as_string())) {
    RCLCPP_ERROR(this->get_logger(), "ZEDMiniDriverNode::ZEDMiniDriverNode: Right camera info not found");
  } else {
    right_info_ = right_info_manager_->getCameraInfo();
  }
  if (!depth_info_manager_->loadCameraInfo(this->get_parameter("depth_camera_info_url").as_string())) {
    RCLCPP_ERROR(this->get_logger(), "ZEDMiniDriverNode::ZEDMiniDriverNode: Depth camera info not found");
  } else {
    depth_info_ = depth_info_manager_->getCameraInfo();
  }
  left_info_.header.set__frame_id(link_namespace_ + "zedm_link/left");
  right_info_.header.set__frame_id(link_namespace_ + "zedm_link/right");
  depth_info_.header.set__frame_id(link_namespace_ + "zedm_link/left");

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

/**
 * @brief Initializes publishers.
 */
void ZEDMiniDriverNode::init_publishers()
{
  // camera_pose
  camera_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/camera_pose",
    DUAQoS::get_datum_qos(5));

  // base_link_pose
  base_link_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/base_link_pose",
    DUAQoS::get_datum_qos(5));

  // imu
  imu_pub_ = this->create_publisher<Imu>(
    "~/imu",
    DUAQoS::get_datum_qos(5));

  // point_cloud
  point_cloud_pub_ = this->create_publisher<PointCloud2>(
    "~/point_cloud",
    DUAQoS::get_scan_qos(1));

  // point_cloud_roi
  point_cloud_roi_pub_ = this->create_publisher<PointCloud2WithROI>(
    "~/point_cloud_roi",
    DUAQoS::get_datum_qos(1));

  // rviz/camera_pose
  rviz_camera_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/rviz/camera_pose",
    DUAQoS::Visualization::get_datum_qos(5));

  // rviz/base_link_pose
  rviz_base_link_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/rviz/base_link_pose",
    DUAQoS::Visualization::get_datum_qos(5));

  // rviz/point_cloud
  rviz_point_cloud_pub_ = this->create_publisher<PointCloud2>(
    "~/rviz/point_cloud",
    DUAQoS::Visualization::get_scan_qos(1));

  // left/image_rect_color
  left_rect_pub_ = std::make_shared<image_transport::CameraPublisher>(
    image_transport::create_camera_publisher(
      this,
      "~/left/image_rect_color",
      DUAQoS::get_image_qos().get_rmw_qos_profile()));

  // right/image_rect_color
  right_rect_pub_ = std::make_shared<image_transport::CameraPublisher>(
    image_transport::create_camera_publisher(
      this,
      "~/right/image_rect_color",
      DUAQoS::get_image_qos().get_rmw_qos_profile()));

  // depth
  depth_pub_ = std::make_shared<image_transport::CameraPublisher>(
    image_transport::create_camera_publisher(
      this,
      "~/depth",
      DUAQoS::get_image_qos().get_rmw_qos_profile()));
}

/**
 * @brief Initializes services.
 */
void ZEDMiniDriverNode::init_services()
{
  // enable
  enable_srv_ = this->create_service<SetBool>(
    "~/enable",
    std::bind(
      &ZEDMiniDriverNode::enable_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

} // namespace ZEDMiniDriver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ZEDMiniDriver::ZEDMiniDriverNode)
