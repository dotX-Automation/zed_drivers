/**
 * ZED Driver node implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 20, 2023
 */

#include <zed_driver/zed_driver.hpp>

namespace ZEDDriver
{

/**
 * @brief ZEDDriverNode constructor.
 *
 * @param opts Node options for the base node.
 */
ZEDDriverNode::ZEDDriverNode(const rclcpp::NodeOptions & opts)
: NodeBase("zed_driver", opts, true)
{
  init_atomics();
  init_sync_primitives();
  init_parameters();
  init_publishers();
  init_services();
  init_tf2();

  // Initialize Eigen library
  int64_t eigen_cores = this->get_parameter("eigen_max_cores").as_int();
  if (eigen_cores > 0) {
    Eigen::setNbThreads(eigen_cores);
    if (verbose_) {
      std::cout << "Eigen::nbThreads(): " << Eigen::nbThreads() << std::endl;
    }
  }

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
  tf_buffer_.reset();

  // Destroy semaphores
  sem_destroy(&depth_sem_1_);
  sem_destroy(&depth_sem_2_);
}

/**
 * @brief Initializes atomic variables.
 */
void ZEDDriverNode::init_atomics()
{
  running_.store(false, std::memory_order_release);
}

/**
 * @brief Initializes synchronization primitives.
 *
 * @throws RuntimeError if something fails during initialization.
 */
void ZEDDriverNode::init_sync_primitives()
{
  if ((sem_init(&depth_sem_1_, 0, 1) != 0) ||
    (sem_init(&depth_sem_2_, 0, 0) != 0))
  {
    RCLCPP_FATAL(
      this->get_logger(),
      "ZEDDriverNode::init_sync_primitives: Failed to initialize semaphores");
    perror("sem_init");
    throw std::runtime_error(
            "ZEDDriverNode::init_sync_primitives: Failed to initialize semaphores");
  }
}

/**
 * @brief Initializes publishers.
 */
void ZEDDriverNode::init_publishers()
{
  // base_link_odometry
  base_link_odom_pub_ = this->create_publisher<Odometry>(
    "~/base_link_odometry",
    DUAQoS::get_datum_qos(5));

  // base_link_pose
  base_link_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/base_link_pose",
    DUAQoS::get_datum_qos(5));

  // camera_odometry
  camera_odom_pub_ = this->create_publisher<Odometry>(
    "~/camera_odometry",
    DUAQoS::get_datum_qos(5));

  // camera_pose
  camera_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/camera_pose",
    DUAQoS::get_datum_qos(5));

  // imu
  imu_pub_ = this->create_publisher<Imu>(
    "~/imu",
    DUAQoS::get_datum_qos(5));

  // imu_filtered
  imu_filtered_pub_ = this->create_publisher<Imu>(
    "~/imu_filtered",
    DUAQoS::get_datum_qos(5));

  // point_cloud
  point_cloud_pub_ = this->create_publisher<PointCloud2>(
    "~/point_cloud",
    DUAQoS::get_scan_qos(1));

  // point_cloud_roi
  point_cloud_roi_pub_ = this->create_publisher<PointCloud2WithROI>(
    "~/point_cloud_roi",
    DUAQoS::get_datum_qos(1));

  // rviz/base_link_odom
  rviz_base_link_odom_pub_ = this->create_publisher<Odometry>(
    "~/rviz/base_link_odom",
    DUAQoS::Visualization::get_datum_qos(5));

  // rviz/base_link_pose
  rviz_base_link_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/rviz/base_link_pose",
    DUAQoS::Visualization::get_datum_qos(5));

  // rviz/camera_odometry
  rviz_camera_odom_pub_ = this->create_publisher<Odometry>(
    "~/rviz/camera_odometry",
    DUAQoS::Visualization::get_datum_qos(5));

  // rviz/camera_pose
  rviz_camera_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/rviz/camera_pose",
    DUAQoS::Visualization::get_datum_qos(5));

  // rviz/point_cloud
  rviz_point_cloud_pub_ = this->create_publisher<PointCloud2>(
    "~/rviz/point_cloud",
    DUAQoS::Visualization::get_scan_qos(1));

  // rviz/point_cloud_roi
  rviz_point_cloud_roi_pub_ = this->create_publisher<PointCloud2>(
    "~/rviz/point_cloud_roi",
    DUAQoS::Visualization::get_scan_qos(1));

  // rviz/roi
  rviz_roi_pub_ = this->create_publisher<MarkerArray>(
    "~/rviz/roi",
    DUAQoS::Visualization::get_datum_qos(1));

  // left/image_rect_color
  left_rect_pub_ = std::make_shared<image_transport::CameraPublisher>(
    image_transport::create_camera_publisher(
      this,
      "~/left/image_rect_color",
      DUAQoS::get_image_qos().get_rmw_qos_profile()));

  // left/sd/image_rect_color
  left_rect_sd_pub_ = std::make_shared<image_transport::CameraPublisher>(
    image_transport::create_camera_publisher(
      this,
      "~/left/sd/image_rect_color",
      DUAQoS::Visualization::get_image_qos().get_rmw_qos_profile()));

  // right/image_rect_color
  right_rect_pub_ = std::make_shared<image_transport::CameraPublisher>(
    image_transport::create_camera_publisher(
      this,
      "~/right/image_rect_color",
      DUAQoS::get_image_qos().get_rmw_qos_profile()));

  // right/sd/image_rect_color
  right_rect_sd_pub_ = std::make_shared<image_transport::CameraPublisher>(
    image_transport::create_camera_publisher(
      this,
      "~/right/sd/image_rect_color",
      DUAQoS::Visualization::get_image_qos().get_rmw_qos_profile()));

  // left stream
  left_stream_pub_ = std::make_shared<TheoraWrappers::Publisher>(
    this,
    "~/left/image_rect_color",
    DUAQoS::Visualization::get_image_qos().get_rmw_qos_profile());

  // right stream
  right_stream_pub_ = std::make_shared<TheoraWrappers::Publisher>(
    this,
    "~/right/image_rect_color",
    DUAQoS::Visualization::get_image_qos().get_rmw_qos_profile());

  // depth
  depth_pub_ = std::make_shared<image_transport::Publisher>(
    image_transport::create_publisher(
      this,
      "~/depth",
      DUAQoS::get_image_qos().get_rmw_qos_profile()));
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
  // Initialize TF buffers and listeners
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize local data
  odom_to_camera_odom_.header.set__frame_id(odom_frame_);
  odom_to_camera_odom_.set__child_frame_id(camera_odom_frame_);
  map_to_camera_odom_.header.set__frame_id(global_frame_);
  map_to_camera_odom_.set__child_frame_id(camera_odom_frame_);
  base_link_to_camera_.header.set__frame_id(link_namespace_ + "base_link");
  base_link_to_camera_.set__child_frame_id(link_namespace_ + "zed2i_link");

  // Initlaize TF timer
  tf_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(
      &ZEDDriverNode::tf_timer_callback,
      this));
}

} // namespace ZEDDriver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ZEDDriver::ZEDDriverNode)