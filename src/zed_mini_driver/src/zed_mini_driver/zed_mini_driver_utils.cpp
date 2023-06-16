/**
 * Utility routines for the ZED Mini Driver node.
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
 * @brief Opens the camera.
 *
 * @return True if the camera was opened successfully, false otherwise.
 */
bool ZEDMiniDriverNode::open_camera()
{
  // Get node parameters
  std::string camera_settings_path = this->get_parameter("camera_settings_path").as_string();
  double depth_max_distance = this->get_parameter("depth_max_distance").as_double();
  double depth_min_distance = this->get_parameter("depth_min_distance").as_double();
  int64_t depth_stabilization = this->get_parameter("depth_stabilization").as_int();
  int64_t serial_number = this->get_parameter("serial_number").as_int();

  // Camera initialization parameters
  // The rationale for this is:
  // - Some parameters can be set from outside
  // - Some parameters are fixed given our use case for this sensor
  // See the ZED SDK documentation for more details
  sl::InitParameters init_params;
  init_params.camera_resolution = resolution_; // This must be validated
  init_params.camera_fps = fps_; // This must be validated
  init_params.camera_image_flip = sl::FLIP_MODE::OFF;
  init_params.camera_disable_self_calib = false;
  init_params.enable_right_side_measure = false;
  init_params.svo_real_time_mode = true;
  init_params.depth_mode = depth_mode_; // This must be validated
  init_params.depth_stabilization = static_cast<int>(depth_stabilization);
  init_params.depth_minimum_distance = static_cast<float>(depth_min_distance);
  init_params.depth_maximum_distance = static_cast<float>(depth_max_distance);
  init_params.coordinate_units = sl::UNIT::METER;
  init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
  init_params.sdk_gpu_id = -1;
  init_params.sdk_verbose = 0;
  init_params.sdk_verbose_log_file = sl::String("");
  // init_params.sdk_cuda_ctx = nullptr;
  init_params.optional_settings_path = sl::String(camera_settings_path.c_str());
  init_params.sensors_required = true;
  init_params.enable_image_enhancement = true;
  init_params.open_timeout_sec = 5.0f;
  init_params.async_grab_camera_recovery = true;

  // Configure input source
  if (serial_number != -1) {
    sl::InputType input_type;
    input_type.setFromSerialNumber(static_cast<unsigned int>(serial_number));
    init_params.input = input_type;
  }

  // Camera positional tracking parameters
  sl::PositionalTrackingParameters tracking_params;
  tracking_params.enable_area_memory = true;
  tracking_params.enable_pose_smoothing = true;
  tracking_params.set_floor_as_origin = false;
  tracking_params.enable_imu_fusion = true;
  tracking_params.set_as_static = false;
  tracking_params.depth_min_range = -1.0f;
  tracking_params.set_gravity_as_origin = true;

  sl::ERROR_CODE err;

  // Open the camera
  err = zed_.open(init_params);
  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_FATAL(
      this->get_logger(),
      "ZEDMiniDriverNode::open_camera: Failed to open the camera (%d): %s",
      static_cast<int>(err),
      sl::toString(err).c_str());
    return false;
  }

  // Enable positional tracking
  err = zed_.enablePositionalTracking(tracking_params);
  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_FATAL(
      this->get_logger(),
      "ZEDMiniDriverNode::open_camera: Failed to enable positional tracking (%d): %s",
      static_cast<int>(err),
      sl::toString(err).c_str());
    zed_.close();
    return false;
  }

  sl::CameraInformation zed_info = zed_.getCameraInformation();
  sl::CameraParameters left_info = zed_info.camera_configuration.calibration_parameters.left_cam;
  sl::CameraParameters right_info = zed_info.camera_configuration.calibration_parameters.right_cam;

  // Set left camera_info
  left_info_.header.set__frame_id(link_namespace_ + "zedm_left_link");
  left_info_.set__height(static_cast<uint32_t>(zed_info.camera_configuration.resolution.height));
  left_info_.set__width(static_cast<uint32_t>(zed_info.camera_configuration.resolution.width));
  left_info_.set__distortion_model("plumb_bob");
  left_info_.d.reserve(5);
  for (int i = 0; i < 5; i++) {
    left_info_.d.push_back(left_info.disto[i]);
  }
  left_info_.k[0] = left_info.fx;
  left_info_.k[2] = left_info.cx;
  left_info_.k[4] = left_info.fy;
  left_info_.k[5] = left_info.cy;
  left_info_.k[8] = 1.0;
  left_info_.r[0] = 1.0;
  left_info_.r[4] = 1.0;
  left_info_.r[8] = 1.0;
  left_info_.p[0] = left_info.fx;
  left_info_.p[2] = left_info.cx;
  left_info_.p[5] = left_info.fy;
  left_info_.p[6] = left_info.cy;
  left_info_.p[10] = 1.0;

  // Set right camera_info
  right_info_.header.set__frame_id(link_namespace_ + "zedm_right_link");
  right_info_.set__height(static_cast<uint32_t>(zed_info.camera_configuration.resolution.height));
  right_info_.set__width(static_cast<uint32_t>(zed_info.camera_configuration.resolution.width));
  right_info_.set__distortion_model("plumb_bob");
  right_info_.d.reserve(5);
  for (int i = 0; i < 5; i++) {
    right_info_.d.push_back(right_info.disto[i]);
  }
  right_info_.k[0] = right_info.fx;
  right_info_.k[2] = right_info.cx;
  right_info_.k[4] = right_info.fy;
  right_info_.k[5] = right_info.cy;
  right_info_.k[8] = 1.0;
  right_info_.r[0] = 1.0;
  right_info_.r[4] = 1.0;
  right_info_.r[8] = 1.0;
  right_info_.p[0] = right_info.fx;
  right_info_.p[2] = right_info.cx;
  right_info_.p[3] =
    -right_info.fx * zed_info.camera_configuration.calibration_parameters.getCameraBaseline();
  right_info_.p[5] = right_info.fy;
  right_info_.p[6] = right_info.cy;
  right_info_.p[10] = 1.0;
}

/**
 * @brief Closes the camera.
 */
void ZEDMiniDriverNode::close_camera()
{
  if (zed_.isOpened()) {
    zed_.close();
  }
}

/**
 * @brief Converts a frame into an Image message.
 *
 * @param frame cv::Mat storing the frame.
 * @return Shared pointer to a new Image message.
 */
Image::SharedPtr ZEDMiniDriverNode::frame_to_msg(cv::Mat & frame)
{
  auto ros_image = std::make_shared<Image>();

  // Set frame-relevant fields
  ros_image->set__width(frame.cols);
  ros_image->set__height(frame.rows);
  ros_image->set__encoding(sensor_msgs::image_encodings::BGR8);
  ros_image->set__step(frame.cols * frame.elemSize());
  ros_image->set__is_bigendian(false);

  // Copy frame data
  size_t size = ros_image->step * frame.rows;
  ros_image->data.resize(size);
  memcpy(ros_image->data.data(), frame.data, size);

  return ros_image;
}

/**
 * @brief Conversion function between sl::Mat and cv::Mat.
 *
 * @param slMat Original frame-holding data structure.
 * @return OpenCV frame-holding data structure, pointing to the same data.
 */
cv::Mat ZEDMiniDriverNode::slMat2cvMat(sl::Mat & input)
{
  // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer
  // from sl::Mat::getPtr<T>()
  // cv::Mat and sl::Mat will share a single memory structure
  return cv::Mat(
    input.getHeight(),
    input.getWidth(),
    CV_8UC4,
    input.getPtr<sl::uchar1>(sl::MEM::CPU),
    input.getStepBytes(sl::MEM::CPU));
}

/**
 * @brief Validates the depth_mode parameter.
 *
 * @param p Parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool ZEDMiniDriverNode::validate_depth_mode(const rclcpp::Parameter & p)
{
  std::string depth_mode = p.as_string();

  if (depth_mode == "PERFORMANCE") {
    depth_mode_ = sl::DEPTH_MODE::PERFORMANCE;
  } else if (depth_mode == "QUALITY") {
    depth_mode_ = sl::DEPTH_MODE::QUALITY;
  } else if (depth_mode == "ULTRA") {
    depth_mode_ = sl::DEPTH_MODE::ULTRA;
  } else if (depth_mode == "NEURAL") {
    depth_mode_ = sl::DEPTH_MODE::NEURAL;
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "ZEDMiniDriverNode::validate_depth_mode: Invalid depth_mode parameter: %s",
      depth_mode.c_str());
    return false;
  }
  return true;
}

/**
 * @brief Validates the fps parameter.
 *
 * @param p Parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool ZEDMiniDriverNode::validate_fps(const rclcpp::Parameter & p)
{
  int64_t fps = p.as_int();

  switch (fps) {
    case 15:
    case 30:
    case 60:
    case 100:
      fps_ = static_cast<int>(fps);
      return true;
    default:
      RCLCPP_ERROR(
        this->get_logger(),
        "ZEDMiniDriverNode::validate_fps: Invalid fps parameter: %ld",
        fps);
      return false;
  }
}

/**
 * @brief Validates the resolution parameter.
 *
 * @param p Parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool ZEDMiniDriverNode::validate_resolution(const rclcpp::Parameter & p)
{
  std::string resolution = p.as_string();

  if (resolution == "HD2K") {
    resolution_ = sl::RESOLUTION::HD2K;
  } else if (resolution == "HD1080") {
    resolution_ = sl::RESOLUTION::HD1080;
  } else if (resolution == "HD720") {
    resolution_ = sl::RESOLUTION::HD720;
  } else if (resolution == "VGA") {
    resolution_ = sl::RESOLUTION::VGA;
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "ZEDMiniDriverNode::validate_resolution: Invalid resolution parameter: %s",
      resolution.c_str());
    return false;
  }
  return true;
}

} // namespace ZEDMiniDriver
