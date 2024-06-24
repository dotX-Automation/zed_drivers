/**
 * Utility routines for the ZED Driver node.
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

namespace ZEDDriver
{

/**
 * @brief Opens the camera.
 *
 * @return True if the camera was opened successfully, false otherwise.
 */
bool ZEDDriverNode::open_camera()
{
  // Get node parameters
  std::string camera_settings_path = this->get_parameter("camera_settings_path").as_string();
  double depth_max_distance = this->get_parameter("depth_max_distance").as_double();
  double depth_min_distance = this->get_parameter("depth_min_distance").as_double();
  int64_t depth_stabilization = this->get_parameter("depth_stabilization").as_int();
  int64_t serial_number = this->get_parameter("serial_number").as_int();
  std::string streaming_codec = this->get_parameter("streaming_codec").as_string();
  std::string streaming_sender_ip = this->get_parameter("streaming_sender_ip").as_string();
  unsigned short streaming_sender_port = this->get_parameter("streaming_sender_port").as_int();
  std::string svo_file = this->get_parameter("svo_file").as_string();
  bool svo_record = this->get_parameter("svo_record").as_bool();
  bool svo_transcode_stream = this->get_parameter("svo_transcode_stream").as_bool();

  // Detect invalid parameter configurations
  if (!streaming_codec.empty() && !streaming_sender_ip.empty()) {
    RCLCPP_FATAL(
      this->get_logger(),
      "ZEDDriverNode::open_camera: Cannot send and receive stream at the same time");
    return false;
  }

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
  init_params.async_grab_camera_recovery = false;

  // Configure input source
  if (serial_number != -1) {
    // First, scan all available cameras
    std::vector<sl::DeviceProperties> available_zeds = sl::Camera::getDeviceList();
    if (available_zeds.empty()) {
      RCLCPP_FATAL(
        this->get_logger(),
        "ZEDDriverNode::open_camera: No ZED cameras found");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Found %lu ZED cameras", available_zeds.size());
    int req_id = -1;
    for (auto & zed_properties : available_zeds) {
      int id = zed_properties.id;
      //std::string zed_path(zed_properties.path.c_str());
      unsigned int serial = zed_properties.serial_number;
      //std::string model(sl::toString(zed_properties.camera_model).c_str());
      //std::string input_type(sl::toString(zed_properties.input_type).c_str());
      //std::string state(sl::toString(zed_properties.camera_state).c_str());

      if (verbose_) {
        std::cout << "ZED " << id << std::endl; //" (" << zed_path << ")" << std::endl;
        std::cout << " - Serial number: " << serial << std::endl;
      //  std::cout << " - Model: " << model << std::endl;
      //  std::cout << " - Input type: " << input_type << std::endl;
      //  std::cout << " - State: " << state << std::endl << std::endl;
      }

      if (serial == static_cast<unsigned int>(serial_number)) {
        req_id = id;
        RCLCPP_INFO(
          this->get_logger(),
          "Opening camera %d matching S/N (%u = %u)",
          id,
          static_cast<unsigned int>(serial_number),
          serial);
      }
    }

    // Then, check if the requested serial number is available
    if (req_id == -1) {
      RCLCPP_FATAL(
        this->get_logger(),
        "ZEDDriverNode::open_camera: Failed to find requested camera (S/N %u)",
        static_cast<unsigned int>(serial_number));
      return false;
    }

    // Finally, open camera by ID
    sl::InputType input_type;
    input_type.setFromCameraID(req_id);
    init_params.input = input_type;
    physical_camera_ = true;
  } else if (!streaming_sender_ip.empty()) {
    sl::InputType input_type;
    input_type.setFromStream(sl::String(streaming_sender_ip.c_str()), streaming_sender_port);
    init_params.input = input_type;
    physical_camera_ = false;
    RCLCPP_INFO(
      this->get_logger(),
      "Opening remote camera at %s:%d",
      streaming_sender_ip.c_str(),
      streaming_sender_port);
  } else if (!svo_record && !svo_file.empty()) {
    sl::InputType input_type;
    input_type.setFromSVOFile(sl::String(svo_file.c_str()));
    init_params.input = input_type;
    physical_camera_ = false;
    RCLCPP_INFO(this->get_logger(), "Opening SVO file %s", svo_file.c_str());
  } else {
    // If all the above fail, the first compatible ZED device found will be opened
    physical_camera_ = true;
  }

  sl::ERROR_CODE err;

  // Open the camera
  err = zed_.open(init_params);
  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_FATAL(
      this->get_logger(),
      "ZEDDriverNode::open_camera: Failed to open the camera (%d): %s",
      static_cast<int>(err),
      sl::toString(err).c_str());
    return false;
  }

  // Enable streaming as sender
  if (!streaming_codec.empty()) {
    sl::StreamingParameters stream_params;
    stream_params.adaptative_bitrate = this->get_parameter("streaming_adaptative_bitrate").as_bool();
    stream_params.bitrate = this->get_parameter("streaming_bitrate").as_int();
    stream_params.chunk_size = this->get_parameter("streaming_chunk_size").as_int();
    stream_params.codec = streaming_codec_; // This must be validated
    stream_params.gop_size = this->get_parameter("streaming_gop_size").as_int();
    stream_params.port = this->get_parameter("streaming_sender_port").as_int();
    stream_params.target_framerate = this->get_parameter("streaming_target_fps").as_int();

    err = zed_.enableStreaming(stream_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_FATAL(
        this->get_logger(),
        "ZEDDriverNode::open_camera: Failed to enable streaming as sender (%d): %s",
        static_cast<int>(err),
        sl::toString(err).c_str());
      zed_.close();
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Streaming enabled");
  }

  // Enable SVO file recording
  if (svo_record && !svo_file.empty()) {
    sl::RecordingParameters recording_params;
    recording_params.compression_mode = svo_compression;
    recording_params.video_filename = sl::String(svo_file.c_str());
    recording_params.transcode_streaming_input = svo_transcode_stream;

    err = zed_.enableRecording(recording_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_FATAL(
        this->get_logger(),
        "ZEDDriverNode::open_camera: Failed to enable SVO recording (%d): %s",
        static_cast<int>(err),
        sl::toString(err).c_str());
      zed_.close();
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "SVO recording enabled");
  }

  // Enable positional tracking
  if (enable_tracking_) {
    sl::PositionalTrackingParameters tracking_params;
    tracking_params.enable_area_memory =
      this->get_parameter("tracking_enable_area_memory").as_bool();
    tracking_params.enable_pose_smoothing =
      this->get_parameter("tracking_enable_pose_smoothing").as_bool();
    tracking_params.set_floor_as_origin = false;
    tracking_params.enable_imu_fusion =
      this->get_parameter("tracking_enable_imu_fusion").as_bool();
    tracking_params.set_as_static = false;
    tracking_params.depth_min_range =
      this->get_parameter("tracking_depth_min_range").as_double();
    tracking_params.set_gravity_as_origin = tracking_set_gravity_as_origin_;

    err = zed_.enablePositionalTracking(tracking_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_FATAL(
        this->get_logger(),
        "ZEDDriverNode::open_camera: Failed to enable positional tracking (%d): %s",
        static_cast<int>(err),
        sl::toString(err).c_str());
      zed_.close();
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Positional tracking enabled");
  }

  sl::CameraInformation zed_info = zed_.getCameraInformation();
  sl::CameraInformation zed_info_sd = zed_.getCameraInformation(
    sl::Resolution(
      sd_resolution_[0],
      sd_resolution_[1]));
  sl::CameraParameters left_info = zed_info.camera_configuration.calibration_parameters.left_cam;
  sl::CameraParameters left_sd_info =
    zed_info_sd.camera_configuration.calibration_parameters.left_cam;
  sl::CameraParameters right_info = zed_info.camera_configuration.calibration_parameters.right_cam;
  sl::CameraParameters right_sd_info =
    zed_info_sd.camera_configuration.calibration_parameters.right_cam;
  camera_model_ = zed_info.camera_model;

  // Initialize frame names
  if (camera_model_ == sl::MODEL::ZED_M) {
    camera_name_ = "zedm";
  } else if (camera_model_ == sl::MODEL::ZED) {
    camera_name_ = "zed";
  } else if (camera_model_ == sl::MODEL::ZED2) {
    camera_name_ = "zed2";
  } else if (camera_model_ == sl::MODEL::ZED2i) {
    camera_name_ = "zed2i";
  } else if (camera_model_ == sl::MODEL::ZED_X) {
    camera_name_ = "zedx";
  } else if (camera_model_ == sl::MODEL::ZED_XM) {
    camera_name_ = "zedxm";
  } else {
    camera_name_ = "zed";
    RCLCPP_WARN(
      this->get_logger(),
      "Camera model %d might not be supported",
      static_cast<int>(camera_model_));
  }
  camera_frame_ = link_namespace_ + camera_name_ + "_link";
  camera_odom_frame_ = link_namespace_ + camera_name_ + "_odom";
  camera_left_frame_ = link_namespace_ + camera_name_ + "_left_link";
  camera_right_frame_ = link_namespace_ + camera_name_ + "_right_link";
  camera_imu_frame_ = link_namespace_ + camera_name_ + "_imu_link";

  // Initialize camera_infos
  init_camera_info(
    left_info,
    left_info_,
    camera_left_frame_);
  init_camera_info(
    right_info,
    right_info_,
    camera_right_frame_,
    -right_info.fx * zed_info.camera_configuration.calibration_parameters.getCameraBaseline());
  init_camera_info(
    left_sd_info,
    left_sd_info_,
    camera_left_frame_);
  init_camera_info(
    right_sd_info,
    right_sd_info_,
    camera_right_frame_,
    -right_sd_info.fx *
    zed_info_sd.camera_configuration.calibration_parameters.getCameraBaseline());

  // Print IMU->Left camera transform
  if (verbose_) {
    std::cout << "Serial number: " << zed_info.serial_number << std::endl;

    sl::Transform imu_to_left = zed_info.sensors_configuration.camera_imu_transform;
    std::cout << "IMU to Left Camera transform:" << std::endl;
    std::cout << " - translation: [ "
              << imu_to_left.getTranslation().tx << ", "
              << imu_to_left.getTranslation().ty << ", "
              << imu_to_left.getTranslation().tz
              << " ] (X, Y, Z)" << std::endl;
    std::cout << " - rotation: [ "
              << imu_to_left.getOrientation().ow << ", "
              << imu_to_left.getOrientation().ox << ", "
              << imu_to_left.getOrientation().oy << ", "
              << imu_to_left.getOrientation().oz
              << " ] (q_w, q_x, q_y, q_z)" << std::endl;
  }

  return true;
}

/**
 * @brief Closes the camera.
 */
void ZEDDriverNode::close_camera()
{
  if (zed_.isOpened()) {
    zed_.close();
    physical_camera_ = false;
  }
}

/**
 * @brief Converts a frame into an Image message.
 *
 * @param frame cv::Mat storing the frame.
 * @return Shared pointer to a new Image message.
 */
Image::SharedPtr ZEDDriverNode::frame_to_msg(cv::Mat & frame)
{
  auto ros_image = std::make_shared<Image>();

  // Set frame-relevant fields
  ros_image->set__width(frame.cols);
  ros_image->set__height(frame.rows);
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
cv::Mat ZEDDriverNode::sl_to_cv(sl::Mat & input)
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
 * @brief Conversion function between sl::Mat and cv::Mat for depth maps.
 *
 * @param input Original frame-holding data structure.
 * @return OpenCV frame-holding data structure, pointing to the same data.
 */
cv::Mat ZEDDriverNode::sl_to_cv_depth(sl::Mat & input)
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
 * @brief Initializes a CameraInfo message structure.
 *
 * @param camera_info CameraInfo message to initialize.
 */
void ZEDDriverNode::init_camera_info(
  const sl::CameraParameters & zed_params,
  camera_info_manager::CameraInfo & info,
  const std::string & frame_id,
  double stereo_baseline)
{
  info.header.set__frame_id(frame_id);
  info.set__height(static_cast<uint32_t>(zed_params.image_size.height));
  info.set__width(static_cast<uint32_t>(zed_params.image_size.width));
  info.set__distortion_model("plumb_bob");
  info.d.reserve(5);
  for (int i = 0; i < 5; i++) {
    info.d.push_back(zed_params.disto[i]);
  }
  info.k[0] = zed_params.fx;
  info.k[2] = zed_params.cx;
  info.k[4] = zed_params.fy;
  info.k[5] = zed_params.cy;
  info.k[8] = 1.0;
  info.r[0] = 1.0;
  info.r[4] = 1.0;
  info.r[8] = 1.0;
  info.p[0] = zed_params.fx;
  info.p[2] = zed_params.cx;
  info.p[3] = stereo_baseline;
  info.p[5] = zed_params.fy;
  info.p[6] = zed_params.cy;
  info.p[10] = 1.0;
}

/**
 * @brief Validates the depth_mode parameter.
 *
 * @param p Parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool ZEDDriverNode::validate_depth_mode(const rclcpp::Parameter & p)
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
  } else if (depth_mode == "NONE") {
    depth_mode_ = sl::DEPTH_MODE::NONE;
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "ZEDDriverNode::validate_depth_mode: Invalid depth_mode parameter: %s",
      depth_mode.c_str());
    return false;
  }
  return true;
}

/**
 * @brief Validates the enable_tracking parameter.
 *
 * @param p Parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool ZEDDriverNode::validate_enable_tracking(const rclcpp::Parameter & p)
{
  if (running_.load(std::memory_order_acquire)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "ZEDDriverNode::validate_enable_tracking: Cannot change enable_tracking while the camera is active");
    return false;
  }

  enable_tracking_ = p.as_bool();
  return true;
}

/**
 * @brief Validates the fps parameter.
 *
 * @param p Parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool ZEDDriverNode::validate_fps(const rclcpp::Parameter & p)
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
        "ZEDDriverNode::validate_fps: Invalid fps parameter: %ld",
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
bool ZEDDriverNode::validate_resolution(const rclcpp::Parameter & p)
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
      "ZEDDriverNode::validate_resolution: Invalid resolution parameter: %s",
      resolution.c_str());
    return false;
  }
  return true;
}

/**
 * @brief Validates the streaming_codec parameter.
 *
 * @param p Parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool ZEDDriverNode::validate_streaming_codec(const rclcpp::Parameter & p)
{
  std::string codec = p.as_string();
  if (codec.empty()) {
    // This disables streaming
    return true;
  }

  if (codec != "H264" && codec != "H265") {
    RCLCPP_ERROR(
      this->get_logger(),
      "ZEDDriverNode::validate_streaming_codec: Invalid streaming_codec parameter: %s",
      codec.c_str());
    return false;
  }

  if (codec == "H264") {
    streaming_codec_ = sl::STREAMING_CODEC::H264;
  } else if (codec == "H265") {
    streaming_codec_ = sl::STREAMING_CODEC::H265;
  }

  return true;
}

/**
 * @brief Validates the svo_compression parameter.
 *
 * @param p Parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool ZEDDriverNode::validate_svo_compression(const rclcpp::Parameter & p)
{
  std::string compression = p.as_string();
  if (compression.empty()) {
    // The default might be ok, or SVO module might be disabled
    return true;
  }

  if (compression == "LOSSLESS") {
    svo_compression = sl::SVO_COMPRESSION_MODE::LOSSLESS;
  } else if (compression == "H264") {
    svo_compression = sl::SVO_COMPRESSION_MODE::H264;
  } else if (compression == "H264_LOSSLESS") {
    svo_compression = sl::SVO_COMPRESSION_MODE::H264_LOSSLESS;
  } else if (compression == "H265") {
    svo_compression = sl::SVO_COMPRESSION_MODE::H265;
  } else if (compression == "H265_LOSSLESS") {
    svo_compression = sl::SVO_COMPRESSION_MODE::H265_LOSSLESS;
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "ZEDDriverNode::validate_svo_compression: Invalid svo_compression parameter: %s",
      compression.c_str());
    return false;
  }

  return true;
}

} // namespace ZEDDriver
