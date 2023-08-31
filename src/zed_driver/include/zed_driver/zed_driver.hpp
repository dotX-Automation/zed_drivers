/**
 * ZED Driver node definition.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 20, 2023
 */

#ifndef ZED_DRIVERS__ZED_DRIVER_HPP
#define ZED_DRIVERS__ZED_DRIVER_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <errno.h>
#include <semaphore.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <sl/Camera.hpp>

#include <rclcpp/rclcpp.hpp>

#include <pose_kit/pose.hpp>
#include <pose_kit/kinematic_pose.hpp>

#include <dua_node/dua_node.hpp>
#include <dua_qos/dua_qos.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <theora_wrappers/publisher.hpp>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <dynamic_systems_control/control_lib.hpp>
#include <dynamic_systems_control/lti.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <dua_interfaces/msg/point_cloud2_with_roi.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <std_srvs/srv/set_bool.hpp>

using namespace dua_interfaces::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;
using namespace visualization_msgs::msg;

using namespace std_srvs::srv;

namespace ZEDDriver
{

/**
 * Drives a ZED camera.
 */
class ZEDDriverNode : public DUANode::NodeBase
{
public:
  ZEDDriverNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());
  virtual ~ZEDDriverNode();

private:
  /* Node initialization routines. */
  void init_parameters();
  void init_publishers();
  void init_services();
  void init_tf2();

  /* TF listeners, broadcaster, and related data. */
  std::string camera_name_;
  std::string camera_frame_;
  std::string camera_odom_frame_;
  std::string camera_left_frame_;
  std::string camera_right_frame_;
  std::string camera_imu_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /* Topic publishers. */
  rclcpp::Publisher<Odometry>::SharedPtr base_link_odom_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr base_link_pose_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr camera_odom_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr camera_pose_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_filtered_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<PointCloud2WithROI>::SharedPtr point_cloud_roi_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr rviz_base_link_odom_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr rviz_base_link_pose_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr rviz_camera_odom_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr rviz_camera_pose_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr rviz_point_cloud_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr rviz_point_cloud_roi_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr rviz_roi_pub_;

  /* image_transport, camera_info publishers. */
  std::shared_ptr<image_transport::Publisher> left_rect_pub_;
  std::shared_ptr<image_transport::Publisher> right_rect_pub_;
  std::shared_ptr<image_transport::Publisher> left_rect_sd_pub_;
  std::shared_ptr<image_transport::Publisher> right_rect_sd_pub_;
  std::shared_ptr<TheoraWrappers::Publisher> left_stream_pub_;
  std::shared_ptr<TheoraWrappers::Publisher> right_stream_pub_;
  std::shared_ptr<image_transport::Publisher> depth_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr right_info_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr left_sd_info_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr right_sd_info_pub_;

  /* Service servers. */
  rclcpp::Service<SetBool>::SharedPtr enable_srv_;

  /* Service callbacks. */
  void enable_callback(
    const SetBool::Request::SharedPtr req,
    const SetBool::Response::SharedPtr resp);

  /* Internal state and data. */
  sl::Camera zed_;
  sl::MODEL camera_model_;
  std::atomic<bool> running_{false};
  camera_info_manager::CameraInfo left_info_{};
  camera_info_manager::CameraInfo left_sd_info_{};
  camera_info_manager::CameraInfo right_info_{};
  camera_info_manager::CameraInfo right_sd_info_{};
  rclcpp::Clock system_clock_ = rclcpp::Clock(RCL_SYSTEM_TIME);

  /* Camera sampling thread and routine. */
  std::thread camera_thread_;
  void camera_routine();

  /* Depth processing thread, routine, and buffers. */
  std::thread depth_thread_;
  void depth_routine();
  sem_t depth_sem_1_;
  sem_t depth_sem_2_;
  sl::Mat depth_map_view_;
  sl::Mat depth_point_cloud_;
  rclcpp::Time last_depth_ts_;

  /* RGB processing thread, routine, and buffers. */
  std::thread rgb_thread_;
  void rgb_routine();
  rclcpp::Time curr_rgb_ts_{};
  sem_t rgb_sem_1_;
  sem_t rgb_sem_2_;
  cv::Mat left_frame_cv_, right_frame_cv_;
  cv::Mat left_frame_sd_cv_, right_frame_sd_cv_;

  /* IMU filters and data. */
  std::array<DynamicSystems::Control::LTISystem, 3> gyro_filters_;
  std::array<DynamicSystems::Control::LTISystem, 3> accel_filters_;

  /* Node parameters. */
  bool autostart_;
  std::string base_link_name_ = "";
  int64_t confidence_ = 50;
  sl::DEPTH_MODE depth_mode_ = sl::DEPTH_MODE::QUALITY;
  int64_t depth_rate_ = 0;
  std::vector<int64_t> depth_resolution_ = {0, 0};
  bool enable_tracking_ = false;
  int fps_ = 15;
  std::string global_frame_ = "";
  double imu_filters_sampling_time_ = 0.0;
  int64_t imu_filters_zoh_steps_ = 0;
  int64_t imu_filters_order_ = 0;
  std::vector<double> imu_filters_low_freqs_ = {0.0, 0.0};
  std::vector<double> imu_filters_high_freqs_ = {0.0, 0.0};
  std::string link_namespace_ = "";
  std::string odom_frame_ = "";
  bool publish_tf_ = false;
  std::string record_path_ = "";
  sl::RESOLUTION resolution_ = sl::RESOLUTION::HD720;
  std::vector<double> roi_box_sizes_;
  std::vector<int64_t> sd_resolution_ = {0, 0};
  bool stream_hd_ = false;
  sl::STREAMING_CODEC streaming_codec_ = sl::STREAMING_CODEC::H264;
  int64_t texture_confidence_ = 100;
  bool verbose_ = false;
  int64_t video_rate_ = 0;

  /* Node parameters validators. */
  bool validate_depth_mode(const rclcpp::Parameter & p);
  bool validate_enable_tracking(const rclcpp::Parameter & p);
  bool validate_fps(const rclcpp::Parameter & p);
  bool validate_resolution(const rclcpp::Parameter & p);
  bool validate_streaming_codec(const rclcpp::Parameter & p);

  /* Auxiliary routines. */
  bool open_camera();
  void close_camera();
  void init_camera_info(
    const sl::CameraParameters & zed_params,
    camera_info_manager::CameraInfo & info,
    const std::string & frame_id,
    double stereo_baseline = 0.0);
  void positional_tracking(sl::Pose & camera_pose);
  void sensor_sampling(sl::SensorsData & sensors_data);
  cv::Mat sl_to_cv(sl::Mat & input);
  cv::Mat sl_to_cv_depth(sl::Mat & input);
  Image::SharedPtr frame_to_msg(cv::Mat & frame);
};

} // namespace ZEDDriver

#endif // ZED_DRIVERS__ZED_DRIVER_HPP
