/**
 * ZED 2i Driver node definition.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 20, 2023
 */

#ifndef ZED_DRIVERS__ZED_2I_DRIVER_HPP
#define ZED_DRIVERS__ZED_2I_DRIVER_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <semaphore.h>
#include <time.h>

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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <dynamic_systems_control/control_lib.hpp>
#include <dynamic_systems_control/lti.hpp>

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

namespace ZED2iDriver
{

/**
 * Drives a ZED 2i camera.
 */
class ZED2iDriverNode : public DUANode::NodeBase
{
public:
  ZED2iDriverNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());
  virtual ~ZED2iDriverNode();

private:
  /* Node initialization routines. */
  void init_atomics();
  void init_sync_primitives();
  void init_parameters();
  void init_publishers();
  void init_services();
  void init_tf_listeners();

  /* TF listeners, timer, and related data. */
  std::string map_frame_;
  std::string odom_frame_;
  std::string zed2i_odom_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex tf_lock_;
  TransformStamped odom_to_camera_odom_{};
  TransformStamped base_link_to_camera_{};
  TransformStamped map_to_camera_odom_{};
  rclcpp::TimerBase::SharedPtr tf_timer_;
  void tf_timer_callback();

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

  /* image_transport publishers. */
  std::shared_ptr<image_transport::CameraPublisher> left_rect_pub_;
  std::shared_ptr<image_transport::CameraPublisher> right_rect_pub_;
  std::shared_ptr<image_transport::CameraPublisher> left_rect_sd_pub_;
  std::shared_ptr<image_transport::CameraPublisher> right_rect_sd_pub_;
  std::shared_ptr<TheoraWrappers::Publisher> left_stream_pub_;
  std::shared_ptr<TheoraWrappers::Publisher> right_stream_pub_;
  std::shared_ptr<image_transport::Publisher> depth_pub_;

  /* Service servers. */
  rclcpp::Service<SetBool>::SharedPtr enable_srv_;

  /* Service callbacks. */
  void enable_callback(
    const SetBool::Request::SharedPtr req,
    const SetBool::Response::SharedPtr resp);

  /* Internal state and data. */
  sl::Camera zed_;
  std::atomic<bool> running_;
  camera_info_manager::CameraInfo left_info_{};
  camera_info_manager::CameraInfo left_sd_info_{};
  camera_info_manager::CameraInfo right_info_{};
  camera_info_manager::CameraInfo right_sd_info_{};

  /* Undersampling stopwatches. */
  rclcpp::Time last_video_ts_;
  rclcpp::Time last_depth_ts_;

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
  PoseKit::Pose depth_curr_pose_;

  /* IMU filters and data. */
  std::array<DynamicSystems::Control::LTISystem, 3> gyro_filters_;
  std::array<DynamicSystems::Control::LTISystem, 3> accel_filters_;

  /* Node parameters. */
  bool autostart_;
  int64_t confidence_ = 50;
  bool delayed_tracking_ = false;
  sl::DEPTH_MODE depth_mode_ = sl::DEPTH_MODE::QUALITY;
  int64_t depth_rate_ = 0;
  std::vector<int64_t> depth_resolution_ = {0, 0};
  int fps_ = 15;
  double imu_filters_sampling_time_ = 0.0;
  int64_t imu_filters_zoh_steps_ = 0;
  int64_t imu_filters_order_ = 0;
  std::vector<double> imu_filters_low_freqs_ = {0.0, 0.0};
  std::vector<double> imu_filters_high_freqs_ = {0.0, 0.0};
  std::string link_namespace_ = "";
  std::string record_path_ = "";
  sl::RESOLUTION resolution_ = sl::RESOLUTION::HD720;
  std::vector<double> roi_box_sizes_;
  std::vector<int64_t> sd_resolution_ = {0, 0};
  bool stream_hd_ = false;
  int64_t texture_confidence_ = 100;
  bool verbose_ = false;
  int64_t video_rate_ = 0;

  /* Node parameters validators. */
  bool validate_depth_mode(const rclcpp::Parameter & p);
  bool validate_fps(const rclcpp::Parameter & p);
  bool validate_resolution(const rclcpp::Parameter & p);

  /* Auxiliary routines. */
  bool open_camera();
  void close_camera();
  void init_camera_info(
    const sl::CameraParameters & zed_params,
    camera_info_manager::CameraInfo & info,
    std::string && frame_id,
    double stereo_baseline = 0.0);
  PoseKit::Pose positional_tracking(sl::Pose & camera_pose);
  void sensor_sampling(sl::SensorsData & sensors_data);
  cv::Mat slMat2cvMat(sl::Mat & input);
  cv::Mat slMat2cvMatDepth(sl::Mat & input);
  Image::SharedPtr frame_to_msg(cv::Mat & frame);
};

} // namespace ZED2iDriver

#endif // ZED_DRIVERS__ZED_2I_DRIVER_HPP
