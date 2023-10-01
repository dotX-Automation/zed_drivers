/**
 * Depth data processing routines.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * August 29, 2023
 */

#include <zed_driver/zed_driver.hpp>

namespace ZEDDriver
{

/**
 * @brief Processes depth data.
 */
void ZEDDriverNode::depth_routine()
{
  RCLCPP_INFO(this->get_logger(), "Depth processing thread started");

  while (true) {
    // Wait for depth data
    sem_wait(&depth_sem_2_);

    // Check if the thread should stop
    if (!running_.load(std::memory_order_acquire)) {
      break;
    }

    // Publish depth map image
    cv::Mat depth_map_view_cv = sl_to_cv_depth(depth_map_view_);
    Image::SharedPtr depth_msg = frame_to_msg(depth_map_view_cv);
    depth_msg->header.set__frame_id(camera_frame_);
    depth_msg->header.stamp.set__sec(static_cast<int32_t>(depth_map_view_.timestamp.getSeconds()));
    depth_msg->header.stamp.set__nanosec(
      static_cast<uint32_t>(depth_map_view_.timestamp.getNanoseconds() % uint64_t(1e9)));
    depth_msg->set__encoding(sensor_msgs::image_encodings::BGRA8);
    depth_pub_->publish(*depth_msg);

    // Get ROI box sizes and corner points (in camera frame)
    Eigen::Vector3f p0(
      0.0f,
      static_cast<float>(-roi_box_sizes_[1] / 2.0),
      static_cast<float>(-roi_box_sizes_[2] / 2.0));
    Eigen::Vector3f p1(
      static_cast<float>(roi_box_sizes_[0]),
      static_cast<float>(-roi_box_sizes_[1] / 2.0),
      static_cast<float>(-roi_box_sizes_[2] / 2.0));
    Eigen::Vector3f p2(
      0.0f,
      static_cast<float>(roi_box_sizes_[1] / 2.0),
      static_cast<float>(-roi_box_sizes_[2] / 2.0));
    Eigen::Vector3f p3(
      0.0f,
      static_cast<float>(-roi_box_sizes_[1] / 2.0),
      static_cast<float>(roi_box_sizes_[2] / 2.0));

    // Compute ROI evaluation vectors (in camera frame)
    Eigen::Vector3f ic = p1 - p0;
    Eigen::Vector3f jc = p2 - p0;
    Eigen::Vector3f kc = p3 - p0;
    float iti = ic.transpose() * ic;
    float jtj = jc.transpose() * jc;
    float ktk = kc.transpose() * kc;

    // Compute ROI transformation matrix
    uint32_t pc_length =
      static_cast<uint32_t>(depth_point_cloud_.getWidth() * depth_point_cloud_.getHeight());
    Eigen::MatrixXf roi_transform = Eigen::MatrixXf::Zero(6, 6);
    roi_transform.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
    roi_transform.block<1, 3>(3, 3) = ic.transpose();
    roi_transform.block<1, 3>(4, 3) = jc.transpose();
    roi_transform.block<1, 3>(5, 3) = kc.transpose();

    // Fill and publish point cloud messages
    PointCloud2 pc_msg{}, pc_roi_msg{};
    PointCloud2WithROI pc_with_roi_msg{};
    std::array<Eigen::Vector3f, 4> roi_corners = {p0, p1, p2, p3};
    pc_with_roi_msg.set__culled(true);
    pc_with_roi_msg.roi.set__type(dua_interfaces::msg::RegionOfInterest::BOX);
    for (int i = 0; i < 4; ++i) {
      pc_with_roi_msg.roi.box_corners[i].header.set__frame_id(camera_left_frame_);
      pc_with_roi_msg.roi.box_corners[i].header.stamp.set__sec(
        static_cast<int32_t>(depth_point_cloud_.timestamp.getSeconds()));
      pc_with_roi_msg.roi.box_corners[i].header.stamp.set__nanosec(
        static_cast<uint32_t>(depth_point_cloud_.timestamp.getNanoseconds() % uint64_t(1e9)));

      pc_with_roi_msg.roi.box_corners[i].point.set__x(roi_corners[i].x());
      pc_with_roi_msg.roi.box_corners[i].point.set__y(roi_corners[i].y());
      pc_with_roi_msg.roi.box_corners[i].point.set__z(roi_corners[i].z());
    }
    sensor_msgs::PointCloud2Modifier pc_modifier(pc_msg);
    sensor_msgs::PointCloud2Modifier pc_roi_modifier(pc_roi_msg);
    pc_msg.header.set__frame_id(camera_left_frame_);
    pc_roi_msg.header.set__frame_id(camera_left_frame_);
    pc_msg.header.stamp.set__sec(static_cast<int32_t>(depth_point_cloud_.timestamp.getSeconds()));
    pc_msg.header.stamp.set__nanosec(
      static_cast<uint32_t>(depth_point_cloud_.timestamp.getNanoseconds() % uint64_t(1e9)));
    pc_roi_msg.header.stamp.set__sec(
      static_cast<int32_t>(depth_point_cloud_.timestamp.getSeconds()));
    pc_roi_msg.header.stamp.set__nanosec(
      static_cast<uint32_t>(depth_point_cloud_.timestamp.getNanoseconds() % uint64_t(1e9)));
    pc_msg.set__height(1);
    pc_roi_msg.set__height(1);
    pc_msg.set__width(pc_length);
    pc_roi_msg.set__width(pc_length);
    pc_msg.set__is_bigendian(false);
    pc_roi_msg.set__is_bigendian(false);
    pc_msg.set__is_dense(true);
    pc_roi_msg.set__is_dense(true);
    pc_modifier.setPointCloud2Fields(
      4,
      "x", 1, PointField::FLOAT32,
      "y", 1, PointField::FLOAT32,
      "z", 1, PointField::FLOAT32,
      "rgba", 1, PointField::FLOAT32);
    pc_roi_modifier.setPointCloud2Fields(
      4,
      "x", 1, PointField::FLOAT32,
      "y", 1, PointField::FLOAT32,
      "z", 1, PointField::FLOAT32,
      "rgba", 1, PointField::FLOAT32);
    sensor_msgs::PointCloud2Iterator<float> iter_pc_x(pc_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_pc_y(pc_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_pc_z(pc_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_pc_rgba(pc_msg, "rgba");
    sensor_msgs::PointCloud2Iterator<float> iter_pc_roi_x(pc_roi_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_pc_roi_y(pc_roi_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_pc_roi_z(pc_roi_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_pc_roi_rgba(pc_roi_msg, "rgba");

    // Fill point cloud matrix and color vector
    Eigen::MatrixXf pc_mat = Eigen::MatrixXf::Zero(6, pc_length);
    std::vector<float> pc_colors(pc_length);
    uint32_t mat_col_idx = 0;
    for (uint64_t i = 0; i < depth_point_cloud_.getHeight(); ++i) {
      for (uint64_t j = 0; j < depth_point_cloud_.getWidth(); ++j) {
        // Extract point position w.r.t. the camera from ZED data
        sl::float4 point3D;
        if (depth_point_cloud_.getValue(j, i, &point3D) == sl::ERROR_CODE::FAILURE) {
          continue;
        }
        if (std::isnan(point3D.x) || std::isnan(point3D.y) || std::isnan(point3D.z)) {
          continue;
        }
        Eigen::Vector3f Qc(
          point3D.x,
          point3D.y,
          point3D.z);
        Eigen::Vector3f qc = Qc - p0;
        pc_mat.block<3, 1>(0, mat_col_idx) = Qc;
        pc_mat.block<3, 1>(3, mat_col_idx) = qc;
        pc_colors[mat_col_idx] = point3D.w;

        mat_col_idx++;
      }
    }
    uint32_t valid_points = mat_col_idx;
    pc_colors.resize(valid_points);
    pc_mat.resize(6, valid_points);

    // Compute ROI from whole point cloud
    // This instruction makes use of all the Eigen magic, and its performance impact depends
    // on the entity of the point cloud.
    pc_mat = roi_transform * pc_mat;

    // Fill point cloud messages
    uint32_t pc_roi_length = 0;
    for (uint32_t i = 0; i < valid_points; ++i) {
      // Fill complete point cloud message
      *iter_pc_x = pc_mat(0, i);
      *iter_pc_y = pc_mat(1, i);
      *iter_pc_z = pc_mat(2, i);
      *iter_pc_rgba = pc_colors[i];

      // Check if point is in ROI, and add it in case
      float itq = pc_mat(4, i);
      float jtq = pc_mat(5, i);
      float ktq = pc_mat(6, i);
      if ((0 < itq) && (itq < iti) &&
        (0 < jtq) && (jtq < jtj) &&
        (0 < ktq) && (ktq < ktk))
      {
        *iter_pc_roi_x = pc_mat(0, i);
        *iter_pc_roi_y = pc_mat(1, i);
        *iter_pc_roi_z = pc_mat(2, i);
        *iter_pc_roi_rgba = pc_colors[i];

        pc_roi_length++;

        // Advance iterators
        ++iter_pc_roi_x;
        ++iter_pc_roi_y;
        ++iter_pc_roi_z;
        ++iter_pc_roi_rgba;
      }

      // Advance iterators
      ++iter_pc_x;
      ++iter_pc_y;
      ++iter_pc_z;
      ++iter_pc_rgba;
    }
    pc_modifier.resize(valid_points);
    pc_roi_modifier.resize(pc_roi_length);
    pc_with_roi_msg.cloud = pc_roi_msg;

    point_cloud_pub_->publish(pc_msg);
    point_cloud_roi_pub_->publish(pc_with_roi_msg);
    rviz_point_cloud_pub_->publish(pc_msg);
    rviz_point_cloud_roi_pub_->publish(pc_roi_msg);

    // Compute ROI box center (in camera frame)
    Eigen::Isometry3f roi_center_iso = Eigen::Isometry3f::Identity();
    roi_center_iso.pretranslate(Eigen::Vector3f(roi_box_sizes_[0] / 2.0f, 0.0f, 0.0f));
    Eigen::Quaternionf roi_center_orientation(roi_center_iso.rotation());

    // Publish the current ROI for visualization
    Marker cleanup_marker{}, roi_marker{};
    MarkerArray roi_markers{};
    cleanup_marker.header.set__frame_id(camera_left_frame_);
    cleanup_marker.header.stamp = pc_msg.header.stamp;
    cleanup_marker.set__action(Marker::DELETEALL);
    roi_marker.header.set__frame_id(camera_left_frame_);
    roi_marker.header.stamp = pc_msg.header.stamp;
    roi_marker.set__ns(this->get_fully_qualified_name());
    roi_marker.set__id(0);
    roi_marker.set__type(Marker::CUBE);
    roi_marker.set__action(Marker::ADD);
    roi_marker.pose.position.set__x(roi_center_iso.translation().x());
    roi_marker.pose.position.set__y(roi_center_iso.translation().y());
    roi_marker.pose.position.set__z(roi_center_iso.translation().z());
    roi_marker.pose.orientation.set__w(roi_center_orientation.w());
    roi_marker.pose.orientation.set__x(roi_center_orientation.x());
    roi_marker.pose.orientation.set__y(roi_center_orientation.y());
    roi_marker.pose.orientation.set__z(roi_center_orientation.z());
    roi_marker.scale.set__x(roi_box_sizes_[0]);
    roi_marker.scale.set__y(roi_box_sizes_[1]);
    roi_marker.scale.set__z(roi_box_sizes_[2]);
    roi_marker.color.set__r(1.0f);
    roi_marker.color.set__g(1.0f);
    roi_marker.color.set__b(0.0f);
    roi_marker.color.set__a(0.1f);
    roi_markers.markers.push_back(cleanup_marker);
    roi_markers.markers.push_back(roi_marker);
    rviz_roi_pub_->publish(roi_markers);

    sem_post(&depth_sem_1_);
  }
}

} // namespace ZEDDriver
