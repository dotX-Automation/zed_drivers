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

} // namespace ZEDMiniDriver
