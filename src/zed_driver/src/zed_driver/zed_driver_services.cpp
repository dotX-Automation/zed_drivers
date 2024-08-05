/**
 * ZED Driver node service callbacks.
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

namespace zed_drivers
{

/**
 * @brief Enable service callback.
 *
 * @param req Service request.
 * @param resp Service response.
 */
void ZEDDriverNode::enable_callback(
  const SetBool::Request::SharedPtr req,
  const SetBool::Response::SharedPtr resp)
{
  if (req->data) {
    bool expected = false;
    if (running_.compare_exchange_strong(
        expected,
        true,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      // Start camera sampling thread
      camera_thread_ = std::thread(
        &ZEDDriverNode::camera_routine,
        this);
    }
  } else {
    bool expected = true;
    if (running_.compare_exchange_strong(
        expected,
        false,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      camera_thread_.join();
      RCLCPP_INFO(this->get_logger(), "Camera sampling thread joined");
    }
  }
  resp->set__success(true);
  resp->set__message("");
}

} // namespace zed_drivers
