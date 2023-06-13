/**
 * Worker threads of the ZED Mini driver node.
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
 * @brief Listens for incoming TFs.
 */
void ZEDMiniDriverNode::tf_thread_routine()
{
  // Initialize TF buffers
  // TODO

  // Start listening
  // TODO Nonzero timeouts, so you can evaluate exit condition
}

} // namespace ZEDMiniDriver
