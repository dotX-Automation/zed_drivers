/**
 * ZED Driver standalone application.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 20, 2023
 */

#include <cstdlib>
#include <csignal>

#include <rclcpp/rclcpp.hpp>

#include <ros2_app_manager/ros2_app_manager.hpp>
#include <ros2_signal_handler/ros2_signal_handler.hpp>

#include <zed_driver/zed_driver.hpp>

using namespace DUAAppManagement;

int main(int argc, char ** argv)
{
  ROS2AppManager<rclcpp::executors::SingleThreadedExecutor,
    ZEDDriver::ZEDDriverNode> app_manager(
    argc,
    argv,
    "zed_driver_app");

  SignalHandler & sig_handler = SignalHandler::get_global_signal_handler();
  sig_handler.init(
    app_manager.get_context(),
    "zed_driver_app_signal_handler",
    app_manager.get_executor());
  sig_handler.install(SIGINT);
  sig_handler.install(SIGTERM);
  sig_handler.install(SIGQUIT);
  sig_handler.ignore(SIGHUP);
  sig_handler.ignore(SIGUSR1);
  sig_handler.ignore(SIGUSR2);

  app_manager.run();

  app_manager.shutdown();
  sig_handler.fini();

  exit(EXIT_SUCCESS);
}
