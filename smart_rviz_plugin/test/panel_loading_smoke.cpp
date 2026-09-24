// Copyright 2026 smartmicro_ros2_radars contributors
// Licensed under the Apache License, Version 2.0.

#include <QApplication>
#include <iostream>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  unsigned loaded = 0;
  try {
    pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
    for (const auto & name : loader.getDeclaredClasses()) {
      if (name.rfind("smart_rviz_plugin/", 0) != 0) {
        continue;
      }
      // Destroy immediately, including when a panel's worker has not started.
      // The test timeout catches cancellation-before-spin shutdown deadlocks.
      {
        auto panel = loader.createSharedInstance(name);
        app.processEvents();
        std::cout << "Loaded: " << name << std::endl;
      }
      ++loaded;
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  std::cout << "Loaded and destroyed " << loaded << " panels" << std::endl;
  return loaded == 5 ? 0 : 1;
}
