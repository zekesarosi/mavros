/*
 * Copyright 2026 Zeke Sarosi <zeke.sarosi@gmail.com>.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief EKF status report plugin
 * @file ekf_status.cpp
 * @author Zeke Sarosi <zeke.sarosi@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <string>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/ekf_status_report.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief EKF status report plugin
 * @plugin ekf_status
 *
 * APM specific plugin. Publishes EKF_STATUS_REPORT (id 193) flags and
 * variances from the ArduPilot EKF to ROS.
 */
class EKFStatusPlugin : public plugin::Plugin
{
public:
  explicit EKFStatusPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "ekf_status")
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "frame_id", "base_link", [&](const rclcpp::Parameter & p) {
        frame_id = p.as_string();
      });

    ekf_status_pub = node->create_publisher<mavros_msgs::msg::EKFStatusReport>(
      "~/status", 10);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&EKFStatusPlugin::handle_ekf_status_report)
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::EKFStatusReport>::SharedPtr ekf_status_pub;

  std::string frame_id;

  void handle_ekf_status_report(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::ardupilotmega::msg::EKF_STATUS_REPORT & report,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto out = mavros_msgs::msg::EKFStatusReport();

    out.header.stamp = node->now();
    out.header.frame_id = frame_id;

    out.flags = report.flags;
    out.velocity_variance = report.velocity_variance;
    out.pos_horiz_variance = report.pos_horiz_variance;
    out.pos_vert_variance = report.pos_vert_variance;
    out.compass_variance = report.compass_variance;
    out.terrain_alt_variance = report.terrain_alt_variance;
    out.airspeed_variance = report.airspeed_variance;

    ekf_status_pub->publish(out);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::EKFStatusPlugin)
