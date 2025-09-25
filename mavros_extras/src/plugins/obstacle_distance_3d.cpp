/**
 * @brief Obstacle Distance 3D plugin
 * @file obstacle_3d.cpp
 * @author Zeke Sarosi
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2025 Zeke Sarosi.
 *
 * This file is part of the MAVROS package and subject to the license terms
 * in the top-level LICENSE file of the MAVROS repository.
 */

 #include <cmath> // For std::isnan

 #include "mavros/mavros_uas.hpp"
 #include "mavros/plugin.hpp"
 #include "mavros/plugin_filter.hpp"
 
 #include "custom_mavros_msgs/msg/obstacle3_d.hpp" // Our custom message
 
 namespace mavros
 {
 namespace extra_plugins
 {
 using namespace std::placeholders;
 
 /**
  * @brief Plugin to handle sending OBSTACLE_DISTANCE_3D MAVLink messages.
  * @plugin obstacle_distance_3d
  *
  * Subscribes to a custom_mavros_msgs/Obstacle3D message and sends the
  * data to the flight controller to report the position of a single obstacle.
  */
 class Obstacle3DPlugin : public plugin::Plugin
 {
 public:
     explicit Obstacle3DPlugin(plugin::UASPtr uas_)
     : Plugin(uas_, "obstacle_distance_3d")
     {
         enable_node_watch_parameters();
 
         // Add a configurable parameter for the MAVLink frame, just like the reference.
         node_declare_and_watch_parameter(
             "mav_frame", "LOCAL_NED", [&](const rclcpp::Parameter & p) {
                 auto mav_frame_str = p.as_string();
                 frame = utils::mav_frame_from_str(mav_frame_str);
             });
 
         obstacle_sub_ = node->create_subscription<custom_mavros_msgs::msg::Obstacle3D>(
             "~/send", 10, std::bind(&Obstacle3DPlugin::obstacle_cb, this, _1));
     }
 
     Subscriptions get_subscriptions() override
     {
         return { /* Rx disabled */};
     }
 
 private:
     rclcpp::Subscription<custom_mavros_msgs::msg::Obstacle3D>::SharedPtr obstacle_sub_;
 
     mavlink::common::MAV_FRAME frame;
 
     /**
      * @brief Callback for incoming obstacle data. Converts and sends to FCU.
      *
      * Message specification: https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE_3D
      * @param msg  The received Obstacle3D message.
      */
     void obstacle_cb(const custom_mavros_msgs::msg::Obstacle3D::SharedPtr msg)
     {
         // --- Data Validation ---
         // Check for invalid floating point numbers before sending.
         if (std::isnan(msg->x) || std::isnan(msg->y) || std::isnan(msg->z)) {
             RCLCPP_WARN(get_logger(), "obstacle_3d: Rejecting message with NaN coordinates.");
             return;
         }
 
         mavlink::ardupilotmega::msg::OBSTACLE_DISTANCE_3D obstacle{};
 
         // --- Field Mapping ---
         obstacle.sensor_type = msg->sensor_type;
         obstacle.obstacle_id = msg->obstacle_id;
         obstacle.x = msg->x;
         obstacle.y = msg->y;
         obstacle.z = msg->z;
         obstacle.min_distance = msg->min_distance;
         obstacle.max_distance = msg->max_distance;
 
         // Use the frame from the parameter, but allow the message to override it
         // This logic can be adjusted based on desired behavior.
         if (msg->frame > 0) {
             obstacle.frame = msg->frame;
         } else {
             obstacle.frame = utils::enum_value(frame);
         }
 
        RCLCPP_INFO(
            get_logger(),
            "OBSTACLE_3D: id: %u, frame: %u, x: %.3f, y: %.3f, z: %.3f",
            obstacle.obstacle_id, obstacle.frame, obstacle.x, obstacle.y, obstacle.z);
        uas->send_message(obstacle);
     }
 };
 
 }	// namespace extra_plugins
 }	// namespace mavros
 
 #include <mavros/mavros_plugin_register_macro.hpp>
 MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::Obstacle3DPlugin)
 