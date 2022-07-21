# Not working hydrophone that is still all in c++
# needs conversion to python as well as many other
# things

# /*
#  * The speed of sound in water is 1484 m/s.
#  */
# static constexpr double speed_sound_in_water = 1484.0;
#
#
# // callback for incoming ping time deltas from the hydrophones
# // principle of operation: JFM
# // (see http://robosub.eecs.wsu.edu/wiki/cs/hydrophones/pinger_bearing/start
# // for details)
# void deltaCallback(
#                 const robosub_msgs::msg::HydrophoneDeltas::ConstSharedPtr msg)
# {
#     Vector3d bearing, d, time_deltas;
#
#     #define NS_IN_SEC 1000000000
#     time_deltas[0] = msg->x_delta.sec +
#                      static_cast<double>(msg->x_delta.nanosec) / NS_IN_SEC;
#     time_deltas[1] = msg->y_delta.sec +
#                      static_cast<double>(msg->y_delta.nanosec) / NS_IN_SEC;
#     time_deltas[2] = msg->z_delta.sec +
#                      static_cast<double>(msg->z_delta.nanosec) / NS_IN_SEC;
#
#     d = time_deltas * speed_sound_in_water;
#
#     bearing = d.cwiseQuotient(hydrophone_positions);
#
#     geometry_msgs::msg::Vector3Stamped bearing_msg;
#     robosub_msgs::msg::VectorSpherical spherical_msg;
#
#     bearing_msg.header.stamp = node->get_clock()->now();
#     bearing_msg.vector.x = -bearing[0];
#     // flip x and y, was spitting out incorrect values
#     bearing_msg.vector.y = -bearing[1];
#
#     bearing_msg.vector.z = bearing[2];
#
#     double mag = std::sqrt(bearing[0] * bearing[0] +
#                            bearing[1] * bearing[1] +
#                            bearing[2] * bearing[2]);
#     double theta = atan2(bearing[1], bearing[0]) * 180.0 / M_PI;
#     double phi = atan2(sqrt(bearing[1] * bearing[1] + bearing[0] * bearing[0]),
#                             bearing[2]) * 180.0 / M_PI;
#
#     spherical_msg.r = mag;
#     spherical_msg.theta = theta;// yaw
#     spherical_msg.phi = phi;    // pitch
#
#     RCLCPP_DEBUG(node->get_logger(), "vector magnitude: %lf", mag);
#
#     visualization_msgs::msg::Marker marker;
#     marker.header.frame_id = "cobalt";
#     marker.header.stamp = node->get_clock()->now();
#     marker.ns = "pinger_bearing";
#     marker.id = 0;
#     marker.type = visualization_msgs::msg::Marker::ARROW;
#     marker.action = visualization_msgs::msg::Marker::ADD;
#     geometry_msgs::msg::Point tail, tip;
#     tail.x = tail.y = tail.z = 0;
#     tip.x = bearing[0];
#     tip.y = bearing[1];
#     tip.z = bearing[2];
#     marker.points.push_back(tail);
#     marker.points.push_back(tip);
#     marker.color.a = 1.0;
#     marker.color.r = 0.0;
#     marker.color.g = 1.0;
#     marker.color.b = 0.0;
#     marker.scale.x = 0.1;
#     marker.scale.y = 0.2;
#
#     bearing_pub->publish(bearing_msg);
#     marker_pub->publish(marker);
#     spherical_pub->publish(spherical_msg);
# }