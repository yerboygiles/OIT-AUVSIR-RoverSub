# Not working hydrophone that is still all in c++
# needs conversion to python as well as many other
# things

#  * The speed of sound in water is 1484 m/s.
import static
import vector3d

vector3d hydrophone_positions
static constexpr double speed_sound_in_water = 1484.0

# callback for incoming ping time deltas from the hydrophones
# principle of operation: JFM
# (see http://robosub.eecs.wsu.edu/wiki/cs/hydrophones/pinger_bearing/start
# for details)
def deltaCallback():

    vector3d bearing
    vector3d d
    vector3d time_deltas

    #define NS_IN_SEC 1000000000
    time_deltas[0] = msg->x_delta.sec + static_cast<double>(msg->x_delta.nanosec) / NS_IN_SEC
    time_deltas[1] = msg->y_delta.sec + static_cast<double>(msg->y_delta.nanosec) / NS_IN_SEC
    time_deltas[2] = msg->z_delta.sec + static_cast<double>(msg->z_delta.nanosec) / NS_IN_SEC

    d = time_deltas * speed_sound_in_water

    bearing = d.cwiseQuotient(hydrophone_positions)

    #geometry_msgs::msg::Vector3Stamped bearing_msg
    #robosub_msgs::msg::VectorSpherical spherical_msg

    bearing_msg.header.stamp = node->get_clock()->now()
    bearing_msg.vector.x = -bearing[0]
    # flip x and y, was spitting out incorrect values
    bearing_msg.vector.y = -bearing[1]

    bearing_msg.vector.z = bearing[2]

    double mag = std::sqrt(bearing[0] * bearing[0] +
                           bearing[1] * bearing[1] +
                           bearing[2] * bearing[2])
    double theta = atan2(bearing[1], bearing[0]) * 180.0 / M_PI
    double phi = atan2(sqrt(bearing[1] * bearing[1] + bearing[0] * bearing[0]),
                            bearing[2]) * 180.0 / M_PI

    spherical_msg.r = mag
    spherical_msg.theta = theta # yaw
    spherical_msg.phi = phi    # pitch

    RCLCPP_DEBUG(node->get_logger(), "vector magnitude: %lf", mag)