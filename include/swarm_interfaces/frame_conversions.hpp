#include <cmath>
#include <geodesy/utm.h>
#include <GeographicLib/Geoid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using GeoPose = geographic_msgs::msg::GeoPose;
using Pose = geometry_msgs::msg::Pose;
using Twist = geometry_msgs::msg::Twist;
using Quaternion = geometry_msgs::msg::Quaternion;

constexpr double origin_x = 368305.0700699703;
constexpr double origin_y = 3278357.100811797;
constexpr double origin_z = 26.90878;
constexpr double origin_r = 0.6021965548550632;
constexpr double utm_zone = 17;
constexpr char utm_band = 'R';

inline tf2::Quaternion q_apark_to_utm = []() {
    tf2::Quaternion q;
    q.setRPY(0, 0, -origin_r);
    return q;
}();

namespace swarm_interfaces::frame_conversions {

    GeoPose apark_to_global(const Pose& apark_pose, double altitude_amsl);

    Twist apark_to_enu(const Twist &vel);

    double quat_to_yaw(const Quaternion& quat);

} // namespace swarm_interfaces::frame_conversions

