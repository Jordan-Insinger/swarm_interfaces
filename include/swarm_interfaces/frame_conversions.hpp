#include <cmath>
#include <geodesy/utm.h>
#include <GeographicLib/Geoid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using GeoPose = geographic_msgs::msg::GeoPose;
using Pose = geometry_msgs::msg::Pose;

static double origin_x = 368305.0700699703;
static double origin_y = 3278357.100811797;
static double origin_z = 26.90878;
static double origin_r = 0.6021965548550632;
static double utm_zone = 17;
static char utm_band = 'R';

tf2::Quaternion q_apark_to_utm;

namespace swarm_interfaces::frame_conversions {

    GeoPose apark_to_global(const Pose &apark_pose, double altitude_amsl);

} // namespace swarm_interfaces::frame_conversions

