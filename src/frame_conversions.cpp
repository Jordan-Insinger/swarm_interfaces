#include "swarm_interfaces/frame_conversions.hpp"

namespace swarm_interfaces::frame_conversions {

GeoPose apark_to_global(const geometry_msgs::msg::Pose &apark_pose, double altitude_amsl) {

    q_apark_to_utm.setRPY(0, 0, -origin_r);

    //Autonomy park setpoint coordinates
    double sp_x = apark_pose.position.x;
    double sp_y = apark_pose.position.y;
    
    //Un-rotate setpoint coordinates
    double dx = cos(origin_r)*sp_x + sin(origin_r)*sp_y;
    double dy = -sin(origin_r)*sp_x + cos(origin_r)*sp_y;

    //Convert park coordinates to UTM
    geodesy::UTMPoint utm_pos;
    utm_pos.zone = utm_zone;
    utm_pos.band = utm_band;
    utm_pos.easting = dx + origin_x;
    utm_pos.northing = dy + origin_y;

    //Convert UTM easting/northing to lat/long
    geographic_msgs::msg::GeoPoint global_pos = geodesy::toMsg(utm_pos);

    //IMPORTANT: Command altitude is AMSL! (feedback is WGS-84 ellipsoid)
    global_pos.altitude = altitude_amsl;

    //Finally, compute global orientation
    tf2::Quaternion q_utm, q_apark;
    tf2::fromMsg(apark_pose.orientation, q_apark);
    q_utm = q_apark_to_utm*q_apark;
    
    geographic_msgs::msg::GeoPose global_pose;
    global_pose.position = global_pos;
    global_pose.orientation = tf2::toMsg(q_utm);

    return global_pose;

}

Twist apark_to_enu(const Twist& vel) {

    Twist vel_enu;
    double cos_origin = cos(origin_r);
    double sin_origin = sin(origin_r);

    vel_enu.linear.x = cos_origin*vel.linear.x + sin_origin*vel.linear.y;
    vel_enu.linear.y = -sin_origin*vel.linear.x + cos_origin*vel.linear.y;
    vel_enu.linear.z = vel.linear.z;
    vel_enu.angular.z = vel.angular.z;

    return vel_enu;
}

double quat_to_yaw(const Quaternion& quat) {

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;

}

} // namespace swarm_interfaces::frame_conversions
