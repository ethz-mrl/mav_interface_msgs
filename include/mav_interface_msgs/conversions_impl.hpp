/*
 * SPDX-FileCopyrightText: 2020-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2023 Dimos Tzoumanikas
 * SPDX-FileCopyrightText: 2022-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MAV_INTERFACE_MSGS_CONVERSIONS_IMPL_HPP
#define MAV_INTERFACE_MSGS_CONVERSIONS_IMPL_HPP

namespace mav_interface_msgs {

inline WaypointEigen::WaypointEigen(const Eigen::Vector3d position,
                                    const Eigen::Quaterniond& orientation,
                                    float position_tolerance,
                                    float orientation_tolerance) :
        position(position),
        orientation(orientation.normalized()),
        position_tolerance(position_tolerance),
        orientation_tolerance(orientation_tolerance)
{
}

inline FullStateStampedEigen::FullStateStampedEigen(const uint64_t timestamp_nano_seconds,
                                                    const Eigen::Vector3d position,
                                                    const Eigen::Quaterniond& orientation,
                                                    const Eigen::Vector3d& linear_velocity,
                                                    const Eigen::Vector3d& linear_acceleration,
                                                    const Eigen::Vector3d& angular_velocity) :
        timestamp_nano_seconds(timestamp_nano_seconds),
        position(position),
        orientation(orientation.normalized()),
        linear_velocity(linear_velocity),
        linear_acceleration(linear_acceleration),
        angular_velocity(angular_velocity)
{
}

inline void WaypointEigen2Msg(const WaypointEigen& waypointEigen, mav_interface_msgs::msg::Waypoint& waypointMsg)
{
    // Copy position
    waypointMsg.position.x = waypointEigen.position.x();
    waypointMsg.position.y = waypointEigen.position.y();
    waypointMsg.position.z = waypointEigen.position.z();

    // Copy orientation
    waypointMsg.orientation.w = waypointEigen.orientation.w();
    waypointMsg.orientation.x = waypointEigen.orientation.x();
    waypointMsg.orientation.y = waypointEigen.orientation.y();
    waypointMsg.orientation.z = waypointEigen.orientation.z();

    // Copy position and orientation tolerance
    waypointMsg.position_tolerance = waypointEigen.position_tolerance;
    waypointMsg.orientation_tolerance = waypointEigen.orientation_tolerance;
}

inline void FullStateStampedEigen2Msg(const FullStateStampedEigen& fullStateStampedEigen,
                                      mav_interface_msgs::msg::FullStateStamped& fullStateStampedMsg)
{
    // Timestamp
    fullStateStampedMsg.timestamp_nano_seconds = fullStateStampedEigen.timestamp_nano_seconds;

    // Copy position
    fullStateStampedMsg.position.x = fullStateStampedEigen.position.x();
    fullStateStampedMsg.position.y = fullStateStampedEigen.position.y();
    fullStateStampedMsg.position.z = fullStateStampedEigen.position.z();

    // Copy orientation
    fullStateStampedMsg.orientation.w = fullStateStampedEigen.orientation.w();
    fullStateStampedMsg.orientation.x = fullStateStampedEigen.orientation.x();
    fullStateStampedMsg.orientation.y = fullStateStampedEigen.orientation.y();
    fullStateStampedMsg.orientation.z = fullStateStampedEigen.orientation.z();

    // Copy linear_velocity
    fullStateStampedMsg.linear_velocity.x = fullStateStampedEigen.linear_velocity.x();
    fullStateStampedMsg.linear_velocity.y = fullStateStampedEigen.linear_velocity.y();
    fullStateStampedMsg.linear_velocity.z = fullStateStampedEigen.linear_velocity.z();

    // Copy linear_acceleration
    fullStateStampedMsg.linear_acceleration.x = fullStateStampedEigen.linear_acceleration.x();
    fullStateStampedMsg.linear_acceleration.y = fullStateStampedEigen.linear_acceleration.y();
    fullStateStampedMsg.linear_acceleration.z = fullStateStampedEigen.linear_acceleration.z();

    // Copy angular_velocity
    fullStateStampedMsg.angular_velocity.x = fullStateStampedEigen.angular_velocity.x();
    fullStateStampedMsg.angular_velocity.y = fullStateStampedEigen.angular_velocity.y();
    fullStateStampedMsg.angular_velocity.z = fullStateStampedEigen.angular_velocity.z();
}

} // namespace mav_interface_msgs

#endif