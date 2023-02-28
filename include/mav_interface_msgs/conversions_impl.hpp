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
                                    float positionTolerance,
                                    float orientationTolerance) :
        position(position),
        orientation(orientation.normalized()),
        positionTolerance(positionTolerance),
        orientationTolerance(orientationTolerance)
{
}

inline FullStateStampedEigen::FullStateStampedEigen(const uint64_t timestampNanoSeconds,
                                                    const Eigen::Vector3d position,
                                                    const Eigen::Quaterniond& orientation,
                                                    const Eigen::Vector3d& linearVelocity,
                                                    const Eigen::Vector3d& linearAcceleration,
                                                    const Eigen::Vector3d& angularVelocity) :
        timestampNanoSeconds(timestampNanoSeconds),
        position(position),
        orientation(orientation.normalized()),
        linearVelocity(linearVelocity),
        linearAcceleration(linearAcceleration),
        angularVelocity(angularVelocity)
{
}

inline void WaypointEigen2Msg(const WaypointEigen& waypointEigen, Waypoint& waypointMsg)
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
    waypointMsg.positionTolerance = waypointEigen.positionTolerance;
    waypointMsg.orientationTolerance = waypointEigen.orientationTolerance;
}

inline void FullStateStampedEigen2Msg(const FullStateStampedEigen& fullStateStampedEigen,
                                      FullStateStamped& fullStateStampedMsg)
{
    // Timestamp
    fullStateStampedMsg.timestampNanoSeconds = fullStateStampedEigen.timestampNanoSeconds;

    // Copy position
    fullStateStampedMsg.position.x = fullStateStampedEigen.position.x();
    fullStateStampedMsg.position.y = fullStateStampedEigen.position.y();
    fullStateStampedMsg.position.z = fullStateStampedEigen.position.z();

    // Copy orientation
    fullStateStampedMsg.orientation.w = fullStateStampedEigen.orientation.w();
    fullStateStampedMsg.orientation.x = fullStateStampedEigen.orientation.x();
    fullStateStampedMsg.orientation.y = fullStateStampedEigen.orientation.y();
    fullStateStampedMsg.orientation.z = fullStateStampedEigen.orientation.z();

    // Copy linearVelocity
    fullStateStampedMsg.linearVelocity.x = fullStateStampedEigen.linearVelocity.x();
    fullStateStampedMsg.linearVelocity.y = fullStateStampedEigen.linearVelocity.y();
    fullStateStampedMsg.linearVelocity.z = fullStateStampedEigen.linearVelocity.z();

    // Copy linearAcceleration
    fullStateStampedMsg.linearAcceleration.x = fullStateStampedEigen.linearAcceleration.x();
    fullStateStampedMsg.linearAcceleration.y = fullStateStampedEigen.linearAcceleration.y();
    fullStateStampedMsg.linearAcceleration.z = fullStateStampedEigen.linearAcceleration.z();

    // Copy angularVelocity
    fullStateStampedMsg.angularVelocity.x = fullStateStampedEigen.angularVelocity.x();
    fullStateStampedMsg.angularVelocity.y = fullStateStampedEigen.angularVelocity.y();
    fullStateStampedMsg.angularVelocity.z = fullStateStampedEigen.angularVelocity.z();
}

} // namespace mav_interface_msgs

#endif
