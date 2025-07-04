/*
 * SPDX-FileCopyrightText: 2020-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2023 Dimos Tzoumanikas
 * SPDX-FileCopyrightText: 2022-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MAV_INTERFACE_MSGS_CONVERSIONS_HPP
#define MAV_INTERFACE_MSGS_CONVERSIONS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mav_interface_msgs/msg/full_state_stamped.hpp>
#include <mav_interface_msgs/msg/full_state_trajectory.hpp>
#include <mav_interface_msgs/msg/path.hpp>
#include <mav_interface_msgs/msg/waypoint.hpp>

namespace mav_interface_msgs {

// Struct identical to the Waypoint msg
struct WaypointEigen {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    float position_tolerance;
    float orientation_tolerance;

    inline WaypointEigen(
        const Eigen::Vector3d position = Eigen::Vector3d::Zero(),
        const Eigen::Quaterniond& orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
        float position_tolerance = 0.2,
        float orientation_tolerance = 0.25);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Struct identical to the Fullstate Stamped
struct FullStateStampedEigen {
    uint64_t timestamp_nano_seconds;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;

    // ToDo ->Think how to handle inconsistend acceleration and orientation commands.

    inline FullStateStampedEigen(
        const uint64_t timestamp_nano_seconds = 0,
        const Eigen::Vector3d position = Eigen::Vector3d::Zero(),
        const Eigen::Quaterniond& orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
        const Eigen::Vector3d& linear_velocity = Eigen::Vector3d::Zero(),
        const Eigen::Vector3d& linear_acceleration = Eigen::Vector3d::Zero(),
        const Eigen::Vector3d& angular_velocity = Eigen::Vector3d::Zero());

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline void WaypointEigen2Msg(const WaypointEigen& waypointEigen, mav_interface_msgs::msg::Waypoint& waypointMsg);

inline void FullStateStampedEigen2Msg(const FullStateStampedEigen& fullStateStampedEigen,
                                      mav_interface_msgs::msg::FullStateStamped& fullStateStampedMsg);

/** Return a resampled version of \p trajectory so that the positions of consecutive states are at
 * most \p max_distance apart.
 */
mav_interface_msgs::msg::FullStateTrajectory resample_trajectory(const mav_interface_msgs::msg::FullStateTrajectory& trajectory, const double max_distance);

} // namespace mav_interface_msgs

#include "conversions_impl.hpp"

#endif
