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
#include <mav_interface_msgs/FullStateStamped.h>
#include <mav_interface_msgs/FullStateTrajectory.h>
#include <mav_interface_msgs/Path.h>
#include <mav_interface_msgs/Waypoint.h>

namespace mav_interface_msgs {

// Struct identical to the Waypoint msg
struct WaypointEigen {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    float positionTolerance;
    float orientationTolerance;

    inline WaypointEigen(
        const Eigen::Vector3d position = Eigen::Vector3d::Zero(),
        const Eigen::Quaterniond& orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
        float positionTolerance = 0.2,
        float orientationTolerance = 0.25);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Struct identical to the Fullstate Stamped
struct FullStateStampedEigen {
    uint64_t timestampNanoSeconds;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d linearVelocity;
    Eigen::Vector3d linearAcceleration;
    Eigen::Vector3d angularVelocity;

    // ToDo ->Think how to handle inconsistend acceleration and orientation commands.

    inline FullStateStampedEigen(
        const uint64_t timestampNanoSeconds = 0,
        const Eigen::Vector3d position = Eigen::Vector3d::Zero(),
        const Eigen::Quaterniond& orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
        const Eigen::Vector3d& linearVelocity = Eigen::Vector3d::Zero(),
        const Eigen::Vector3d& linearAcceleration = Eigen::Vector3d::Zero(),
        const Eigen::Vector3d& angularVelocity = Eigen::Vector3d::Zero());

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline void WaypointEigen2Msg(const WaypointEigen& waypointEigen, Waypoint& waypointMsg);

inline void FullStateStampedEigen2Msg(const FullStateStampedEigen& fullStateStampedEigen,
                                      FullStateStamped& fullStateStampedMsg);

/** Return a resampled version of \p trajectory so that the positions of consecutive states are at
 * most \p max_distance apart.
 */
FullStateTrajectory resample_trajectory(const FullStateTrajectory& trajectory, const double max_distance);

} // namespace mav_interface_msgs

#include "conversions_impl.hpp"

#endif
