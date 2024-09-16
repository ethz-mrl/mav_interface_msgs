/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cmath>
#include <eigen_conversions/eigen_msg.h>
#include <mav_interface_msgs/FullStateStamped.h>
#include <mav_interface_msgs/FullStateTrajectory.h>
#include <mav_interface_msgs/conversions.h>

namespace mav_interface_msgs {

namespace detail {

double position_distance(const FullStateStamped& a, const FullStateStamped& b)
{
    Eigen::Vector3d a_position, b_position;
    tf::vectorMsgToEigen(a.position, a_position);
    tf::vectorMsgToEigen(b.position, b_position);
    return (b_position - a_position).norm();
}



geometry_msgs::Vector3
lerp_vector3(const double t, const geometry_msgs::Vector3& start, const geometry_msgs::Vector3& end)
{
    Eigen::Vector3d start_eigen, end_eigen;
    tf::vectorMsgToEigen(start, start_eigen);
    tf::vectorMsgToEigen(end, end_eigen);
    const Eigen::Vector3d value_eigen = start_eigen + t * (end_eigen - start_eigen);
    geometry_msgs::Vector3 value;
    tf::vectorEigenToMsg(value_eigen, value);
    return value;
}



geometry_msgs::Quaternion slerp_quaternion(const double t,
                                           const geometry_msgs::Quaternion& start,
                                           const geometry_msgs::Quaternion& end)
{
    Eigen::Quaterniond start_eigen, end_eigen;
    tf::quaternionMsgToEigen(start, start_eigen);
    tf::quaternionMsgToEigen(end, end_eigen);
    const Eigen::Quaterniond value_eigen = start_eigen.slerp(t, end_eigen);
    geometry_msgs::Quaternion value;
    tf::quaternionEigenToMsg(value_eigen, value);
    return value;
}



FullStateStamped
interp_state(const double t, const FullStateStamped& start, const FullStateStamped& end)
{
    FullStateStamped s;
    s.timestampNanoSeconds =
        start.timestampNanoSeconds + t * (end.timestampNanoSeconds - start.timestampNanoSeconds);
    s.position = lerp_vector3(t, start.position, end.position);
    s.orientation = slerp_quaternion(t, start.orientation, end.orientation);
    s.linearVelocity = lerp_vector3(t, start.linearVelocity, end.linearVelocity);
    s.linearAcceleration = lerp_vector3(t, start.linearAcceleration, end.linearAcceleration);
    // TODO: It's probably not correct to just linearly interpolate angular velocity but it's also
    // not currently used.
    s.angularVelocity = lerp_vector3(t, start.angularVelocity, end.angularVelocity);
    return s;
}

} // namespace detail



FullStateTrajectory resample_trajectory(const FullStateTrajectory& trajectory,
                                        const double max_distance)
{
    FullStateTrajectory resampled;
    resampled.header = trajectory.header;
    resampled.taskID = trajectory.taskID;
    resampled.initialWaypoint = trajectory.initialWaypoint;
    resampled.flushReferenceQueue = trajectory.flushReferenceQueue;
    if (trajectory.trajectory.empty()) {
        return resampled;
    }
    auto& rt = resampled.trajectory;
    rt.push_back(trajectory.trajectory.front());
    for (size_t i = 1; i < trajectory.trajectory.size(); i++) {
        // Interpolate intermediate states.
        const auto dist = [&](const FullStateStamped& s) {
            return detail::position_distance(s, trajectory.trajectory[i]);
        };
        for (double d = dist(rt.back()); d > max_distance; d = dist(rt.back())) {
            const double t = max_distance / d;
            rt.push_back(detail::interp_state(t, rt.back(), trajectory.trajectory[i]));
        }
        // Push-back the original state.
        rt.push_back(trajectory.trajectory[i]);
    }
    return resampled;
}

} // namespace mav_interface_msgs
