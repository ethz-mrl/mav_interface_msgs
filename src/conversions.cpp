/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cmath>
#include <mav_interface_msgs/msg/full_state_stamped.hpp>
#include <mav_interface_msgs/msg/full_state_trajectory.hpp>
#include <mav_interface_msgs/conversions.h>


// copied over from okvis/ros2/eigen_conversions_reimplementation.hpp to avoid cyclic and unnecessary dependencies.
namespace tf {
    inline void quaternionMsgToEigen(const geometry_msgs::msg::Quaternion &q_msg, Eigen::Quaterniond &q) {
        q = Eigen::Quaterniond(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    }

    inline void quaternionEigenToMsg(const Eigen::Quaterniond &q, geometry_msgs::msg::Quaternion &q_msg) {
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();
        q_msg.w = q.w();
    }

    inline void vectorMsgToEigen(const geometry_msgs::msg::Vector3 &v_msg, Eigen::Vector3d &v) {
        v.x() = v_msg.x;
        v.y() = v_msg.y;
        v.z() = v_msg.z;
    }

    inline void vectorEigenToMsg(const Eigen::Vector3d &v, geometry_msgs::msg::Vector3 &v_msg) {
        v_msg.x = v.x();
        v_msg.y = v.y();
        v_msg.z = v.z();
    }
}

namespace mav_interface_msgs {

namespace detail {

double position_distance(const mav_interface_msgs::msg::FullStateStamped& a, const mav_interface_msgs::msg::FullStateStamped& b)
{
    Eigen::Vector3d a_position, b_position;
    tf::vectorMsgToEigen(a.position, a_position);
    tf::vectorMsgToEigen(b.position, b_position);
    return (b_position - a_position).norm();
}

geometry_msgs::msg::Vector3 lerp_vector3(const double t, const geometry_msgs::msg::Vector3& start, const geometry_msgs::msg::Vector3& end)
{
    Eigen::Vector3d start_eigen, end_eigen;
    tf::vectorMsgToEigen(start, start_eigen);
    tf::vectorMsgToEigen(end, end_eigen);
    const Eigen::Vector3d value_eigen = start_eigen + t * (end_eigen - start_eigen);
    geometry_msgs::msg::Vector3 value;
    tf::vectorEigenToMsg(value_eigen, value);
    return value;
}



geometry_msgs::msg::Quaternion slerp_quaternion(const double t,
                                           const geometry_msgs::msg::Quaternion& start,
                                           const geometry_msgs::msg::Quaternion& end)
{
    Eigen::Quaterniond start_eigen, end_eigen;
    tf::quaternionMsgToEigen(start, start_eigen);
    tf::quaternionMsgToEigen(end, end_eigen);
    const Eigen::Quaterniond value_eigen = start_eigen.slerp(t, end_eigen);
    geometry_msgs::msg::Quaternion value;
    tf::quaternionEigenToMsg(value_eigen, value);
    return value;
}



mav_interface_msgs::msg::FullStateStamped interp_state(const double t, const mav_interface_msgs::msg::FullStateStamped& start, const mav_interface_msgs::msg::FullStateStamped& end)
{
    mav_interface_msgs::msg::FullStateStamped s;
    s.timestamp_nano_seconds =
        start.timestamp_nano_seconds + t * (end.timestamp_nano_seconds - start.timestamp_nano_seconds);
    s.position = lerp_vector3(t, start.position, end.position);
    s.orientation = slerp_quaternion(t, start.orientation, end.orientation);
    s.linear_velocity = lerp_vector3(t, start.linear_velocity, end.linear_velocity);
    s.linear_acceleration = lerp_vector3(t, start.linear_acceleration, end.linear_acceleration);
    // TODO: It's probably not correct to just linearly interpolate angular velocity but it's also
    // not currently used.
    s.angular_velocity = lerp_vector3(t, start.angular_velocity, end.angular_velocity);
    return s;
}

} // namespace detail



mav_interface_msgs::msg::FullStateTrajectory resample_trajectory(const mav_interface_msgs::msg::FullStateTrajectory& trajectory,
                                        const double max_distance)
{
    mav_interface_msgs::msg::FullStateTrajectory resampled;
    resampled.header = trajectory.header;
    resampled.task_id = trajectory.task_id;
    resampled.initial_waypoint = trajectory.initial_waypoint;
    resampled.flush_reference_queue = trajectory.flush_reference_queue;
    if (trajectory.trajectory.empty()) {
        return resampled;
    }
    auto& rt = resampled.trajectory;
    rt.push_back(trajectory.trajectory.front());
    for (size_t i = 1; i < trajectory.trajectory.size(); i++) {
        // Interpolate intermediate states.
        const auto dist = [&](const mav_interface_msgs::msg::FullStateStamped& s) {
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
