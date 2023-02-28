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

  WaypointEigen(const Eigen::Vector3d position = Eigen::Vector3d::Zero(),
                const Eigen::Quaterniond& orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), float positionTolerance = 0.2,
                float orientationTolerance = 0.25)
      : position(position), orientation(orientation.normalized()), positionTolerance(positionTolerance),
        orientationTolerance(orientationTolerance){};

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

  FullStateStampedEigen(const uint64_t timestampNanoSeconds = 0,
                        const Eigen::Vector3d position = Eigen::Vector3d::Zero(),
                        const Eigen::Quaterniond& orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
                        const Eigen::Vector3d& linearVelocity = Eigen::Vector3d::Zero(),
                        const Eigen::Vector3d& linearAcceleration = Eigen::Vector3d::Zero(),
                        const Eigen::Vector3d& angularVelocity = Eigen::Vector3d::Zero())
      : timestampNanoSeconds(timestampNanoSeconds), position(position), orientation(orientation.normalized()),
        linearVelocity(linearVelocity), linearAcceleration(linearAcceleration), angularVelocity(angularVelocity){};

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline void WaypointEigen2Msg(const WaypointEigen& waypointEigen, Waypoint& waypointMsg) {
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
                                      FullStateStamped& fullStateStampedMsg) {
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
