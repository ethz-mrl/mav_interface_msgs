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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  float positionTolerance;
  float orientationTolerance;

  WaypointEigen(const Eigen::Vector3d position = Eigen::Vector3d::Zero(),
                Eigen::Quaterniond orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), float positionTolerance = 0.2,
                float orientationTolerance = 0.25)
      : position(position), orientation(orientation.normalized()), positionTolerance(positionTolerance),
        orientationTolerance(orientationTolerance){};
};

// Struct identical to the Fullstate Stamped
struct FullStateStampedEigen {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  uint64_t timestampNanoSeconds;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d linearVelocity;
  Eigen::Vector3d linearAcceleration;
  Eigen::Vector3d angularVelocity;

  // ToDo ->Think how to handle inconsistend acceleration and orientation commands.

  FullStateStampedEigen(const uint64_t timestampNanoSeconds = 0,
                        const Eigen::Vector3d position = Eigen::Vector3d::Zero(),
                        Eigen::Quaterniond orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero(),
                        Eigen::Vector3d linearAcceleration = Eigen::Vector3d::Zero(),
                        Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero())
      : timestampNanoSeconds(timestampNanoSeconds), position(position), orientation(orientation.normalized()),
        linearVelocity(linearVelocity), linearAcceleration(linearAcceleration), angularVelocity(angularVelocity){};
};

inline void WaypointEigen2Msg(const WaypointEigen &waypointEigen, Waypoint &waypointMsg) {
  // Copy position
  waypointMsg.position.x = waypointEigen.position(0);
  waypointMsg.position.y = waypointEigen.position(1);
  waypointMsg.position.z = waypointEigen.position(2);

  // Copy orientation
  waypointMsg.orientation.w = waypointEigen.orientation.w();
  waypointMsg.orientation.x = waypointEigen.orientation.x();
  waypointMsg.orientation.y = waypointEigen.orientation.y();
  waypointMsg.orientation.z = waypointEigen.orientation.z();

  // Copy position and orientation tolerance
  waypointMsg.positionTolerance = waypointEigen.positionTolerance;
  waypointMsg.orientationTolerance = waypointEigen.orientationTolerance;
}

inline void FullStateStampedEigen2Msg(const FullStateStampedEigen &fullStateStampedEigen,
                                      FullStateStamped &fullStateStampedMsg) {
  // Timestamp
  fullStateStampedMsg.timestampNanoSeconds = fullStateStampedEigen.timestampNanoSeconds;

  // Copy position
  fullStateStampedMsg.position.x = fullStateStampedEigen.position(0);
  fullStateStampedMsg.position.y = fullStateStampedEigen.position(1);
  fullStateStampedMsg.position.z = fullStateStampedEigen.position(2);

  // Copy orientation
  fullStateStampedMsg.orientation.w = fullStateStampedEigen.orientation.w();
  fullStateStampedMsg.orientation.x = fullStateStampedEigen.orientation.x();
  fullStateStampedMsg.orientation.y = fullStateStampedEigen.orientation.y();
  fullStateStampedMsg.orientation.z = fullStateStampedEigen.orientation.z();

  // Copy linearVelocity
  fullStateStampedMsg.linearVelocity.x = fullStateStampedEigen.linearVelocity(0);
  fullStateStampedMsg.linearVelocity.y = fullStateStampedEigen.linearVelocity(1);
  fullStateStampedMsg.linearVelocity.z = fullStateStampedEigen.linearVelocity(2);

  // Copy linearAcceleration
  fullStateStampedMsg.linearAcceleration.x = fullStateStampedEigen.linearAcceleration(0);
  fullStateStampedMsg.linearAcceleration.y = fullStateStampedEigen.linearAcceleration(1);
  fullStateStampedMsg.linearAcceleration.z = fullStateStampedEigen.linearAcceleration(2);

  // Copy angularVelocity
  fullStateStampedMsg.angularVelocity.x = fullStateStampedEigen.angularVelocity(0);
  fullStateStampedMsg.angularVelocity.y = fullStateStampedEigen.angularVelocity(1);
  fullStateStampedMsg.angularVelocity.z = fullStateStampedEigen.angularVelocity(2);
}

} // namespace mav_interface_msgs

#endif
