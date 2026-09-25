/*
 * Copyright 2015,2016,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Eigen::Quaternion helter functions
 * @file uas_quaternion_utils.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup nodelib
 * @{
 */

#include <cmath>

#include <mavros/frame_tf.hpp>

namespace mavros
{
namespace ftf
{

/*
 * Note: order of axis are match tf2::LinearMath (bullet).
 * YPR rotation convention -> YAW first, Pitch second, Roll third
 * Compatibility checked by unittests.
 */

Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d & rpy)
{
  // YPR - ZYX
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
  );
}

Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond & q)
{
  // YPR - ZYX
  return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}

Eigen::Vector3d quaternion_to_rpy_zyx(const Eigen::Quaterniond & q)
{
  // Inverse of quaternion_from_rpy(): q = Rz(yaw) * Ry(pitch) * Rx(roll).
  const double n = q.norm();
  if (n < 1e-12) {
    return Eigen::Vector3d::Zero();
  }

  const double w = q.w() / n;
  const double x = q.x() / n;
  const double y = q.y() / n;
  const double z = q.z() / n;

  const double sinp = 2.0 * (w * y - z * x);
  // At pitch = ±90 deg, roll and yaw are one angle. The general atan2 form
  // loses the rotation there, including when sinp lands a ulp outside [-1, 1].
  if (sinp >= 1.0 - 1e-8) {
    const double r01 = 2.0 * (x * y - w * z);
    const double r02 = 2.0 * (x * z + w * y);
    return Eigen::Vector3d(0.0, M_PI / 2.0, -std::atan2(r01, r02));
  }
  if (sinp <= -1.0 + 1e-8) {
    const double r01 = 2.0 * (x * y - w * z);
    const double r02 = 2.0 * (x * z + w * y);
    return Eigen::Vector3d(0.0, -M_PI / 2.0, std::atan2(-r01, -r02));
  }

  const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const double pitch = std::asin(sinp);
  const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

  return Eigen::Vector3d(roll, pitch, yaw);
}

double quaternion_get_yaw(const Eigen::Quaterniond & q)
{
  // to match equation from:
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  const double & q0 = q.w();
  const double & q1 = q.x();
  const double & q2 = q.y();
  const double & q3 = q.z();

  return std::atan2(2. * (q0 * q3 + q1 * q2), 1. - 2. * (q2 * q2 + q3 * q3));
}

}       // namespace ftf
}       // namespace mavros
