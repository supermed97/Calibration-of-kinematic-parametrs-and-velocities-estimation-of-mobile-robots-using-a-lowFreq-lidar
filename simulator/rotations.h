#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

using Matrix3_2f = Eigen::Matrix<float, 3, 2>;
using Matrix2_3f = Eigen::Matrix<float, 2, 3>;
using Matrix1_2f = Eigen::Matrix<float, 1, 2>;

inline Eigen::Matrix3f Rx(float theta) {
  float c=cos(theta);
  float s=sin(theta);
  Eigen::Matrix3f R;
  R <<
    1,  0,  0,
    0,  c, -s,
    0,  s,  c;
  return R;
}


inline Eigen::Matrix3f Ry(float theta) {
  float c=cos(theta);
  float s=sin(theta);
  Eigen::Matrix3f R;
  R <<
    c,  0,  s,
    0,  1,  0,
   -s,  0,  c;
  return R;
}

inline Eigen::Matrix3f Rz(float theta) {
  float c=cos(theta);
  float s=sin(theta);
  Eigen::Matrix3f R;
  R <<
    c, -s, 0,
    s,  c, 0,
    0,  0, 1;
  return R;
}

inline Eigen::Matrix2f Rtheta(float theta) {
  float c=cos(theta);
  float s=sin(theta);
  Eigen::Matrix2f R;
  R <<
    c, -s, 
    s,  c;
  return R;
}

inline Eigen::Isometry2f v2t(const Eigen::Vector3f pose) {
  Eigen::Isometry2f iso;
  iso.translation()=pose.head<2>();
  iso.linear()=Rtheta(pose(2));
  return iso;
}

inline Eigen::Vector3f t2v(const Eigen::Isometry2f iso) {
  Eigen::Vector3f pose;
  pose.head<2>()=iso.translation();
  pose(2)=atan2(iso.linear()(1,0),iso.linear()(0,0));
  return pose;
}


