#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

using Vector3f=Eigen::Vector3f;
using Vector2f=Eigen::Vector2f;
using Vector2fVector=std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >;
using Matrix2fVector=std::vector<Eigen::Matrix2f, Eigen::aligned_allocator<Eigen::Matrix2f> >;
using Vector3fVector=std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >;
using Matrix3_2f = Eigen::Matrix<float, 3, 2>;
using Matrix2_3f = Eigen::Matrix<float, 2, 3>;
using IntPairVector=std::vector<std::pair<int, int> >;


double getTime();

void drawLine(Vector2fVector& dest,
              const Vector2f& p0,
              const Vector2f& p1,
              float density=100);

void points2scan(std::vector<float>& dest,
                 const Vector2fVector& points,
                 const Eigen::Isometry2f& laser_pose,
                 const float angle_min=-M_PI,
                 const float angle_max=M_PI,
                 const float range_max=100);

void scan2points(Vector2fVector& dest,
                 const std::vector<float>& ranges,
                 const Eigen::Isometry2f& laser_pose,
                 const float angle_min=-M_PI,
                 const float angle_max=M_PI,
                 const float range_max=100);

void twist2pose(Eigen::Vector3f& pose,
                const float& tv, const float& rv, const float& dT,
                const Eigen::Vector3f& start_pose=Eigen::Vector3f::Zero());

// Inefficiency award
void points2timeScan(std::vector<float>& dest,
                     std::vector<double>& stamps,
                     std::vector<float>& beam_angles,
                     Vector3fVector& laser_poses,
                     const size_t num_samples,
                     const Vector2fVector& points,
                     const Eigen::Isometry2f& laser_pose,
                     const float &tv,
                     const float &rv,
                     const float&t_start,
                     const float &dT,
                     const float angle_min=-M_PI,
                     const float angle_max=M_PI,
                     const float range_max=100);

void plotPoints(Vector2fVector& points);

void plotCorrespondences(Vector2fVector& points,
                         IntPairVector& correspondences);


enum MidPointStatus {Good=0, TooCloseForNormal=1, TooFarForNormal=2, Bad=3};
struct MidPointAndNormal{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Eigen::Vector2f midpoint;
  Eigen::Vector2f normal;
  Eigen::Matrix2f J_midpoint;
  Eigen::Matrix2f J_normal;
  MidPointStatus status=Bad;
};

using MidPointAndNormalVector =
  std::vector<MidPointAndNormal,
              Eigen::aligned_allocator<MidPointAndNormal>
              >;

struct EndPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Eigen::Vector2f point=Eigen::Vector2f(std::numeric_limits<float>::quiet_NaN(),
                                        std::numeric_limits<float>::quiet_NaN());
  Eigen::Matrix2f J;
  bool status=false;
};
  
using EndPointVector =
  std::vector<EndPoint,
              Eigen::aligned_allocator<EndPoint>
              >;

void plotCorrespondences(EndPointVector& points,
                         IntPairVector& correspondences);
