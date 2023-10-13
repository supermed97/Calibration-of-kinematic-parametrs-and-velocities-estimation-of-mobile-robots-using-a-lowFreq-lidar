#pragma once
#include "eigen_micp_2d_utils.h"

struct MICP2D {

  void setMeasurement(const std::vector<float>& ranges,
                             const std::vector<double>& stamps,
                             const std::vector<float>& beam_angles);
  
  inline Eigen::Isometry2f& startPose() {return _start_pose;}
  inline const Eigen::Isometry2f& startPose() const {return _start_pose;}

  inline Eigen::Isometry2f& sensorPose() {return _sensor_pose;}
  inline const Eigen::Isometry2f& sensorPose() const {return _sensor_pose;}

  inline Vector2f& X() {return _X;}
  inline const Vector2f& X() const {return _X;}

  inline int numGoodMatches() const {return _num_good_matches;}
  inline const EndPointVector& endpoints() const {return _endpoints;}
  inline const MidPointAndNormalVector& midpoints() const {return _midpoints;}

  void oneRound();
protected:
  void timeScan2Points();
  MidPointAndNormal midpointAndNormal(const EndPoint& p1, const EndPoint& p2);
  bool errorAndJacobian(Eigen::Vector3f& e,
                        Matrix3_2f& J,
                        const MidPointAndNormal& mi,
                        const MidPointAndNormal& mj);

  Eigen::Isometry2f _sensor_pose =  Eigen::Isometry2f::Identity();
  const std::vector<float>* _ranges_ptr=0;
  const std::vector<double>* _stamps_ptr=0;
  const std::vector<float>* _beam_angles_ptr=0;
  int _num_good_matches=0;
  
  // tv, rv
  Vector2f _X=Eigen::Vector2f::Zero();
  Eigen::Matrix2f _H=Eigen::Matrix2f::Zero();
  Eigen::Vector2f _b=Eigen::Vector2f::Zero();

  IntPairVector _correspondences;

  Eigen::Isometry2f _estimated_pose=Eigen::Isometry2f::Identity();
  Eigen::Isometry2f _start_pose=Eigen::Isometry2f::Identity();
  float _normal_max_angle=0.5;
  float _range_max=100;
  float _range_min=0.1;
  float _d_max = 3;
  int   _search_window = 20;
  float   _d_normal_max = 2;
  float   _d_normal_min = 1;
  int _num_beams = 360;
private:
  float _normal_min_cos=cos(_normal_max_angle);
  float _d_normal2_max=pow(_d_normal_max,2);
  int   _d_normal2_min = pow(_d_normal_min, 2);

  EndPointVector _endpoints;
  MidPointAndNormalVector _midpoints;
};
