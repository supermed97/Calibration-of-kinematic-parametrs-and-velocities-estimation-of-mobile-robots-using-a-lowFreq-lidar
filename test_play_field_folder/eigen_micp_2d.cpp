#include <stdexcept>
#include <iostream>
#include "eigen_micp_2d.h"
#include "eigen_micp_2d_utils.h"
#include "rotations.h"

using namespace std;

inline void twist2pose(Eigen::Vector3f& pose,
                       Matrix3_2f& J,
                       const float& tv, const float& rv, const float& dT,
                       const Eigen::Vector3f& start_pose=Eigen::Vector3f::Zero()) { //given v and omega, we integrate them to get the pose of the robot
  const float rho=tv*dT;
   const float theta=rv*dT;
  float ps=0, pc=0;
  float dps=0, dpc=0;
  static constexpr float coeffs [] = {1., 0.5, -1./6., -1./24.,1./120., 1./720.};
 
  float p=1;
  float dp=1;
  for (uint8_t i=0; i< (sizeof(coeffs)/sizeof(float)); ++i) {
    const float c=coeffs[i];
    const float dc=i*c;
    if (i&0x1) {
     pc+=c*p;
     dpc+=dc*dp;
    } else {
     ps+=c*p;
     dps+=dc*dp;
    }
    dp=p;
    p*=theta;
  }
  const auto R=Rtheta(start_pose(2));
  pose=start_pose;
  pose.head<2>()+=R*Eigen::Vector2f(rho*ps, rho*pc);
  pose(2)+=theta;
  J <<
    dT * ps, dT*dps*rho,
    dT * pc, dT*dpc*rho,
    0,       dT;
  J.block<2,2>(0,0)=R*J.block<2,2>(0,0);
}

inline void endpoint(Eigen::Vector2f& ep,
                     Matrix2_3f& J,
                     const Vector3f& pose,
                     const Eigen::Isometry2f& sensor_pose,
                     const float& r,
                     const float& alpha) {
  float angle=alpha+pose(2);
  float c=cos(angle), s=sin(angle);
  ep=sensor_pose * (pose.head<2>() + Vector2f(r*c, r*s));
  J.setZero();
  J.block<2,2>(0,0)=sensor_pose.linear();
  J.block<2,1>(0,2)=sensor_pose.linear()*Eigen::Vector2f(-r*s, r*c);
}

inline void normalize(Eigen::Vector2f& nv,
                      Eigen::Matrix2f& J,
                      const Eigen::Vector2f& v) {
  float n2=v.squaredNorm();
  float n=sqrt(n2);
  nv=v*1./n;
  J.setIdentity();
  J*=n2;
  J-=v*v.transpose();
  J/=(n*n2);
}
 

MidPointAndNormal MICP2D::midpointAndNormal(const EndPoint& p1,
                                            const EndPoint& p2) {
  MidPointAndNormal returned;
  Vector2f delta=p2.point-p1.point;
  const auto n=delta.squaredNorm();
  if (n>_d_normal2_max) {
    returned.status=TooFarForNormal;
    return returned;
  }
  if (n <_d_normal2_min) {
    returned.status=TooCloseForNormal;
    return returned;
  }
  returned.status=Good;
  returned.midpoint=0.5*(p1.point+p2.point);
  returned.J_midpoint=0.5*(p1.J+p2.J);
  static Eigen::Matrix2f S;
  S << 0, -1, 1, 0;
  auto orth=S*delta;
  Eigen::Matrix2f J_norm;
  normalize(returned.normal, J_norm, orth);
  returned.J_normal=J_norm*S*(p2.J-p1.J);
  return returned;
}

bool MICP2D::errorAndJacobian(Eigen::Vector3f& e,
                             Matrix3_2f& J,
                             const MidPointAndNormal& mi,
                             const MidPointAndNormal& mj){
  e.setZero();
  J.setZero();
  const Vector2f mp_delta=(mj.midpoint-mi.midpoint);
  const Vector2f n_sum=(mi.normal+mj.normal);
  if (mi.normal.dot(mj.normal)<_normal_min_cos)
    return false;
  e(0)=(mp_delta).dot(n_sum);
  e.block<2,1>(1,0)=mj.normal-mi.normal;
  
  J.block<1,2>(0,0)=
    n_sum.transpose()*(mj.J_midpoint-mi.J_midpoint)
    +mp_delta.transpose()*(mj.J_normal+mi.J_normal);
  J.block<2,2>(1,0)=mj.J_normal-mi.J_normal;
  return true;
}

void MICP2D::timeScan2Points() { // considering the motion of the robot as constant it puts the points in 2d ??

  const auto& ranges = *_ranges_ptr;
  const auto& stamps = * _stamps_ptr;
  const auto& beam_angles = * _beam_angles_ptr;
  const float& tv=_X(0);
  const float& rv=_X(1);

  size_t num_samples=ranges.size();
  _endpoints.resize(num_samples);
  
  double previous_stamp=-1;
  int dest_idx=0;
  auto start_pose=t2v(_start_pose);
  for (size_t i=0; i<stamps.size(); ++i) {
    auto& ep=_endpoints[i];
    const auto s=stamps[i];
    const auto r=ranges[i];
    const auto a=beam_angles[i];
    ep.status=false;
    double dT=0;
    if (previous_stamp>0){
      dT=s-previous_stamp;
    } else 
      previous_stamp=s;
    
    if (r>=_range_max) {
      ep.point=Eigen::Vector2f(std::numeric_limits<float>::quiet_NaN(),
                                std::numeric_limits<float>::quiet_NaN());
      continue;
    }
    ep.status=true;
    Eigen::Vector3f pose;
    Matrix3_2f J_pose;
    twist2pose(pose, J_pose, tv, rv, dT, start_pose);
    Matrix2_3f J_ep;
    endpoint(ep.point, J_ep, pose, _sensor_pose, r, a);
    ep.J=J_ep*J_pose;
  }
}

void MICP2D::setMeasurement(const std::vector<float>& ranges,
                            const std::vector<double>& stamps,
                            const std::vector<float>& beam_angles) {
  _ranges_ptr=&ranges;
  _stamps_ptr=&stamps;
  _beam_angles_ptr=&beam_angles;
  if (stamps.size()!=ranges.size())
    throw std::runtime_error("stamps-ranges size mismatch");
  if (stamps.size()!=beam_angles.size())
    throw std::runtime_error("stamps-beam_angles size mismatch");
  const size_t s=ranges.size();
  _endpoints.reserve(s);
  _midpoints.reserve(s);
}

void MICP2D::oneRound() {
  
  _correspondences.reserve(_num_beams);
  _correspondences.clear();

  const std::vector<float>& ranges=*_ranges_ptr;
  const std::vector<double>& stamps=*_stamps_ptr;
  const std::vector<float>& beam_angles=*_beam_angles_ptr;

  float& tv=_X(0);
  float& rv=_X(1);
  cerr << "tv: " << tv << " rv: " << rv << endl;
  double t_start=getTime();

  timeScan2Points();
  
  size_t num_samples=ranges.size();
  _midpoints.resize(num_samples-1);
  int midpoints_num_good=0;
  int skip=1;
  int normal_jumped=0;
  for (size_t i=0; i<num_samples-skip; ++i) {
    auto& mp=_midpoints[i];
    mp.status=Bad;
    const auto& ep1=_endpoints[i];
    const auto& ep2=_endpoints[i+skip];
    
    if (! ep1.status || !ep2.status) {
      skip=1;
      continue;
    }

    mp=midpointAndNormal(ep1, ep2);
    if (mp.status==Good) {
      ++midpoints_num_good;
      skip=1;
    }
    if (mp.status == TooCloseForNormal) {
      normal_jumped ++;
      skip++;
    }
    
  }
  cerr << "good_midpoints: " << midpoints_num_good << endl;
  cerr << "normal_jumped: " << normal_jumped << endl;
  
  int outer_end=std::min(_num_beams, (int)num_samples-1);
  float chi2=0;
  _num_good_matches=0;
  _b.setZero();
  _H.setZero();
  for (int i=0; i<outer_end; ++i) {
    auto& mi=_midpoints[i];
    if (mi.status!=Good)
      continue;
    int inner_start=std::max(0, i+_num_beams-_search_window);
    int inner_end=std::min((int)_midpoints.size(),  i+_num_beams+_search_window);
    int j_min=-1;
    float d_max_2=_d_max*_d_max;
    for (int j=inner_start; j<inner_end; ++j) {
      const auto& mj=_midpoints[j];
      if (mj.status!=Good)
        continue;
      
      float d=(mi.midpoint-mj.midpoint).squaredNorm();
      if (d_max_2>d) {
        j_min=j;
        d_max_2=d;
      }
    }
    if (j_min!=-1) {
      _correspondences.push_back(std::make_pair(i,j_min));
      auto& mj=_midpoints[j_min];
      Eigen::Vector3f e;
      Matrix3_2f J;
      bool is_good=errorAndJacobian(e, J, mi, mj);
      if (is_good) {
        chi2+=e.dot(e);
        _H+=J.transpose()*J;
        _b+=J.transpose()*e;
        ++_num_good_matches;
      }
    }
  }
  Vector2f dx=_H.ldlt().solve(-_b);
  cerr << "dx: "<< dx.transpose() << endl;
  tv+=dx(0);
  rv+=dx(1);
  double t_end=getTime();
  cerr << "ng: " << _num_good_matches << " chi2: " << chi2 << " chi2/good: " << chi2/_num_good_matches << endl;
  cerr << "time iteration: " << t_end-t_start << endl;
  cout << "set size ratio -1" << endl;
  cout << "set term x11 2" << endl;
  plotCorrespondences(_endpoints, _correspondences);
  char c;
  cin >> c;
}



