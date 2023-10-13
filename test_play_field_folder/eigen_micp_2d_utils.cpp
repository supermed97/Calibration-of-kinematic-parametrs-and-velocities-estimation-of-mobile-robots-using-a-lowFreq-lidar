#include <sys/time.h>
#include <iostream>
#include "rotations.h"
#include "eigen_micp_2d_utils.h"

using namespace std;

double getTime() {
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_sec*1e3+tv.tv_usec*1e-3;
}

void drawLine(Vector2fVector& dest,
              const Vector2f& p0,
              const Vector2f& p1,
              float density) {
  Vector2f delta=p1-p0;
  float length=delta.norm();
  int n_points=length*density;
  Vector2f d=delta/n_points;
  for (int i=0; i<n_points; ++i) 
    dest.push_back(p0+d*float(i));
  
}


void points2scan(std::vector<float>& dest,
                 const Vector2fVector& points,
                 const Eigen::Isometry2f& laser_pose,
                 const float angle_min,
                 const float angle_max,
                 const float range_max){
  auto num_beams=dest.size();
  const float fov=angle_max-angle_min;
  auto inv_res=num_beams/fov;
  if (! num_beams)
    return;
  const float r2=pow(range_max,2);
  std::fill(dest.begin(), dest.end(), r2);
  const auto inv_t=laser_pose.inverse();
  for (const auto& p: points) {
    auto p_local=inv_t*p;
    const float n2=p_local.squaredNorm();
    if (n2>r2)
      continue;
    const float angle=atan2(p_local.y(), p_local.x());
    size_t angle_idx=(angle-angle_min)*inv_res;
    if (angle_idx>=num_beams)
      continue;
    auto& d=dest[angle_idx];
    d=std::min(d,n2);
  }
  for (auto& d: dest)
    d=sqrt(d);
}

void scan2points(Vector2fVector& dest,
                 const std::vector<float>& ranges,
                 const Eigen::Isometry2f& laser_pose,
                 const float angle_min,
                 const float angle_max,
                 const float range_max){
  auto num_beams=ranges.size();
  dest.reserve(dest.size()+num_beams);
  const float fov=angle_max-angle_min;
  auto res=fov/num_beams;
  if (! num_beams)
    return;
  for (size_t i=0; i<num_beams; ++i) {
    const auto& r = ranges[i];
    if (r>=range_max)
      continue;
    float alpha=angle_min+i*res;
    dest.push_back(laser_pose*Vector2f(r*cos(alpha), r*sin(alpha)));
  }
}


void twist2pose(Eigen::Vector3f& pose,
                const float& tv, const float& rv, const float& dT,
                const Eigen::Vector3f& start_pose) {
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
}

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
                     const float angle_min,
                     const float angle_max,
                     const float range_max) {
  dest.reserve(dest.size()+num_samples);
  stamps.reserve(stamps.size()+num_samples);
  beam_angles.reserve(beam_angles.size()+num_samples);
  laser_poses.reserve(laser_poses.size()+num_samples);
  std::vector<float> temp_dest(num_samples);
  Eigen::Isometry2f iso=laser_pose;
  const float fov=angle_max-angle_min;
  auto res=fov/num_samples;
  for (auto i=0; i<num_samples; ++i) {
    Vector3f pose;
    twist2pose(pose, tv, rv, i*dT, t2v(laser_pose));
    points2scan(temp_dest, points, v2t(pose), angle_min, angle_max, range_max);
    dest.push_back(temp_dest[i]);
    stamps.push_back(t_start+i*dT);
    beam_angles.push_back(angle_min+i*res);
    laser_poses.push_back(pose);
  }
}

void plotPoints(Vector2fVector& points) {
  cout << "plot '-' w p ps 1" << endl;
  for (const auto& p: points) {
    if (std::isnan(p.x()))
        continue;
    cout << p.transpose() << endl;
  }
  cout << "e" << endl;
}

void plotCorrespondences(Vector2fVector& points, IntPairVector& correspondences) {
  cout << "plot '-' w p ps 1, '-' w l lw 0.1" << endl;
  for (const auto& p: points) {
    if (std::isnan(p.x()))
        continue;
    cout << p.transpose() << endl;
  }
  cout << "e" << endl;
  for (const auto& c: correspondences) {
    const auto& pi=points[c.first];
    const auto& pj=points[c.second];
    cout << pi.transpose() << endl;
    cout << pj.transpose() << endl;
    cout << endl;
  }
  cout << "e" << endl;
}

void plotCorrespondences(EndPointVector& points, IntPairVector& correspondences) {
  cout << "plot '-' w p ps 1, '-' w l lw 0.1" << endl;
  for (const auto& p: points) {
    if (std::isnan(p.point.x()))
        continue;
    cout << p.point.transpose() << endl;
  }
  cout << "e" << endl;
  for (const auto& c: correspondences) {
    const auto& pi=points[c.first].point;
    const auto& pj=points[c.second].point;
    cout << pi.transpose() << endl;
    cout << pj.transpose() << endl;
    cout << endl;
  }
  cout << "e" << endl;
}
