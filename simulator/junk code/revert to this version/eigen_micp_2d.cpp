#include <stdexcept>
#include <iostream>
#include "eigen_micp_2d.h"
#include "eigen_micp_2d_utils.h"
#include "rotations.h"
#include <fstream>

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


Eigen::Matrix<float, 2, 3> MICP2D::Jacobian_outer(float tick_l_i, float tick_r_i, float t_i){  // jacobian 
cerr<< " in Jacobian func!  "<< endl;
cerr<< " tick_r in J is : " << tick_r_i << endl; cerr<< " tick_l in J is : " << tick_l_i << endl; 
Eigen::Matrix<float, 2, 3> J;
float k_r=_K(0);
float k_l=_K(1);
float b =_K(2);
J << 
    tick_r_i/(2*t_i), tick_l_i/(2*t_i) ,           0,
    tick_r_i/ (b*t_i),-tick_l_i/(b*t_i),                (  (k_l*tick_l_i) - (k_r*tick_r_i) )/ ( (b*b)*t_i );
return J;
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
    /** use it to get integration step**/
    if (previous_stamp>0){
      dT=s-previous_stamp;
    } else 
      previous_stamp=s;
    /************************/
    if (r>=_range_max) {
      ep.point=Eigen::Vector2f(std::numeric_limits<float>::quiet_NaN(),
                                std::numeric_limits<float>::quiet_NaN());
      continue;
    }
   // _dt= dT;
    ep.status=true;
    Eigen::Vector3f pose;
    Matrix3_2f J_pose;
    twist2pose(pose, J_pose, tv, rv, dT, start_pose);
    Matrix2_3f J_ep;
    endpoint(ep.point, J_ep, pose, _sensor_pose, r, a);
    ep.J=J_ep*J_pose;
  }
}

void MICP2D::set_ticks(vector< float >& ticks_r_,
                      vector< float >& ticks_l_,std::vector< float >& timestamps_){
ticks_r=ticks_r_;
ticks_l=ticks_l_;
timestamps=timestamps_;
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
  const size_t s=ranges.size(); // s= total num of scans, ok acknowledged
  _endpoints.reserve(s); // vector of structs Endpoint, Endpoint = (x,y) J and status
  _midpoints.reserve(s); // vector of structs midpoint_and_normal, midpoint_and_normal= m, n, J_n, J_m and status
}
void MICP2D::set_init(float k_r, float k_l,float b){
_K<<k_r,k_l,b;

}
void MICP2D::oneRound() {
  string Correspondence_file="corres.dat";
  int counter =0;
  _dt=timestamps[1];
   
  cerr<< " ticks_r  " << endl; printvec(ticks_r);
  cerr<< " ticks_l  " << endl; printvec(ticks_l); 

  float t_r=ticks_r[1];
  float t_l=ticks_l[1]; 
  cerr<< " dt : "<<_dt<< " t_r : "<< t_r << " t_l : "<< t_l<< endl;
  float k_tv=(_K(0)*t_r + _K(1)*t_l )/(2*_dt);
  float k_rv=(_K(0)*t_r - _K(1)*t_l )/(_K(2)*_dt);
  cerr << "MY tv: " << k_tv << " MY rv: " << k_rv << endl;

  _correspondences.reserve(_num_beams); //using IntPairVector=std::vector<std::pair<int, int> >;
  _correspondences.clear();

  const std::vector<float>& ranges=*_ranges_ptr;
  const std::vector<double>& stamps=*_stamps_ptr;
  const std::vector<float>& beam_angles=*_beam_angles_ptr;

  float& tv=_X(0); //referencing _X(0) as tv, modifying tv i modify _X(0) too
  float& rv=_X(1); //referencing _X(1) as rv
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
  cerr<<"outer_end : "<<outer_end<<endl;
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
      //cerr<<"Jacobian matrix : "<< J<<endl;
      if (is_good) {
         chi2+=e.dot(e);
        _H+=J.transpose()*J;
        _b+=J.transpose()*e;
        ++_num_good_matches;
        //counter ++;  cerr<<" counter is: "<<counter<<endl;
        
      }
    }
  }
  // date le corrispondenze i e j, capire come i e j matchano a quale indice di tick_r,l e timestamp 
  Save_correspondences(Correspondence_file,_correspondences);
   cerr<<" num of good matches : "<<_num_good_matches <<endl;
 // _H*=10;
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

void printvec(std::vector<auto>& vectors){
for (int i = 0; i < vectors.size(); ++i){
     cerr << vectors[i]<<endl ;}
     cerr<<endl;
     cerr<<endl;

}

void read_ticks(std::string fileName, std::vector<float>* vecr, std::vector<float>* vecl,std::vector<float>* tempi){
  ifstream input_File;         
  input_File.open(fileName);
  string line;
  float tick_r,tick_l,time;
  if(input_File.fail()) { 
    cerr << "error COULDNT OPEN FILE " << endl; 
      } 
    
   while (std::getline(input_File, line)) {
    std::stringstream ss(line);   
    if (ss >> tick_r >> tick_l>>time) {
      (*vecr).push_back(tick_r); (*vecl).push_back(tick_l);
      (*tempi).push_back(time);
    }
  } 

}

void Save_correspondences(std::string fileName, std::vector<std::pair<int, int> > correspond){
  ofstream os(fileName);
for (auto & elem:correspond){
    os << elem.first <<' '<< elem.second<< endl;

}


}