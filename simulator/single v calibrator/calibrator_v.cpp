#include "calibrator.h"
#include "../simulator.h"
#include "Eigen/Geometry"
#include "../rotations.h"
#include "Eigen/Cholesky"
#include <iostream>
#include <memory>

/**
void f(vector<object> *vo)
vector<object> vo;
f(&vo);
(*vo).push_back(object());
vo->push_back(object()); // short-hand
**/
using namespace std;
using Vector3f = Eigen::Vector3f;
calibrator::calibrator(std::vector<float>& timestamps_, std::vector<float>& parameters, float k_l_, float k_r_, float b_){

  timestamps=&timestamps_;
  //set_ticks();
  v=parameters[0];
  omega=parameters[1];
  error.setZero();
  chi.setZero();
  sum_Jacobian.setZero();
  covariance.setIdentity();
  k_l=k_l_;
  k_r=k_r_;
  b=b_;
}

void calibrator::set_ticks(std::vector<float>& ticks_r_,std::vector<float>& ticks_l_){
(*ticks_r)=ticks_r_;
(*ticks_l)=ticks_l_;
}

Vector2f calibrator::calc_error_v(float tick_l_i, float tick_r_i, float t_i){ //define x,yand theta
float theta_e=theta(tick_r_i, tick_l_i,k_l,k_r,b);

error_2(0)=  (s(tick_r_i,tick_l_i,k_l,k_r))- (v*t_i);
error_2(1)=  (theta_e) - (omega*t_i);

cout << "error at time " << t_i << endl<<  error_2 << endl;
return error_2;
}

Eigen::Matrix<float, 2, 3> calibrator::computeJacobian_v(float tick_l_i, float tick_r_i, float t_i){  // jacobian at time i
Eigen::Matrix<float, 2, 3> J;
float omega_e=theta(tick_r_i, tick_l_i,k_l,k_r,b);
J <<
    tick_r_i/(2), tick_l_i/(2) ,           0,
    tick_r_i/ (b),                            -tick_l_i/(b),                              (  (k_l*tick_l_i) - (k_r*tick_r_i) )/ ( (b*b) );
return J;
}

Eigen::Matrix3f calibrator::calc_H(Eigen::Matrix3f * H, Eigen::Vector3f* b) {
  chi_f=0;
  float chi;
  Eigen::Matrix<float, 2, 3> J;
  Eigen:: Vector2f err,sum_err;
  J.setZero();
  err.setZero();
  sum_err.setZero();

  for (int i = 0; i < (*ticks_l).size(); ++i){

      J=calibrator::computeJacobian_v((*ticks_l)[i],(*ticks_r)[i],(*timestamps)[i] );
      cout<< "Jacobian at time : "<< (*timestamps)[i] << endl;
      cout<< J << endl;
      err=calibrator:: calc_error_v((*ticks_l)[i],(*ticks_r)[i],(*timestamps)[i]);
     // cout<< " summed error till time : "<< (*timestamps)[i] << endl;
      //cout << sum_err<< endl;
      //chi=err.squaredNorm();
      chi=err.transpose()*err;
      chi_f+=chi;
      *H+=(J.transpose()*J );
      *b+=(J.transpose()*err);
  }
  *H+=Eigen::Matrix3f::Identity()*100;
  return *H;
}



void calibrator::least_square_optimize() {

  Eigen::Matrix<float, 3, 3> H;
  Eigen::Matrix<float, 3, 1> b_ls;
  H.setZero();
  b_ls.setZero();
  calc_H(&H,&b_ls);
  _dx=H.ldlt().solve(-b_ls);

cout<<"delta_k below "<< endl << endl;
cout << _dx << endl << endl;
k_r+=_dx(0);
k_l+= _dx(1);
b+= _dx(2);
}

void calibrator::print_paramters(){
cout<< "k_r is : " << k_r << endl;
cout << "k_l is : " << k_l << endl;
cout << "b is : " << b<< endl;

}

void calibrator::run(int max_iterations) {

   cerr<< " time stamps " << endl; printvec(timestamps);
   cerr<< " ticks_r " << endl; printvec(ticks_r);
    cerr<< " ticks_l " << endl; printvec(ticks_l);
  int current_iteration=0;
  while (current_iteration<max_iterations) {
   print_paramters();
    cerr << "Iteration: " << current_iteration << endl;
    least_square_optimize();
    ++current_iteration;
    cerr << " chi: " << chi_f << endl;

  }
}
template <typename T>
void calibrator::printvec(std::vector<T>* vectors){
for (int i = 0; i < (*vectors).size(); ++i){
     cout << (*vectors)[i]<<endl ;}
     cout<<endl;
     cout<<endl;

}


template <typename T>
void calibrator::printvec(std::shared_ptr< std::vector<T > > vectors){
for (int i = 0; i < (*vectors).size(); ++i){
     cout << (*vectors)[i]<<endl ;}
     cout<<endl;
     cout<<endl;

}
