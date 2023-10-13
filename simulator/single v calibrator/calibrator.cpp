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

Vector3f calibrator::calc_error(float tick_l_i, float tick_r_i, float t_i){ //define x,yand theta
float omega_e=theta(tick_r_i, tick_l_i,k_l,k_r,b);

float v_x= v*polinomial(omega*t_i,'x');
float v_y= v*polinomial(omega*t_i,'y');
cout<<" tick_l is "<<tick_l_i<<endl;
cout<<" tick_r is "<<tick_r_i<<endl;
//Eigen:: Vector3f error;
error(0)= -(v_x*t_i)+ x(tick_r_i,tick_l_i,k_l,k_r,b,omega_e);
error(1)= -(v_y*t_i)+ y(tick_r_i,tick_l_i,k_l,k_r,b,omega_e);
error(2)= -(omega*t_i) +omega_e ;
cout << "error at time " << t_i << endl<<  error << endl;
return error;
}

Eigen::Matrix3f calibrator::computeJacobian(float tick_l_i, float tick_r_i, float t_i){  // jacobian at time i
Eigen::Matrix<float, 3, 3> J;
float omega_e=theta(tick_r_i, tick_l_i,k_l,k_r,b);
J <<

    polinomial(omega_e*t_i, 'x') *tick_r_i/2, polinomial(omega_e*t_i, 'x') *tick_l_i/2 ,           0,
     polinomial(omega_e*t_i, 'y') *tick_r_i/2, polinomial(omega_e*t_i, 'y') *tick_l_i/2 ,           0 ,
     tick_r_i/b,                            -tick_l_i/b,                              -( (k_r*tick_r_i) - (k_l*tick_l_i) )/ (b*b);
return J;
}
/**
Vector3f sum_error(){

for(auto& time_i: (*timestamps)){
  Eigen::Vector3f err;
	err =calc_error(time_i);
  chi+=err;
                    }
  return chi;
}
**/
Eigen::Matrix3f calibrator::calc_H(Eigen::Matrix3f * H, Eigen::Vector3f* b) {
  chi_f=0;
  float chi;
  Eigen::Matrix3f J;
  Eigen::Vector3f b_,err,sum_err;
  J.setZero();
  b_.setZero();
  err.setZero();
  sum_err.setZero();
  for (int i = 0; i < (*ticks_l).size(); ++i){
      J=calibrator::computeJacobian((*ticks_l)[i],(*ticks_r)[i],(*timestamps)[i] );
      cout<< "Jacobian at time : "<< (*timestamps)[i] << endl;
      cout<< J << endl;
      err=calibrator:: calc_error((*ticks_l)[i],(*ticks_r)[i],(*timestamps)[i]);
      sum_err+=err;
      cout<< " summed error till time : "<< (*timestamps)[i] << endl;
      cout << sum_err<< endl;
      chi=err.squaredNorm();
      chi_f+=chi;
      *H+=(J*covariance*J.transpose() );
      *b+=(J*covariance*err);
  }
  return J;
}



void calibrator::least_square_optimize() {
  /**
  float* k_l_ptr = & k_l;
  float* k_r_ptr =& k_r
  float* b = & b;**/
  Eigen::Matrix<float, 3, 3> H;
  Eigen::Matrix<float, 3, 1> b_ls;
  H.setZero();
  b_ls.setZero();
  calc_H(&H,&b_ls);
  _dx=H.ldlt().solve(-b_ls);

cout<<"delta_k below "<< endl << endl;
cout << _dx << endl << endl;
  ///**
 k_r+=_dx(0);
 k_l+= _dx(1);
 b+= _dx(2); //**/
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

void calibrator::printvec(std::vector<auto>* vectors){
for (int i = 0; i < (*vectors).size(); ++i){
     cout << (*vectors)[i]<<endl ;}
     cout<<endl;
     cout<<endl;

}



void calibrator::printvec(std::shared_ptr< std::vector<auto > > vectors){
for (int i = 0; i < (*vectors).size(); ++i){
     cout << (*vectors)[i]<<endl ;}
     cout<<endl;
     cout<<endl;

}

/**
void ICP::draw(std::ostream& os) {
  os << "set size ratio -1" << endl;
  os << "plot '-' w p ps 2 title \"fixed\", '-' w p ps 2 title \"moving\", '-' w l lw 1 title \"correspondences\" " << endl;
  for  (const auto& p: _fixed)
    os << p.transpose() << endl;
  os << "e" << endl;
  for  (const auto& p: _moving)
    os << (_X*p).transpose() << endl;
  os << "e" << endl;
  for (const auto& c: _correspondences) {
    os << c._fixed.transpose() << endl;
    os << c._moving.transpose() << endl;
    os << endl;
  }
  os << "e" << endl;
}
**/
