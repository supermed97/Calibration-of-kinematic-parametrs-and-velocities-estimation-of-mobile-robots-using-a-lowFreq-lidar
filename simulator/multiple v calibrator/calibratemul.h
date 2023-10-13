#pragma once
#include "Eigen/Geometry"
#include <iostream>
#include <list>
#include <memory> 

using floats=std::vector<float>;
using Vector2f = Eigen::Vector2f;

class calibrator {
public:
	float k_l;
	float k_r;
	float b;
	std::vector<std::vector<float>> ticks_l ;
	std::vector<std::vector<float>> ticks_r ;
	std::vector<std::vector<float>> timestamps;
	std::vector<float> v,omega;
   	Eigen::Vector3f error; // error at instant i
	Eigen::Vector2f error_2;
	Eigen::Vector3f chi; //chi is sum of error
	float chi_f;
  Eigen::Matrix3f sum_Jacobian;
	Eigen::Matrix3f covariance;
	Eigen::Matrix<float, 3,1> _dx;
// constructor
   calibrator(std::vector< std::vector<float> >& timestamps_, 
              std::vector< std::vector<float> >& parameters,
			  float k_l_, float k_r_, float b_);

  void set_ticks(std::vector< std::vector<float> >& ticks_r_,
                 std::vector< std::vector<float> >& ticks_l_);

	//Eigen::Vector3f calc_error(float tick_l_i, float tick_r_i, float t_i);

	Eigen::Vector2f calc_error_v(float tick_l_i, float tick_r_i, float t_i,float v_i, float omega_i);

	//Eigen::Matrix3f computeJacobian(float tick_l_i, float tick_r_i, float time_i);

	Eigen::Matrix<float, 2, 3> computeJacobian_v(float tick_l_i, float tick_r_i, float t_i,float v_i, float omega_i);

	Eigen::Matrix3f calc_H(Eigen::Matrix3f * H, Eigen::Vector3f* b);

	
  void least_square_optimize();

  void print_paramters();
template <typename T>
  void printvec(std::vector<T>& vectors);

  void run(int max_iterations);
template <typename T>
void printvec(std::shared_ptr< std::vector<T > > vectors);

template <typename T>
void printvec(std::vector<T>* vectors);




};


