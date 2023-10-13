//#include "eigen_icp_2d.h"
#include "Eigen/Geometry"
#include "rotations.h"
#include "Eigen/Cholesky"
#include <iostream>
using namespace std;

 int main(int argc, char const *argv[]) {
  Eigen::Matrix<float, 3, 3> H;
  Eigen::Matrix<float, 3, 3> sum;
  H.setIdentity();
  sum.setZero();
  cout << H<< endl;
  cout << sum << endl<<endl;;
    for(int i=0; i <5; i++){
sum+=H;

    }

      cout <<"sum : "<<  sum << endl;
  return 0;

}
