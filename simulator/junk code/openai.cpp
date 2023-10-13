#include <iostream>
#include <cmath>
#include <Eigen/Dense>

// Define the number of data points and the dimensions of the input and output vectors
const int N = 10;
const int M = 2;
const int P = 2;

// Define the input and output vectors
Eigen::MatrixXd X(N, M);
Eigen::MatrixXd Y(N, P);

int main()
{


float x=0;
float y=-0;
if (x==y){
std::cout<< " yes 0 is seen as -0 \n"; }

else if (x!=y){
std::cout<< " no 0 is seen differently as -0 \n"; 
}

  // Set the values of the input and output vectors
  for (int i = 0; i < N; i++)
  {
    X(i, 0) = i;
    X(i, 1) = i * i;
    Y(i, 0) = sin(i);
    Y(i, 1) = cos(i);
  }

  // Use the least squares method to find the optimal parameters
  Eigen::MatrixXd theta = (X.transpose() * X).ldlt().solve(X.transpose() * Y);

  // Print the optimal parameters
  std::cout << "theta = " << theta << std::endl;

  return 0;
}

