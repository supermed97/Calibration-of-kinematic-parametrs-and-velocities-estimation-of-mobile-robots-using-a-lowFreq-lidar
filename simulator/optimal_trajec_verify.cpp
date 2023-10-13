#include "simulator.h"
#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "rotations.h"
#include <iostream>
#include <fstream>

using namespace std;
extern const char ** environ;
int main(int argc, char** argv) {

    if (argc<3) {
    cerr << "usage: " << " k_r, k_l, b" << endl;
    return -1;
  }

float tick_r, tick_l;
string line;                  // VARIABLE TO USE TO READ EACH LINE
ifstream input_File;         // OBJECT TO OPEN FILE AND ACCESS IT
input_File.open("ticks_file.dat"); // USE OBJECT TO OPEN FILE
    


float k_r=atof(argv[1]); float k_l=atof(argv[2]); float b=atof(argv[3]);
float time;
vector<float> timestamps;
vector<float> ticks_r, ticks_l;

  if(input_File.fail()) { 
    cerr << "error COULDNT OPEN FILE " << endl; 
    return 1; // no point continuing if the file didn't open...
  } 
    
   while (std::getline(input_File, line)) {
    std::stringstream ss(line);   // create ss object to split line based on space
    // save each value in a variable
    if (ss >> tick_r >> tick_l>>time) {
      
      cerr<< tick_r << ' ' << tick_l << endl;  // print for control
      if (time!=0){
          ticks_r.push_back(tick_r); ticks_l.push_back(tick_l); // push each element in their appropriate vector
           timestamps.push_back(time);
      }
    }
  } //end of while
  
float theta_;
float x_,y_;
ofstream file("opt_traj.dat");

for (int i = 0; i < ticks_l.size(); ++i){
  cout<< "tick_r : " << ticks_r[i] << " tick_l : " << ticks_l[i] << endl;
theta_=theta(ticks_r[i],ticks_l[i],k_l,k_r,b);
x_=x(ticks_r[i],ticks_l[i],k_l, k_r, b,theta_);
y_=y(ticks_r[i],ticks_l[i],k_l, k_r, b,theta_);
file <<x_ << ' ' << y_ << endl; 
}

cout <<  " k_r : " << k_r << " k_l : " << k_l << " b : " << b <<endl;
return 0;
}
