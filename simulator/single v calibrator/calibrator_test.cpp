#include "calibrator.h"
#include "../simulator.h"
#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "../rotations.h"
#include <iostream>
#include <fstream>

using namespace std;
extern const char ** environ;
int main(int argc, char** argv) {

    if (argc<6) {
    cerr << "usage: " << " number of iterations, v, omega, k_r, k_l, b" << endl;
    return -1;
  }

float tick_r, tick_l;
string line;                  // VARIABLE TO USE TO READ EACH LINE
ifstream input_File;         // OBJECT TO OPEN FILE AND ACCESS IT
input_File.open("../ticks_file.dat"); // USE OBJECT TO OPEN FILE


int iterations=atoi(argv[1]);
float v=atof(argv[2]);
float omega = atof(argv[3]);
float k_r=atof(argv[4]); float k_l=atof(argv[5]); float b=atof(argv[6]);
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

vector<float> parameters = {v,omega};

cerr  << " number of iterations : " << iterations << ", v : " << v << ", omega : "<< omega
      <<", k_l : "<<k_l << ", k_r : "<<k_r << ", b : "<<b << endl ;


calibrator calibrator( timestamps,parameters,k_l, k_r, b);
calibrator.set_ticks(ticks_r,ticks_l);
calibrator.run(iterations);
calibrator.print_paramters();

Vector3f _dx, err3; Vector2f err;
err.setZero();
_dx(0)=calibrator.k_r;
_dx(1)= calibrator.k_l;
_dx(2)=calibrator.b ;

for (int i = 0; i < ticks_l.size(); ++i){
float theta_e=theta(ticks_r[i],ticks_l[i],_dx(1),_dx(0),_dx(2));
err(0)+=  (s(ticks_r[i],ticks_l[i],_dx(1),_dx(0))/timestamps[i])- (v);
err(1)+=  (theta_e/timestamps[i]) - (omega);
}

cout<< " summed error is : " << endl;
cout << err << endl;

return 0;
}
