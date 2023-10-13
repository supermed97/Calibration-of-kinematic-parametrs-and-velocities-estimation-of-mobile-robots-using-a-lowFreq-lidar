#include "calibratemul.h"
#include "../simulator.h"
#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "../rotations.h"
#include <iostream>
#include <fstream>

using namespace std;
extern const char ** environ;
int main(int argc, char** argv) {

    if (argc<4) {
    cerr << "usage: " << " number of iterations, k_r, k_l, b" << endl;
    return -1;
  }


float velocity,vel;
float w;
vector<float> tv;
vector<float> rv;
string line_vel;
ifstream vel_File;
vel_File.open("velocities.dat");
while (std::getline(vel_File, line_vel)) {
    std::stringstream ss(line_vel);   // create ss object to split line based on space

    if (ss >> velocity >> w) {
         tv.push_back(velocity);
         rv.push_back(w);

    }
}


float tick_r, tick_l;
string line;
ifstream input_File;
input_File.open("mulmotion_ticks_file.dat");


int iterations=atoi(argv[1]);
float k_r=atof(argv[2]); float k_l=atof(argv[3]); float b=atof(argv[4]);
float time;
vector<vector<float>> mticks_r, history;
vector<vector<float>> mticks_l;
vector<float> timestamps,timestamps2,timestamps3;
vector<float> ticks_r1, ticks_l1, ticks_r2, ticks_l2, ticks_r3, ticks_l3;
float v=tv[0]; float omega=rv[0];
float v2=tv[1]; float omega2=rv[1];
float v3=tv[2];  float omega3=rv[2];

float tottime;
  if(input_File.fail()) {
    cerr << "error COULDNT OPEN FILE " << endl;
    return 1; // no point continuing if the file didn't open...
  }

   while (std::getline(input_File, line)) {
    std::stringstream ss(line);   // create ss object to split line based on space

    if (ss >> tick_r >> tick_l>> tottime>> time >> vel >> w) {

    //  cerr<< tick_r << ' ' << tick_l << ' '<< vel << ' ' << w<<endl;  // print for control
      if (vel==v && w==omega){
          ticks_r1.push_back(tick_r); ticks_l1.push_back(tick_l); // push each element in their appropriate vector
           timestamps.push_back(time);
      }
      else if (vel==v2 && w == omega2){
          ticks_r2.push_back(tick_r); ticks_l2.push_back(tick_l); // push each element in their appropriate vector
           timestamps2.push_back(time);

      }
     else if (vel==v3 && w==omega3){
              ticks_r3.push_back(tick_r); ticks_l3.push_back(tick_l); // push each element in their appropriate vector
              timestamps3.push_back(time);
     }

    }
  } //end of while
 mticks_r={ticks_r1, ticks_r2,ticks_r3};
 mticks_l={ticks_l1,ticks_l2,ticks_l3};
 history={timestamps,timestamps2,timestamps3};
vector<vector<float>> parameters = {tv,rv};


cout <<" size of t1: "<< mticks_r[0].size()<<endl ;
cout <<" size 0f t2: "<< mticks_r[1].size()<<endl ;
cout <<" size of t3 : "<< mticks_r[2].size()<<endl ;
cerr  << " number of iterations : " << iterations << endl;
cout<<" v  : "<<v<< " omega : "<< omega<<endl;
cout<<" v2  : "<<v2<< " omega2 : "<< omega2<< endl;
cout<<" v3  : "<<v3<< " omega3 : "<< omega3<< endl;
cout<<" k_r : " << k_r << " k_l : " << k_l << " b : " << b <<endl;

calibrator calibrator( history,parameters,k_l, k_r, b);
calibrator.set_ticks(mticks_r,mticks_l);
calibrator.run(iterations);
calibrator.print_paramters();

Vector3f _dx, err3; Vector2f err;
err.setZero();
_dx(0)=calibrator.k_r;
_dx(1)= calibrator.k_l;
_dx(2)=calibrator.b ;
/**
for (int i = 0; i < ticks_l.size(); ++i){
float theta_e=theta(ticks_r[i],ticks_l[i],_dx(1),_dx(0),_dx(2));
err(0)+=  (s(ticks_r[i],ticks_l[i],_dx(1),_dx(0))/timestamps[i])- (v);
err(1)+=  (theta_e/timestamps[i]) - (omega);
}

cout<< " summed error is : " << endl;
cout << err << endl;
**/
return 0;


/**

for(auto & elem: ticks_r1){
     cout<< "ticks_r1 are : " << elem<< endl;
}

for(auto & elem: ticks_l1){
     cout<< "ticks_l1 are : " << elem<< endl;
}
for(auto & elem: ticks_r3){
     cout<< "ticks_r3 are : " << elem<< endl;
}
for(auto & elem: ticks_l3){
     cout<< "ticks_l3 are : " << elem<< endl;
}

for(auto & elem: timestamps){
     cout<< "time is : " << elem<< endl;
}

**/
}
