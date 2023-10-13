#include "eigen_micp_2d_utils.h"
#include "rotations.h"
#include <iostream>
#include <fstream>
#include <limits>
#include "eigen_micp_2d.h"
#include <sstream>
#include <string>
#include <vector>

using namespace std;
extern const char ** environ;

int main(int argc, char** argv) {
  /**____________________ PROF SECTOR ______________________________________**/
  using ContainerType=Vector2fVector;
   
  cout << "set size ratio -1" << endl;
  cout << "set term x11 0" << endl;
  cout << "set size ratio -1" << endl;
  cout << "set term x11 1" << endl;  
  /***_______________________________________________________ ***/
    
  /****FILL VECTORS FOR MICP FROM MY SIMULATOR *****/
  double time_stamp;
  float  x,y,theta;   // VARIABLES TO STORE DATA OF EACH LINE TEMPORARILY
  float beam_angle, range;       // VARIABLES TO STORE DATA OF EACH LINE TEMPORARILY
  string line;                  // VARIABLE TO READ LINE
  ifstream input_File;         // OBJECT TO OPEN FILE AND ACCESS IT
  input_File.open("micp_file.dat"); // USE OBJECT TO OPEN FILE
    
  if(input_File.fail()) { 
    cout << "error COULDNT OPEN FILE " << endl; 
    return 1; // no point continuing if the file didn't open...
  } 
    
  std::vector<double> stamps;      // work vector 1
  std::vector<float> beam_angles; // work vector 2    
  std::vector<float> ranges;      // work vector 3
  //std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > endpoints;
  ofstream os("endpoints");
  while (std::getline(input_File, line)) {
    std::stringstream ss(line);   // create ss object to split line based on space
    // save each value in a variable
    if (ss >> time_stamp >> x >> y >> theta >> beam_angle >> range) {
      if(range !=0){
      cerr<<time_stamp << ' ' << beam_angle <<' ' << range << endl;  // print for control
      // push each element in their appropriate vector
      stamps.push_back(time_stamp); beam_angles.push_back(beam_angle); ranges.push_back(range);
      Eigen::Isometry2f iso;
      iso.linear() << cos(theta), -sin(theta), sin(theta), cos(theta);
      iso.translation() << x, y;
      Eigen::Vector2f p(range*cos(beam_angle), range*sin(beam_angle));
      p=iso*p;
      os << p.transpose() << endl;}
    }
  } //end of while
  
  
 
  MICP2D micp;
  // tv=0, rv=0;
 // micp.X().setZero();
  micp.setMeasurement (ranges, stamps, beam_angles);
  while (1) {
    micp.oneRound();
    // char c;
    // cin >> c;
  }
  
  
}



