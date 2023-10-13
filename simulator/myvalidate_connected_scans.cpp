#include "eigen_micp_2d_utils.h"
#include "rotations.h"
#include <iostream>
#include <fstream>
#include <limits>
#include "eigen_orig_micp_2d.h"
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;
extern const char ** environ;

// function divides a vector of x elements into y subvectors , 
//the first  y-1 subvectors have z elements while the last subvector has  x-((y-1)*z )elements 
template <typename T>
std::vector<std::vector<T>> divideVector(std::vector<T>& vec, int y, int z) {
    int x = vec.size();
    std::vector<std::vector<T>> subVectors(y);

    int subVectorIndex = 0;
    for (int i = 0; i < x; i++) {
        subVectors[subVectorIndex].push_back(vec[i]);
        if (subVectors[subVectorIndex].size() == z && subVectorIndex < y - 1) {
            subVectorIndex++;
        }
    }

    return subVectors;
}
template <typename T>
std::vector<T> getElements(const std::vector<std::vector<T>>& vec2d, int i, int j) {
  std::vector<T> result;
  result.insert(result.begin(), vec2d[i].begin(), vec2d[i].end());
  result.insert(result.begin(), vec2d[j].begin(), vec2d[j].end());
  return result;
}
template <typename T>
void getElements_(const std::vector<std::vector<T>>& vec2d, int i, int j, std::vector<T>& result) {
  result.clear();
result.insert(result.begin(), vec2d[i].begin(), vec2d[i].end());
  result.insert(result.end(), vec2d[j].begin(), vec2d[j].end());
  /**
   //result.clear();
  //result.reserve(result.size() + vec2d[i].size() + vec2d[j].size());
  result.insert(result.begin(), vec2d[i].begin(), vec2d[i].end());
  result.insert(result.begin()+1, vec2d[j].begin(), vec2d[j].end()); **/
}
/**
in prev func you can also use : 
result.clear()
result.insert(result.begin(), vec2d[i].begin(), vec2d[i].end());
  result.insert(result.end, vec2d[j].begin(), vec2d[j].end());
  but prev scans get discarded
**/

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
     // cerr<<time_stamp << ' ' << beam_angle <<' ' << range << endl;  // print for control
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


int num_scans = ranges.size()/360;
  cerr<<" num of scans : "<<num_scans<<endl;

 std::vector<std::vector<float>> mul_ranges=divideVector(ranges,num_scans,360);
  std::vector<std::vector<float>> mul_beam_angles=divideVector(beam_angles,num_scans,360);
  std::vector<std::vector<double>> mul_stamps=divideVector(stamps,num_scans,360);
  
 int ITERATIONS=5;
  MICP2D micp;
  string  c;
 
 ranges.clear(); beam_angles.clear(); stamps.clear();
 getElements_(mul_ranges,0,1,ranges);
 getElements_(mul_beam_angles,0,1,beam_angles);
 getElements_(mul_stamps,0,1,stamps);

  micp.setMeasurement (ranges, stamps, beam_angles);
  micp.control=0;
  int i=1;
    while(1){
    micp.oneRound();
    cerr<<"_________"<<endl;
    cerr<<"insert c to change to next scans or  other character to remain with the current scans"<<endl;
    cerr<<"____________"<<endl;
    cin >> c;
    if(c=="c" && i<num_scans-1){
      cerr<<"associating scan" << i << " and scan"<< i+1<<endl;
      cerr<<"_____"<<endl;
      getElements_(mul_ranges,i,i+1,ranges);
      getElements_(mul_beam_angles,i,i+1,beam_angles);
      getElements_(mul_stamps,i,i+1,stamps);

      micp.control=1;
       i++;
      micp.setMeasurement (ranges, stamps, beam_angles);
     
    }
  }
 
  
}



