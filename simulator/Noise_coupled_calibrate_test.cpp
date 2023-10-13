#include "eigen_micp_2d_utils.h"
#include "rotations.h"
#include <iostream>
#include <fstream>
#include <limits>
#include "eigen_micp_2d.h"
#include <sstream>
#include <string>
#include <vector>
#include <random>
using namespace std;
extern const char ** environ;

template<typename T, typename U, typename V>
void add_noise(std::vector<T>& data, U mean, V stddev) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<T> dist(mean, stddev);

    for (auto& x : data) {
        x += dist(gen);
        //cerr<<x;
    }

}


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
}



int main(int argc, char** argv) {
if (argc<3) {
    cerr << "usage: initial values for "<< " k_r k_l b" << endl;
    return -1;
  }

float k_r=atof(argv[1]); float k_l=atof(argv[2]); float b = atof(argv[3]); 
 double mean, stddev;
    std::cerr << "Enter mean: " <<endl;
    std::cin >> mean;
    std::cerr << "Enter standard deviation: "<<endl;
    std::cin >> stddev;

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
  add_noise(ranges,mean,stddev);


  int num_scans = ranges.size()/360;
  cerr<<" num of scans : "<<num_scans<<endl;
  vector<float> ticks_r,ticks_l,tempi;
  string ticks_file="abs_ticks_file.dat";
  read_ticks(ticks_file,&ticks_r,&ticks_l,&tempi); 
std::vector<std::vector<float>> mul_ranges=divideVector(ranges,num_scans,360);
  std::vector<std::vector<float>> mul_beam_angles=divideVector(beam_angles,num_scans,360);
  std::vector<std::vector<double>> mul_stamps=divideVector(stamps,num_scans,360);
  std::vector<std::vector<float>> mul_ticks_r=divideVector(ticks_r,num_scans,1);
  std::vector<std::vector<float>> mul_ticks_l=divideVector(ticks_l,num_scans,1);
  std::vector<std::vector<float>> mul_tempi=divideVector(tempi,num_scans,1);
  
  cerr<<" vals : "<< mul_ticks_r[1][0]<<endl;

int i=1;
  MICP2D micp;
  string  c;
  // tv=0, rv=0;
 // micp.X().setZero();

ranges.clear(); beam_angles.clear(); stamps.clear();
 getElements_(mul_ranges,0,1,ranges);
 getElements_(mul_beam_angles,0,1,beam_angles);
 getElements_(mul_stamps,0,1,stamps);

 

  micp.setMeasurement (ranges, stamps, beam_angles);
  micp.set_ticks(ticks_r,ticks_l,tempi);
  micp.set_init(k_r,k_l,b);
  micp.control=0;
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
       //i++;
      micp.setMeasurement (ranges, stamps, beam_angles);
      micp.update_initial_matrices(); //save the last h e b matrix when switching scans
      micp.set_ticks(mul_ticks_r[i+1],mul_ticks_l[i+1],mul_tempi[i+1]);
     i++;
      
    
    }
  }
 
  
}
/**
 * The mean and standard deviation of the noise affect the data in the following ways:

    Mean: The mean of the noise determines the average value of the noise that will be added to each data point. 
    If the mean of the noise is positive, it will shift the entire dataset upward, 
    while a negative mean will shift it downward. If the mean is zero, 
    then the noise will be centered around zero and will not have a systematic effect on the dataset.

    Standard deviation: The standard deviation of the noise determines the 
    amount of variation in the noise that will be added to each data point. 
    A larger standard deviation will result in noisier data with greater variability, 
    while a smaller standard deviation will result in less noisy data with less variability. 
    If the standard deviation is zero, then no noise will be added to the data.

In summary, the mean and standard deviation of the noise determine the amount and type of noise that will be added to the dataset, and can have a significant effect on the resulting data.
 * 
*/

  




