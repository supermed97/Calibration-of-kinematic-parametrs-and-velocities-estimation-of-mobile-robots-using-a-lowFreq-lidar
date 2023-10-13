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
  
  if (argc<3) {
    cerr << "usage: " << environ[0] << " tv rv " << endl ;
    return -1;
  }
  
  float  tv, rv;
  tv=atof(argv[1]);  
  rv=atof(argv[2]);
   
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
    
     if(input_File.fail()) // checks to see if file opened 
    { 
      cout << "error COULDNT OPEN FILE " << endl; 
      return 1; // no point continuing if the file didn't open...
    } 
    
    std::vector<double> stamps;      // work vector 1
    std::vector<float> beam_angles; // work vector 2    
    std::vector<float> ranges;      // work vector 3
while (std::getline(input_File, line)) 
{
    std::stringstream ss(line);   // create ss object to split line based on space
   
            if (ss >> time_stamp >> x >> y >> theta >> beam_angle >> range) // save each value in a variable
    {
		   cerr<<time_stamp << ' ' << beam_angle <<' ' << range << endl;  // print for control
          stamps.push_back(time_stamp); beam_angles.push_back(beam_angle); ranges.push_back(range); // push each element in their appropriate vector
    }
} //end of while
  
  
 
  MICP2D micp;
  // tv=0, rv=0;
  micp.X().setZero();
  micp.setMeasurement (ranges, stamps, beam_angles);
  while (1) {
    micp.oneRound();
    // char c;
    // cin >> c;
  }
  
  
}



