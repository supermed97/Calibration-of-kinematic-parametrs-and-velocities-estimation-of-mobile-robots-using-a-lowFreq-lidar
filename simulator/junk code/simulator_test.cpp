#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "rotations.h"
#include "simulator.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
using Vector2f = Eigen::Vector2f; // 2x1 vector of floats
using Vec_points= std::vector<Vector2f, Eigen::aligned_allocator<Vector2f> >; // vector of (x,y) points
using Vector4f = Eigen::Vector4f;
using Vector2fVector=std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >;
using ContainerType = std::vector<Vector2f, Eigen::aligned_allocator<Vector2f> >;




using namespace std;
extern const char ** environ;

int main(int argc, char** argv) {
/**   INSERT WHHEL VELOCITIES AND SIM TIME FROM TERMINAL **/
if (argc<4) {
    cerr << "usage: " << environ[0] << " RIGHT_WHEEL_VEL LEFT_WHEEL_VELOCITY FREQUENCY SIMULATION_PERIOD" << endl ;
    return -1;
  }
              /** ODOMETRY PARAMETERS DEFINITION AND ASSIGNMENT **/
float period; float end_time = 0; float b = 5; float omega_r;  float omega_l; float sim_time; 
 
  omega_r=atof(argv[1]);
  omega_l=atof(argv[2]);
  period=atof(argv[3]); cout<<" FREQUENCY  : "<<period<<endl;
  sim_time=atof(argv[4]); cout<<" SIMULATION_PERIOD  : "<<sim_time<<endl;
/** POSE IS A STRUCT TO CONTAIN ROBOT'S POSE DATA AT EACH TIME TIME STEP **/   
position  pose;   

/** I USE SPACE_POINT AND TRAJECTORY AS A WAY TO HAVE A VECTOR OF EACH (X,Y) POSITION OF THE ROBOT **/  
Vector2f space_point; 
Vec_points trajectory,ray_points; 


pose.delta_x=0; pose.delta_y=0; pose.theta=0;  //INITIAL POSE OF ROBOT
float beam_angle = 0; 						   //INITIAL ANGLE OF LIDAR

/** CREATE FILES TO TRACK EVOLUTION OF THETA AND POLINOMIAL VALUES **/
string rays_file="rays.dat"; string lines_file="lines.dat";
ofstream file0("theta.dat"); ofstream file1("poli.dat"); ofstream three_d("3d.dat");

/**_________________   CREATE LINES IN ENVIRONMENTS  __________________________________________________**/
ContainerType lines_points; //ONE SINGLE VECTOR OF (X,Y) POINTS TO STORE THE POINTS OF ALL THE LINES
lines_elements line_info;  // STRUCT CONTAINING 2 VECTORS TO STORE INFO OF ONE LINE EACH
std::vector<lines_elements> Vec_line_info; // VECTOR OF STRUCT TO STORE INFO OF ALL THE LINES CREATED
  for (int i=0; i<3; ++i) {
    Vector2f p0=(Vector2f::Random()-Vector2f(-0.5, -0.5))*100; 
    Vector2f p1=(Vector2f::Random()-Vector2f(-0.5, -0.5))*100;
    drawLine(lines_points, p0, p1,15,&line_info); // create or draw a line in the environment and store its data in line_info, lines_points contains points of all lines
    Vec_line_info.push_back(line_info); // store info of all lines in a struct each 
  } 
    map_to_file(lines_file, lines_points);	// write points of lines created to file for visualization
	
Vector2f s; // vector of THE scalars for which the lines intersect
Vector2f ray_dir; // vector FOR  direction of the ray
Vector2f point_intersect; // point at which ray and line intersects
float range; // distance measured by lidar
Vector2fVector intersections; // vector of vectors of the scalars, I need this this to take only the intersection closest to the robot 

while(end_time <= sim_time){
	beam_angle+=5;
	pose.delta_x= delta_s(2, omega_r, omega_l, end_time)*polinomial(pose.theta,'x');
	pose.delta_y= delta_s(2, omega_r, omega_l, end_time)*polinomial(pose.theta,'y'); cout << "OMEGA_R: " << omega_r <<" OMEGA_L : " << omega_l <<endl;
	pose.theta= delta_theta(2, omega_r , omega_l, end_time,b); file0 << end_time << ' ' << pose.theta << endl; file1 << polinomial(pose.theta,'x') << ' ' << polinomial(pose.theta,'y') << endl; three_d<< end_time <<' ' << pose.delta_x <<  ' '<< pose.delta_y << endl;
	space_point(0)=pose.delta_x; space_point(1)=pose.delta_y;
														if(beam_angle<=360){
	ray_dir=create_ray(&pose, (beam_angle*M_PI)/180); // cast a ray at current position and return vector of its direction
	 cout<< "x : " << pose.delta_x << " y : " << pose.delta_y <<endl;  cout<< "I AM RAY_DIRECTION -----> " << ray_dir <<endl;
	for (auto& elem: Vec_line_info  ){ //at current position  for each line
		s=calc_intersection(elem.line_p0,elem.line_dir, &pose,  ray_dir ); // calculate scalar for which ray should intersect with each line
		intersections.push_back(s); }//end of for
		cout<< "I AM s -----> " << s <<endl;
		point_intersect=cast_ray(&pose,ray_dir,ray_points,rays_file,intersections); // calculate points of intersection, take shortest range and (draw ?)
		
		                 
																					    } // end of if
	trajectory.emplace_back(space_point); 
	end_time+=period;
	cout << "Time:" << end_time <<"s" <<endl;
	} //END OF WHILE
	file0.close();

     int cnt=0;
     ofstream file("data.dat");
     for (auto& elem: trajectory){  
      cnt ++;
      cout<< cnt << ": each element : " << elem << endl;
     file << elem(0) << ' ' << elem(1) << endl; }
 file.close();

  
  

  
  
}


