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
if (argc<5) {
    cerr << "usage: " << environ[0] << " TRANSLATIONAL VELOCITY V, ROTATIONAL VELOCITY w,  FREQUENCY SIMULATION_PERIOD" << " NUM OF SCANS"<< endl ;
    return -1;
  }
              /** ODOMETRY PARAMETERS DEFINITION AND ASSIGNMENT **/
float period; float end_time = 0; float b = 5; float radius=2; float v;  float omega; float omega_r;  float omega_l; float sim_time;  int num_scans;
  cout<<" Med thesis version 2.0   : "<<period<<endl;
  v=atof(argv[1]);
  omega=atof(argv[2]);
  period=atof(argv[3]); cout<<" FREQUENCY  : "<<period<<endl;
  sim_time=atof(argv[4]); cout<<" SIMULATION_PERIOD  : "<<sim_time<<endl;
  num_scans=atoi(argv[5]); cout<<" NUM OF SCANS  : "<<num_scans<<endl;
/** POSE IS A STRUCT TO CONTAIN ROBOT'S POSE DATA AT EACH TIME TIME STEP **/   
position  pose;   

omega_r= v/radius + (b*omega)/(2*radius);
omega_l= v/radius - (b*omega)/(2*radius);

/** I USE SPACE_POINT AND TRAJECTORY AS A WAY TO HAVE A VECTOR OF EACH (X,Y) POSITION OF THE ROBOT **/  
Vector2f space_point; 
Vec_points trajectory,ray_points; 


pose.delta_x=0; pose.delta_y=0; pose.theta=0;  //INITIAL POSE OF ROBOT
float beam_angle = 0; 						   //INITIAL ANGLE OF LIDAR
int size_intersection =0;

/** CREATE FILES TO TRACK EVOLUTION OF THETA AND POLINOMIAL VALUES **/
string rays_file="rays.dat"; string lines_file="lines.dat"; string lines_file1="lines1.dat";
ofstream file0("theta.dat"); ofstream file1("poli.dat"); ofstream three_d("3d.dat");

/**_________________   CREATE LINES IN ENVIRONMENTS  __________________________________________________**/
ContainerType lines_points; //ONE SINGLE VECTOR OF (X,Y) POINTS TO STORE THE POINTS OF ALL THE LINES
lines_elements line_info;  // STRUCT CONTAINING 2 VECTORS TO STORE INFO OF ONE LINE EACH
std::vector<lines_elements> Vec_line_info; // VECTOR OF STRUCT TO STORE INFO OF ALL THE LINES CREATED
ofstream overwrite0(lines_file); overwrite0 <<" # "<< endl;

// x0,y0 = 10  d = 20
int x0,y0,d;
x0=0;y0 =0 ;  d = 25;
Vector2f p0(x0-d,y0-d),p1(x0-d,y0+d),p2(x0+d,y0-d),
							   p3(x0+d,y0+d),p4(x0-d,y0+d),p5(x0+d,y0+d), p6(x0-d,y0-d),p7(x0+d,y0-d);

mydrawLine( p0, p1, &line_info,lines_file, &Vec_line_info);
mydrawLine( p2, p3, &line_info,lines_file, &Vec_line_info);
mydrawLine( p4, p5, &line_info,lines_file, &Vec_line_info);
mydrawLine( p6, p7, &line_info,lines_file, &Vec_line_info);

lines_file="lines1.dat"; ofstream overwrite1(lines_file); overwrite1 <<" # "<< endl;
int x0_1,y0_1,d_1;
x0_1=13,y0_1 = 13;  d_1 = 2;
Vector2f p0_1(x0_1-d_1,y0_1-d_1),p1_1(x0_1-d_1,y0_1+d_1),p2_1(x0_1+d_1,y0_1-d_1),
							   p3_1(x0_1+d_1,y0_1+d_1),p4_1(x0_1-d_1,y0_1+d_1),p5_1(x0_1+d_1,y0_1+d_1);
mydrawLine( p0_1, p1_1, &line_info,lines_file, &Vec_line_info);
mydrawLine( p2_1, p3_1, &line_info,lines_file, &Vec_line_info);
mydrawLine( p4_1, p5_1, &line_info,lines_file, &Vec_line_info);


lines_file="lines2.dat"; ofstream overwrite2(lines_file); overwrite2 <<" # "<< endl;
int x0_2,y0_2,d_2;
x0_2=-13,y0_2 = -13;  d_2 = 4;
Vector2f p0_2(x0_2-d_2,y0_2-d_2),p1_2(x0_2-d_2,y0_2+d_2),p2_2(x0_2+d_2,y0_2-d_2),
							   p3_2(x0_2+d_2,y0_2+d_2),p4_2(x0_2-d_2,y0_2+d_2),p5_2(x0_2+d_2,y0_2+d_2);
mydrawLine( p0_2, p1_2, &line_info,lines_file, &Vec_line_info);
mydrawLine( p2_2, p3_2, &line_info,lines_file, &Vec_line_info);
mydrawLine( p4_2, p5_2, &line_info,lines_file, &Vec_line_info);




Vector2f s; // vector of THE scalars for which the lines intersect
Vector2f ray_dir; // vector FOR  direction of the ray
Vector2f point_intersect; // point at which ray and line intersects
float range = 0; // distance measured by lidar, initialize to zero because I pass it as pointer whoose value is contrained
Vector2fVector intersections; // vector of vectors of the scalars, I need this this to take only the intersection closest to the robot 
float min_scalar=0;
string micp_file="micp_file.dat";
ofstream overwrite(micp_file);
overwrite <<" # "<< endl;
ofstream os("endpoints_sim");
int scans=0; // number of scans to monitor hw many scans i make progressively
float sense_time=0;
while(scans < num_scans){
	
	if (size_intersection % 360 ==0){
		pose.theta= delta_theta(radius, omega_r , omega_l, (scans*period),b);
		pose.delta_x= delta_s(radius, omega_r, omega_l, (scans*period))*polinomial(pose.theta,'x');
		pose.delta_y= delta_s(radius, omega_r, omega_l,(scans*period))*polinomial(pose.theta,'y'); 
		//sense_time=end_time;
		//file0 << end_time << ' ' << pose.theta << endl; file1 << polinomial(pose.theta,'x') << ' ' << polinomial(pose.theta,'y') << endl; three_d<< end_time <<' ' << pose.delta_x <<  ' '<< pose.delta_y << endl;
		scans++;
	}
	space_point(0)=pose.delta_x; space_point(1)=pose.delta_y;
	ray_dir=create_ray(&pose, (beam_angle+pose.theta)); // cast a ray at current position and return vector of its direction
	
	 cout<< "x : " << pose.delta_x << " y : " << pose.delta_y <<endl;  //cout<< "I AM RAY_DIRECTION IN MAIN() -----> " << ray_dir <<endl;
	for (auto& elem: Vec_line_info  ){ //at current position  for each line
		s=calc_intersection(elem.line_p0,elem.line_dir, &pose,  ray_dir ); // calculate scalar for which ray should intersect with each line
		//std::cout << "The intersection scalars in MAIN FUNCTION ARE :\n" << s.transpose() << std::endl;	
		if(s(0)>= 0 && s(0)<=1 && s(1)>=0){
			intersections.push_back(s);}
		
	}//end of for
	if (intersections.size()>0){size_intersection++;

		point_intersect=cast_ray(&pose,ray_dir,ray_points,rays_file,intersections,&range, &min_scalar); // calculate points of intersection, take shortest range and (draw ?)
		intersections.clear(); // IMPORTANT TO CLEAR THIS VECTOR, EFFING IMPORTANT !!!!!
		//cout<< "I AM POINT OF INTERSECTION IN MAIN() " << point_intersect <<endl;
		Filewrite_for_micp(micp_file, ((scans)*period), &pose ,  beam_angle , range , point_intersect, min_scalar );   // write to file for micp  }
		if(   point_intersect(0) !=pose.delta_x){ 

			os << point_intersect(0)<< ' ' <<  point_intersect(1) << endl;  
		}              
																				

	} // size_intersection  if
	trajectory.emplace_back(space_point); 
	end_time+=period;
	cout << "Time: " << end_time <<"s" <<endl;
	beam_angle+=0.018;
	} //END OF WHILE



	file0.close();

     int cnt=0;
     ofstream file("data.dat");
     for (auto& elem: trajectory){  
      cnt ++;
      cout<< cnt << ": each element : " << elem << endl;
      file << elem(0) << ' ' << elem(1) << endl; }

 file.close();

cout<< "number of effective scans " << size_intersection <<endl; 
  
  

  
  
}


