#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "rotations.h"
#include "simulator.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <iostream>
#include <cmath>
#include <string>

using namespace std;
using Vector2f=Eigen::Vector2f;
using Vector2fVector=std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >;

void write_to_file(string filename, string text){
	ofstream file(filename);
	file<< text;
	file.close();
	}

float polinomial(float theta, char dir){
 float approx;
 if (theta <= 1 && theta >=-1){
 if(dir =='x'){
  approx = 1 - ( pow(theta,2) )/6  + ( pow(theta,4) )/120 - ( pow(theta,6) )/5040 ;
  }
 else if (dir =='y'){
  approx = theta/2 - ( pow(theta,3) )/24  + ( pow(theta,5) )/720 ;
  }
}
  else {
  if(dir =='x'){
  approx = sin(theta)/theta ;
  }
 else if (dir =='y'){
  approx = (1-cos(theta))/theta;
  }
}
  return approx;}
  
  
 
 
float theta(int tick_r, int tick_l,float k_l, float k_r, float b){
 float theta = (k_l *tick_l) - (k_r * tick_r);
 cout<< "I am IN FUNCTION THETA ! : " << theta << endl ;
 theta = theta/b;
 
 return theta;
  }
  
float  x(int tick_r, int tick_l,float k_l, float k_r, float b,float theta){
 float x =  ( (k_l *tick_l) + (k_r * tick_r) )/2;
 x = x * polinomial(theta,'x');
 return x;
  }
  
float  y( int tick_r,int tick_l,float k_l, float k_r, float b,float theta){
 float y =  ( (k_l *tick_l) + (k_r * tick_r) )/2;
 y = y * polinomial(theta,'y');
 return y;
  }
  
 float delta_s(float r, float omega_r, float omega_l, float T) {
	 float delta_s = (r*omega_r*T)+ (r*omega_l*T);
	 delta_s=delta_s/2;
	 return delta_s;
	 }
float delta_theta(float r, float omega_r, float omega_l, float T,float b) {
	 float delta_th = (r*omega_r*T) - (r*omega_l*T);
	 delta_th=delta_th/b;
	 return delta_th;
	 }	 
	 
std::vector<float> quad_solver(float a, float b, float c){
           std::vector<float> sol;
          float  discriminant, x1, x2;
        discriminant = b*b - 4*a*c;
    
    if (discriminant >= 0) {
        x1 = (-b + sqrt(discriminant)) / (2*a);
        x2 = (-b - sqrt(discriminant)) / (2*a);
        if(x1==x2){ sol.push_back(x1);}
        else{
        sol.push_back(x1); sol.push_back(x2);}
    }
    return sol;
}

void map_to_file(string filename, Vector2fVector& vec_of_points){  // write 
	ofstream file(filename);
    int riga =0;
    for (auto& elem: vec_of_points){  
    riga ++;
     file << elem(0) << ' ' << elem(1) << endl;
					if(filename=="rays.dat" && riga %2==0){
								file << endl;  }
		      }
     
 file.close();

	}

Vector2f create_ray(position *Pose_robot, float ray_angle){
	
  Eigen::Vector2f vector_direction,P_robot;        															  
  vector_direction(0)=cos(ray_angle); 											           
  vector_direction(1)= sin(ray_angle);
    
	  return vector_direction;  
		}
		
		
Vector2f cast_ray(position *Pose_robot, Vector2f ray_dir, Vector2fVector& ray_points, string filename, Vector2fVector intersections, float *range_, float *min_scalar_){	
	
	Eigen::Vector2f P_robot, point_intersect;     // declare variable for robots pose and point of intersection     															  //cout << "______________" << endl;  cout << "I AM DELTA : " << delta << endl;
    float range;   // declare variable for distance between robots pose and point of intersection in environment_line
    std::vector<float> ranges; // this vector is to stores all the ranges of the rayS starting from a specific robots pose intersecting with the lines
     
    P_robot(0)=Pose_robot->delta_x; P_robot(1)=Pose_robot->delta_y; 
	float const minimum_range = 10;	
	float const max_range = 100;
	
	for (auto& elem:intersections){ // for each vector (intersection scalars) in intersections
		point_intersect=P_robot+ray_dir*elem(1); // calculate the point of the intersection, utilize elem(0) if you wanted to use the env_line equation instead
		//cout<<"SCALARS INTERSECT -> "<<elem << " POINT INTERSECT : ---> " << point_intersect << endl;
		range = norm2d(P_robot,point_intersect); // calculate the norm or distance or range
		ranges.push_back(range); // store each range in ranges..
		}
		
	float min_range= *min_element(ranges.begin(), ranges.end()); // take minimum range in ranges
	for (auto& elem:intersections){     // with mnnum range, reverse engineer the point of intersection that maps to it
		if( norm2d(P_robot,P_robot+ray_dir*elem(1)) == min_range){
			//cout<<"SCALAR for minimum range -> "<<elem ; 
      *min_scalar_ = elem(1);
			point_intersect = P_robot+ray_dir*elem(1); }
			
		}
	
	if (norm2d(P_robot,point_intersect)<= max_range && norm2d(P_robot,point_intersect)>= minimum_range){
		ray_points.push_back(P_robot);
		ray_points.push_back(point_intersect);   
		*range_ = norm2d(P_robot,point_intersect); }
		else{
			point_intersect = P_robot;
			*range_ =0;
			}
      map_to_file(filename, ray_points);
      
       return point_intersect;
       
	}	

	
	
	
void drawLine(Vector2fVector& dest, const Vector2f& p0, const Vector2f& p1,float density, lines_elements *line_info) { 
		
																							cout << "I AM p0 : " << p0 << endl; cout << "______________" << endl; cout << "I AM p1 : " << p1 << endl; 
  Vector2f delta=p1-p0; 																	cout << "______________" << endl;  cout << "I AM DELTA : " << delta << endl;
  float length=delta.norm();   																 cout << "______________" << endl;  cout << "I AM NORM : " << length << endl;
  int n_points=length*density;  
  Vector2f d=delta/n_points;  
  line_info->line_p0 = p0;
  line_info->line_dir = d;       														cout << "______________" << endl;  cout << "I AM THE DIRECTION : " << d << endl;
  for (int i=0; i<n_points; ++i) 
    dest.push_back(p0+d*float(i));
  
}
	

Vector2f calc_intersection(Vector2f P0, Vector2f line_dir, position *Pose_robot,  Vector2f ray_dir ){ //returns 1x2 vector of intersections
	// P0 and line_dir are the lines/obstacles to sense in the world parameter
	// Pose_robot and ray_dir are the ray_parameters
	
	 Eigen::Matrix2f A;
   Eigen::Vector2f b;
   A << line_dir(0),-ray_dir(0), 
        line_dir(1),-ray_dir(1);
   
   b << Pose_robot->delta_x -P0(0), Pose_robot->delta_y -P0(1);
   
   //std::cout << "Here is the matrix A:\n" << A << std::endl;
   //std::cout << "Here is the vector b:\n" << b << std::endl;
   Eigen::Vector2f s;
   if(A.determinant()==0){s(0)=0; s(1)=0;}
   else{
   s = A.colPivHouseholderQr().solve(b);}
  //std::cout << "The intersection scalars are :\n" << s << std::endl;	
   
   return s;
	
	}


float norm2d(Vector2f Start, Vector2f End){
	  float range= pow(Start(0)-End(0) ,2) + pow(Start(1)-End(1),2);
	  range = pow(range,0.5);
	  return range; 
	}

float norm2d(float x1, float y1, float x2, float y2){
	  float range= pow(x1-x2 ,2) + pow(y1-y2,2);
	  range = pow(range,0.5);
	  return range; 
	}



void mydrawLine(Eigen::Vector2f P0,Eigen::Vector2f P1,lines_elements *line_info, std::string file_name, std::vector<lines_elements> *Vec_line_info  ){
line_info->line_dir = P1 - P0;
line_info->line_p0 =P0;
(*Vec_line_info).push_back(*line_info);
ofstream ov(file_name, std::ios_base::app);
ov << P0.transpose()<<endl;
ov<< P1.transpose()<<endl;
ov << endl;


}


void Filewrite_for_micp(string filename, float time_stamp, position *Pose_robot, float beam_angle, float range, Vector2f& point_intersect, float min_scalar ){  // write 
	
	ofstream file(filename, std::ios_base::app);
    
    if(norm2d(Pose_robot->delta_x,Pose_robot->delta_y, point_intersect(0),point_intersect(1) ) == range){
     file << time_stamp << ' ' << Pose_robot->delta_x << ' ' << Pose_robot->delta_y << ' ' << Pose_robot->theta << ' ' << (beam_angle)<< ' ' << range << ' ' << point_intersect(0) << 
      ' ' << point_intersect(1) << ' ' << min_scalar <<  endl;
				}
     
 file.close();

	}















