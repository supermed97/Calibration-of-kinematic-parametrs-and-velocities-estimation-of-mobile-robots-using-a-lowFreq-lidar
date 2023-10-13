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
    cerr << "usage: " << environ[0] << " translational_velocity Rotational_velocity integration_step" << " NUM OF SCANS"<< endl ;
    return -1;
  }
              /** ODOMETRY PARAMETERS DEFINITION AND ASSIGNMENT **/
float period; float end_time = 0; float b ; float v;  float omega; float sim_time;  int num_scans;
  cout<<" Med thesis version 2.0   : "<<period<<endl;
  v=atof(argv[1]);
  omega=atof(argv[2]);
  period=atof(argv[3]); cout<<" integration step _dt  : "<<period<<endl;
  //sim_time=atof(argv[4]); cout<<" SIMULATION_PERIOD  : "<<sim_time<<endl;
  num_scans=atoi(argv[4])+1; cout<<" NUM OF SCANS  : "<<argv[4]<<endl;
float k_r; float k_l; float radius = 0.6;

cout <<"_________________________________________"<<endl;
cout << "Insert k_r : " <<  endl; cin >> k_r;
cout << "Insert k_l: " <<  endl; cin >> k_l;
cout << "Insert b : " <<  endl; cin >>   b;
cout <<"_________________________________________"<<endl;

cerr<<" translational vel used in simulation : "<< v<< endl;
cerr<<" rotational vel used in simulation : "<< omega<< endl;

cout <<"_____________________________________________"<<endl;
cerr << "k_r: " << k_r << " k_l: " << k_l << " b : "<<  b<< endl;
cout <<"_________________________________________"<<endl;

vector<pair <float , float>> vel_pair;

float v2; float omega2;
cout << "Insert v2 : " <<  endl; cin >> v2;
cout << "Insert omega2 : " <<  endl; cin >> omega2;

float v3; float omega3;
cout << "Insert v3 : " <<  endl; cin >> v3;
cout << "Insert omega3 : " <<  endl; cin >> omega3;

float v4; float omega4;
cout << "Insert v4 : " <<  endl; cin >> v4;
cout << "Insert omega4 : " <<  endl; cin >> omega4;

float v5; float omega5;
cout << "Insert v5 : " <<  endl; cin >> v5;
cout << "Insert omega5 : " <<  endl; cin >> omega5;

vel_pair.push_back(std::make_pair(v,omega));
vel_pair.push_back(std::make_pair(v2,omega2));
vel_pair.push_back(std::make_pair(v3,omega3));
vel_pair.push_back(std::make_pair(v4,omega4));
vel_pair.push_back(std::make_pair(v5,omega5));

/** POSE IS A STRUCT TO CONTAIN ROBOT'S POSE DATA AT EACH TIME TIME STEP **/
position  pose;

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
string ticks="ticks_file.dat";  ofstream ticks_file(ticks); //absolute ticks
string abs_ticks="abs_ticks_file.dat";  ofstream abs_ticks_file(abs_ticks); //absolute ticks
ofstream overwrite(micp_file);
overwrite <<" # "<< endl;
ofstream os("endpoints_sim");
int scans=1; // number of scans to monitor hw many scans i make progressively
float sense_time=0;
int total_scans=0;
int scans_cp=1;
Vector2f init_tick(0,0);

for(int i=0; i<vel_pair.size() ; i++){
    float tick_r_=(vel_pair[i].first+(b*vel_pair[i].second/2))/k_r; //right tick
	float tick_l_=(vel_pair[i].first-(b*vel_pair[i].second/2))/k_l; // left tick
	cerr<<"using Pair : "<< i+1 << endl;
	cerr<<"v : "<< vel_pair[i].first <<" w: "<<vel_pair[i].second<<  endl;
		while(scans < num_scans){

			

				
				pose.theta= theta( (tick_r_*(scans*period))+init_tick(0), (tick_l_*(scans*period))+init_tick(1),k_l, k_r, b);
				pose.delta_x= x((tick_r_*(scans*period))+init_tick(0), (tick_l_*(scans*period))+init_tick(1),k_l, k_r, b,pose.theta);
				pose.delta_y= y((tick_r_*(scans*period))+init_tick(0), (tick_l_*(scans*period))+init_tick(1),k_l, k_r, b,pose.theta);
				ticks_file << (tick_r_*(scans*period))+init_tick(0) << ' ' << (tick_l_*(scans*period))+init_tick(1) <<' ' << scans_cp*period << endl  ;
				abs_ticks_file << (tick_r_*(scans*period)) << ' ' << (tick_l_*(scans*period)) <<' ' << scans*period << endl  ;
				cout<< "Robot poses while scanning "<<endl;
				cout<< "x : " << pose.delta_x << " y : " << pose.delta_y <<endl<<endl;
		
				
			space_point(0)=pose.delta_x; space_point(1)=pose.delta_y;
			ray_dir=create_ray(&pose, (beam_angle+pose.theta)); 
			for (auto& elem: Vec_line_info  ){ //at current position  for each line
				s=calc_intersection(elem.line_p0,elem.line_dir, &pose,  ray_dir ); 
				
				if(s(0)>= 0 && s(0)<=1 && s(1)>=0){
					intersections.push_back(s);}

			}//end of for
			if (intersections.size()>0){size_intersection++; total_scans++;
				point_intersect=cast_ray(&pose,ray_dir,ray_points,rays_file,intersections,&range, &min_scalar); // calculate points of intersection, take shortest range and (draw ?)
				intersections.clear(); 

				if(i > 0){
					Filewrite_for_micp(micp_file, ((scans_cp)*period), &pose ,  
				    beam_angle , range , point_intersect, min_scalar );
				}
				else if (i==0){ 
				Filewrite_for_micp(micp_file, ((scans)*period), &pose ,  
				beam_angle , range , point_intersect, min_scalar ); }  // write to file for micp  }
				
				
					scans++;
				    scans_cp++;
				
				
				if(   point_intersect(0) !=pose.delta_x){

					os << point_intersect(0)<< ' ' <<  point_intersect(1) << endl;
				}


			} // size_intersection  if
			trajectory.emplace_back(space_point);
			end_time+=period;
			
			beam_angle+=0.018;
			//cerr<<" end of while : "<< scans<< endl;
			} //END OF WHILE
			cerr<<"scans_cp : "<< scans_cp<<endl;
			cerr<<"scans : "<< scans<<endl;
			init_tick(0)+=((scans-1)*period)*(vel_pair[i].first+(b*vel_pair[i].second/2))/k_r;
			init_tick(1)+=((scans-1)*period)*(vel_pair[i].first-(b*vel_pair[i].second/2))/k_l;
			cerr<<" init_tick : "<< init_tick.transpose()<<endl;
			size_intersection =0;
			scans=1;
}
ticks_file.close();
	file0.close();

     int cnt=0;
     ofstream file("data.dat");
     for (auto& elem: trajectory){
      cnt ++;
      file << elem(0) << ' ' << elem(1) << endl; }

 file.close();
cout<< endl;
cout<< "number of effective scans " << total_scans <<endl;





}
