#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "../rotations.h"
#include "../simulator.h"
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
if (argc<7) {
    cerr << "usage: " << environ[0] << " v omega time_step SIMULATION_PERIOD k_r k_l b" << endl ;
    return -1;
  }
              /** ODOMETRY PARAMETERS DEFINITION AND ASSIGNMENT **/
float period; float end_time = 0;
float v=atof(argv[1]);
float omega=atof(argv[2]);
float k_r=atof(argv[5]); float k_l=atof(argv[6]); float b = atof(argv[7]); float omega_r;  float omega_l; float sim_time;

   period=atof(argv[3]); cout<<" FREQUENCY  : "<<period<<endl;
  sim_time=atof(argv[4]); cout<<" SIMULATION_PERIOD  : "<<sim_time<< "k_r : " << k_r << " k_l : " << k_l << " b : " << b <<endl;
/** POSE IS A STRUCT TO CONTAIN ROBOT'S POSE DATA AT EACH TIME TIME STEP **/
position  pose;

/** I USE SPACE_POINT AND TRAJECTORY AS A WAY TO HAVE A VECTOR OF EACH (X,Y) POSITION OF THE ROBOT **/
Vector2f space_point;
Vec_points trajectory,ray_points;


pose.delta_x=0; pose.delta_y=0; pose.theta=0;  //INITIAL POSE OF ROBOT
float beam_angle = 0; 						   //INITIAL ANGLE OF LIDAR
int size_intersection =0;

float v2; float omega2;
cout << "Insert v2 : " <<  endl; cin >> v2;
cout << "Insert omega2 : " <<  endl; cin >> omega2;

float v3; float omega3;
cout << "Insert v3 : " <<  endl; cin >> v3;
cout << "Insert omega3 : " <<  endl; cin >> omega3;

vector<float> tv={v,v2,v3};
vector<float> rv={omega,omega2,omega3};



end_time=period;
string ticks="mulmotion_ticks_file.dat";  ofstream ticks_file(ticks);
string velocities="velocities.dat";  ofstream pipe_vel(velocities);
pipe_vel << v << ' ' << omega << endl;
pipe_vel << v2 << ' ' << omega2 << endl;
pipe_vel << v3 << ' ' << omega3 << endl;
pipe_vel.close();
Vector3f init_pos;
init_pos.setZero();
float init_tick_r=0;
float init_tick_l=0;

vector<float>  t_r={ (v+(b*omega/2))/k_r, (v2+(b*omega2/2))/k_r , (v3+(b*omega3/2))/k_r   }; //right tick for each velocity
vector <float> t_l={ (v-(b*omega/2))/k_l,  (v2-(b*omega2/2))/k_l ,(v3-(b*omega3/2))/k_l  }; // left tick<< ' '
int index= 0;
float slot = sim_time/tv.size();
float tot_time=period;

while(tot_time <= sim_time){
		pose.theta= theta( (t_r[index])*end_time, (t_l[index])*end_time,k_l, k_r, b) + init_pos(0);
		pose.delta_x= x( (t_r[index])*end_time, (t_l[index])*end_time,k_l, k_r, b,pose.theta)+ init_pos(1);
		pose.delta_y= y( (t_r[index])*end_time, (t_l[index])*end_time,k_l, k_r, b,pose.theta)+init_pos(2);
        ticks_file << ( (t_r[index])*(end_time) ) + init_tick_r << ' ' <<( (t_l[index])*(end_time) ) + init_tick_l<< ' '<< tot_time << ' '<< end_time<<' '<<tv[index] << ' ' << rv[index] << endl  ;
    	space_point(0)=pose.delta_x; space_point(1)=pose.delta_y;

	trajectory.emplace_back(space_point);

	if(tot_time >= slot*(index+1)  ){
		end_time=0;
		index++;

	 }
	tot_time+=period;
    end_time+=period;

	} //END OF WHILE




     int cnt=0;
     ofstream file("mulmotion.dat");
     for (auto& elem: trajectory){
      cnt ++;
     // cout<< cnt << ": each element : " << elem << endl;
      file << elem(0) << ' ' << elem(1) << endl; }

 file.close();
cout<<"OUT OF THE WHILE LOOP"<<endl;
cout<<"index : "<<index<<endl;
cout<<"tr[index] : "<< t_r[index]<< " t_l[index] "<<t_l[index]<<endl;
cout<<" init_tick_r : "<< init_tick_r<< " init_tick_l "<<init_tick_r<<endl;
cout<< "number of effective scans " << size_intersection <<endl;
cout<<" v  : "<<v<< " omega : "<< omega<<endl;
cout<<" v2  : "<<v2<< " omega2 : "<< omega2<< endl;
cout<<" v3  : "<<v3<< " omega3 : "<< omega3<< endl;
cout<<" k_r : " << k_r << " k_l : " << k_l << " b : " << b <<endl;
cout<< " slot : " << slot << endl;




}
/**


cout<<"BEFORE UPDATE : "<<endl;
		cout<<"index : "<<index<<endl;
		cout << "Time: " << end_time <<"s" << endl;
		cout<<" init_tick_r : "<< init_tick_r<< " init_tick_l "<<init_tick_r<<endl;
		   init_tick_r += (t_r[index])*(end_time);
		   init_tick_l +=  (t_l[index])*(end_time); cout<<"index : "<<index<<endl;
		   cout<<"t_r[index] : "<<t_r[index]<<endl;
		   cout<<"t_l[index] : "<<t_l[index]<<endl;
		cout<<"AFTER UPDATE : "<<endl;
		cout << "Time: " << end_time <<"s" << endl;
		cout<<" init_tick_r : "<< init_tick_r<< " init_tick_l "<<init_tick_r<<endl;
           index++;
		cout<<"index : "<<index<< ", tot_time : " <<tot_time <<", slot*(index+1) "<< slot*(index+1)<<  endl;
		   init_pos(0)=pose.theta;
		   init_pos(1)=pose.delta_x;
		   init_pos(2)=pose.delta_y;
		    end_time=0;

*/
