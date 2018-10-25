/*
 * TrajectoryGen.cpp
 *
 *  Created on: Aug 30, 2018
 *      Author: laljarus
 */

#include <iostream>
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
//#include "Eigen-3.3/Eigen/QR"
#include "tools.h"
#include "TrajectoryGen.h"
#include <math.h>
#include <unordered_map>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Tools tools;

PathPlanning::PathPlanning(){}
PathPlanning::~PathPlanning(){}

void PathPlanning::Init(const double &car_s_in,const double &car_d_in,const double &car_speed_in,const vector<double> &maps_s_in,
		const vector<double> &maps_x_in,const vector<double> &maps_y_in,const vector<double> &previous_path_x_in,const vector<double> &previous_path_y_in,
		const double end_path_s_in,const double end_path_d_in,const vector<vector<double>> &sensor_fusion_in,const double &car_yaw_in,const bool &initialize_in){

			car_yaw = car_yaw_in;
			car_s = car_s_in;
			car_d = car_d_in;
			car_speed = car_speed_in/2.25;
			maps_s = maps_s_in;
			maps_x = maps_x_in;
			maps_y = maps_y_in;
			previous_path_x = previous_path_x_in;
			previous_path_y = previous_path_y_in;
			end_path_s = end_path_s_in;
			end_path_d = end_path_d_in;
			sensor_fusion = sensor_fusion_in;
			initialized = initialize_in;


}

vector<vector<double>> PathPlanning::GenerateTrajectory(double &set_speed,double &d){

	//double acc = 6;
	//double set_speed = PathPlanning::state_machine(); // velocity in m/s
	//double d = 6;
	double speed_limit = 21.5;
	int path_len = 101;
	double Total_time = (path_len-1)*0.02; // time in seconds
	double avg_speed;
	static int counter = 0;	

	/*set_speed += counter*0.02*acc;
	counter++;

	if(set_speed>20){
		set_speed = 20;
	}*/

	vector<double> next_x_vals;
	vector<double> next_y_vals;

	vector<double> initial_state_x;
	vector<double> initial_state_y;
	vector<double> final_state_x;
	vector<double> final_state_y;

	double time;

	vector<double> pos;
	double x,y;

	double final_s;
	vector<double> car_pos_init_xy;
	vector<double> car_pos_final_xy;
	vector<double> coefficients_x,coefficients_y;


	int prev_path_weight = 65;

	int prev_path_size = previous_path_x.size();


	if((set_speed-car_speed)>8){
		set_speed = car_speed+8;
	}else if ((car_speed - set_speed)>8){
		set_speed = car_speed-8 ;
	}

	avg_speed = (car_speed+set_speed)/2;

	if(avg_speed > speed_limit){
		avg_speed = speed_limit;
	}

	if(set_speed > speed_limit){
		set_speed = speed_limit;
	}

	/*double TimeToTurn = 20;

	if(counter > int(TimeToTurn/dt)){
		d = 10;
	}
	counter++;*/

	//cout<<"Previous Path Size: "<<prev_path_size<<endl;
	cout<<"set speed:"<<set_speed<<endl;

	double dx,dy,ds;

	if(prev_path_size == 0){

		car_pos_init_xy = tools.getXY(car_s,car_d,maps_s,maps_x,maps_y);
		final_s = avg_speed*Total_time + car_s;
		car_pos_final_xy = tools.getXY(final_s,car_d,maps_s,maps_x,maps_y);

		initial_state_x.push_back(car_pos_init_xy[0]);
		initial_state_x.push_back(car_speed*cos(car_yaw));
		initial_state_x.push_back(0);

		initial_state_y.push_back(car_pos_init_xy[1]);
		initial_state_y.push_back(car_speed*sin(car_yaw));
		initial_state_y.push_back(0);


		final_state_x.push_back(car_pos_final_xy[0]);
		final_state_x.push_back(set_speed*cos(car_pos_final_xy[2]));
		final_state_x.push_back(0);

		final_state_y.push_back(car_pos_final_xy[1]);
		final_state_y.push_back(set_speed*sin(car_pos_final_xy[2]));
		final_state_y.push_back(0);

		//cout<<"ds:"<<(final_s-car_s)<<endl;
		//cout<<"ds_dot:"<<(set_speed - car_speed)<<endl;


		coefficients_x = PathPlanning::JMT(initial_state_x,final_state_x,Total_time);
		coefficients_y = PathPlanning::JMT(initial_state_y,final_state_y,Total_time);

		for(int i = 0;i<path_len;i++){

			time = dt*i;
			x = coefficients_x[0] + coefficients_x[1]*time+ coefficients_x[2]*pow(time,2)
					+ coefficients_x[3]*pow(time,3)+coefficients_x[4]*pow(time,4)+coefficients_x[5]*pow(time,5);

			y = coefficients_y[0] + coefficients_y[1]*time+ coefficients_y[2]*pow(time,2)
							+ coefficients_y[3]*pow(time,3)+coefficients_y[4]*pow(time,4)+coefficients_y[5]*pow(time,5);

			next_x_vals.push_back(x);
			next_y_vals.push_back(y);
		}

	}else if(prev_path_size != 0 && prev_path_size < prev_path_weight) {

		for(int i = 0; i < prev_path_size; i++){

			next_x_vals.push_back(previous_path_x[i]);
			next_y_vals.push_back(previous_path_y[i]);
		}

		double x_k = previous_path_x[prev_path_size-1];
		double x_k_1 = previous_path_x[prev_path_size -2];
		double x_k_2 = previous_path_x[prev_path_size -3];

		double x_dot_k = (x_k - x_k_1)/dt;
		double x_dot_k_1 = (x_k_1 - x_k_2)/dt;

		double x_dot_dot_k = (x_dot_k - x_dot_k_1)/dt;


		double y_k = previous_path_y[prev_path_size -1];
		double y_k_1 = previous_path_y[prev_path_size - 2];
		double y_k_2 = previous_path_y[prev_path_size - 3];

		double y_dot_k = (y_k - y_k_1)/dt;
		double y_dot_k_1 = (y_k_1 - y_k_2)/dt;

		double y_dot_dot_k = (y_dot_k - y_dot_k_1)/dt;

		double car_speed_k = sqrt(x_dot_k*x_dot_k+y_dot_k*y_dot_k);
		double yaw_k = atan2(y_dot_k,x_dot_k);
		vector<double> init_pos_sd = tools.getFrenet(x_k,y_k,yaw_k,maps_x,maps_y);

		if(abs(car_speed-set_speed)>2){
			avg_speed = (car_speed_k+set_speed)/2;
		}else{
			avg_speed = (car_speed_k+set_speed)/2;
		}

		if(avg_speed > speed_limit){
			avg_speed = speed_limit;
		}

		final_s = avg_speed*Total_time + end_path_s-abs(car_d-d);

		//cout<<"Car speed:"<<car_speed_k<<endl;
		//cout<<"Average Speed:"<<avg_speed<<endl;

		//cout<<"Car S:"<<car_s<<endl;
		//cout<<"Final S:"<<final_s<<endl;
		//cout<<"End Path S:"<<end_path_s<<endl;

		car_pos_final_xy = tools.getXY(final_s,d,maps_s,maps_x,maps_y);

		ds = final_s - end_path_s;
		dx = car_pos_final_xy[0]-x_k;
		dy = car_pos_final_xy[1]-y_k;

		double dist_xy = sqrt(dx*dx+dy*dy);
		double dist_sd = sqrt(ds*ds +(car_d-d)*(car_d-d));

		if((dist_xy-dist_sd)>0){
			final_s = final_s - (dist_xy-dist_sd);
		}

		car_pos_final_xy = tools.getXY(final_s,d,maps_s,maps_x,maps_y);

		//cout<<"distance_traveled_freenet:"<<dist_sd<<endl;
		//cout<<"distance traveled xy:"<<dist_xy<<endl;
		//cout<<"end path s error:"<<end_path_s - init_pos_sd[0]<<endl;

		//cout<<"x_diff:"<<(next_x_vals[prev_path_size-1]- x_k)<<endl;
		//cout<<"y_diff:"<<(next_y_vals[prev_path_size-1]- y_k)<<endl;

		initial_state_x.push_back(x_k);
		initial_state_x.push_back(x_dot_k);
		initial_state_x.push_back(x_dot_dot_k);

		initial_state_y.push_back(y_k);
		initial_state_y.push_back(y_dot_k);
		initial_state_y.push_back(y_dot_dot_k);

		final_state_x.push_back(car_pos_final_xy[0]);
		final_state_x.push_back(set_speed*cos(car_pos_final_xy[2]));
		final_state_x.push_back(0);

		final_state_y.push_back(car_pos_final_xy[1]);
		final_state_y.push_back(set_speed*sin(car_pos_final_xy[2]));
		final_state_y.push_back(0);

		coefficients_x = PathPlanning::JMT(initial_state_x,final_state_x,Total_time);
		coefficients_y = PathPlanning::JMT(initial_state_y,final_state_y,Total_time);

		for(int i = 0;i<(path_len-prev_path_weight);i++){

			time = dt*i;
			x = coefficients_x[0] + coefficients_x[1]*time+ coefficients_x[2]*pow(time,2)
					+ coefficients_x[3]*pow(time,3)+coefficients_x[4]*pow(time,4)+coefficients_x[5]*pow(time,5);

			y = coefficients_y[0] + coefficients_y[1]*time+ coefficients_y[2]*pow(time,2)
							+ coefficients_y[3]*pow(time,3)+coefficients_y[4]*pow(time,4)+coefficients_y[5]*pow(time,5);

			next_x_vals.push_back(x);
			next_y_vals.push_back(y);
		}

	} else {

		for(int i = 0; i < prev_path_size; i++){

			next_x_vals.push_back(previous_path_x[i]);
			next_y_vals.push_back(previous_path_y[i]);
		}
	}

	//cout<<"set x,y:"<<next_x_vals[0]<<","<<next_y_vals[0]<<endl;
	vector<vector<double>> result;

	result.push_back(next_x_vals);
	result.push_back(next_y_vals);

	return result;
}

vector<double> PathPlanning::JMT(vector<double> &start, vector<double> &end, double T){

    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */

    MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;

	MatrixXd B = MatrixXd(3,1);
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];

	MatrixXd Ai = A.inverse();

	MatrixXd C = Ai*B;

	vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}

    return result;

}


vector<vector<double>> PathPlanning::state_machine(){

	double set_speed;
	vector<vector<double>> trajectory;
	vector<string> possible_states;

	bool toggle = false;
	static int prev_lane = 1;
	static int counter = 0;
	static int counter_turn = 0;
	static int new_lane = car_lane;
	static int lane_id = 0;
	static double set_d = 6;

	if((car_d>0 and car_d<=4) and new_lane !=0){
		new_lane = 0;
		counter = 0;
	}else if((car_d>4 and car_d<=8) and new_lane !=1){
		new_lane = 1;
		counter = 0;
	}else if((car_d>8 and car_d<=12) and new_lane !=2){
		new_lane = 2;
		counter = 0;
	}

	if(counter > 50){
		car_lane = new_lane;
	}
	counter++;

	prev_lane = car_lane;
	cout<<"Car_s:"<<car_s<<endl;

	if(state.compare("KL") == 0){
		possible_states.push_back("KL");
		if(car_lane == 1){
			possible_states.push_back("TL");
			possible_states.push_back("TR");
		}else if(car_lane == 0){
			possible_states.push_back("TR");
		}else if(car_lane == 2){
			possible_states.push_back("TL");
		}
	}else{
		possible_states.push_back("KL");
	}

	vector<double> lane_d = {2,6,10};

	if(counter_turn > 500){
		counter_turn = 0;
		lane_id =(lane_id+1)%3;
		set_d = lane_d[lane_id];
	}
	counter_turn++;
	//cout<<"counter_turn"<<counter_turn;
	//cout<<"set_d"<<set_d;

	sensor_fusion_processing();
	set_speed = keep_lane();

	trajectory = GenerateTrajectory(set_speed,set_d);

	return trajectory;
}


void PathPlanning::sensor_fusion_processing(){

	//vector<double> result;
	double set_speed = 20;

	double distance_front_left = 10000,distance_front_center = 10000, distance_front_right = 10000;
	double distance_back_left = 10000,distance_back_center = 10000, distance_back_right = 10000;
	double id_front_left,id_front_center,id_front_right,id_back_left,id_back_right,id_back_center;
	double id,x,y,vx,vy,s,d,v,ax,ay;
	vector<double> car,old_car;
	vector<double> left_lane,right_lane,center_lane;
	double average_speed_left = 30,average_speed_center = 30, average_speed_right = 30;
	double total_speed_left = 0,total_speed_right = 0, total_speed_center = 0;
	double n_left=0,n_center=0,n_right=0;
	static vector<double> old_speed_x,old_speed_y;
	if(!initialized){
		for(int i = 0;i<sensor_fusion.size();i++){
			old_speed_x.push_back(0);
			old_speed_y.push_back(0);
		}
	}


	for(int i = 0;i<sensor_fusion.size();i++){

			id = sensor_fusion[i][0];
			x = sensor_fusion[i][1];
			y = sensor_fusion[i][2];
			vx = sensor_fusion[i][3];
			vy = sensor_fusion[i][4];
			s = sensor_fusion[i][5];
			d = sensor_fusion[i][6];
			v = sqrt(vx*vx+vy*vy);
			ax = (old_speed_x[id]-vx)/dt;
			ay = (old_speed_y[id]-vy)/dt;
			old_speed_x[id] = vx;
			old_speed_y[id] = vy;
			sensor_fusion[i].push_back(ax);
			sensor_fusion[i].push_back(ay);

			//car = sensor_fusion[i];
			//sensor_fusion[i].push_back(1);
			//  cout<<"id:"<<id<<endl;

			/*if(!initialized){
				//sensor_fusion[i].push_back(sqrt(vx*vx + vy*vy)); // current speed
				sensor_fusion[i].push_back(0); // previous speed
			} else {
				//sensor_fusion[i].push_back(sqrt(vx*vx + vy*vy)); // current speed
				old_car = SensorFusion_map[id];
				cout<<"Old_car:"<<old_car.size()<<endl;
				sensor_fusion[i].push_back(sqrt(old_car[3]*old_car[3]+old_car[4]*old_car[4]));

			}*/
			if(d>0 and d<=4){
				sensor_fusion[i].push_back(0); // left lane
				if(s>car_s){
					n_left +=1;
					total_speed_left += v;
					average_speed_left = total_speed_left/n_left;
				}

				if(((s - car_s)<distance_front_left) and s>car_s){
					distance_front_left = s -car_s;
					id_front_left = id;
				}

				if(((car_s - s)<distance_back_left) and s<car_s){
					distance_back_left = car_s -s;
					id_back_left = id;
				}
			}

			if(d>4 and d<=8){
				sensor_fusion[i].push_back(1); // center lane
				if(s>car_s){
					n_center +=1;
					total_speed_center += v;
					average_speed_center = total_speed_center/n_center;
				}

				if(((s - car_s)<distance_front_center)and s>car_s){
					distance_front_center = s -car_s;
					id_front_center = id;
				}

				if(((car_s - s)<distance_back_center) and s<car_s){
					distance_back_center = car_s -s;
					id_back_center = id;
				}

			}

			if(d>8 and d<=12){
				sensor_fusion[i].push_back(2); // right lane.
				if(s>car_s){
					n_right +=1;
					total_speed_right += v;
					average_speed_right = total_speed_right/n_right;
				}
				if(((s - car_s)<distance_front_right) and s>car_s){
					distance_front_right = s -car_s;
					id_front_right = id;
				}
				if(((car_s - s)<distance_back_right) and s<car_s){
					distance_back_right = car_s -s;
					id_back_right = id;
				}
			}
			car = sensor_fusion[i];
			//cout<<"car size:"<<car.size()<<endl;
			SensorFusion_map.insert({id,car});
	}

	left_lane.push_back(average_speed_left);
	left_lane.push_back(id_front_left);
	left_lane.push_back(id_back_left);

	center_lane.push_back(average_speed_center);
	center_lane.push_back(id_front_center);
	center_lane.push_back(id_back_center);

	right_lane.push_back(average_speed_right);
	right_lane.push_back(id_front_right);
	right_lane.push_back(id_back_right);

	lane_info.clear();
	lane_info.push_back(left_lane);
	lane_info.push_back(center_lane);
	lane_info.push_back(right_lane);

}

double PathPlanning::keep_lane(){

		double set_speed = 20.5;
		double distance_front,front_car_speed,relative_speed,acceleration;
		vector<double> front_car;
		double path_len = 100;
		double TotalTime = dt*path_len;
		double distance_treshold = 50;
		double min_distance = 30;
		double front_car_new_s, car_new_s,avg_speed,distance_new;
		double id_front_left,id_front_center,id_front_right;
		static double old_speed = 0;

		id_front_left 	= lane_info[0][1];
		id_front_center = lane_info[1][1];
		id_front_right 	= lane_info[2][1];

		if(car_lane == 0){
			if(SensorFusion_map.count(id_front_left)){
				front_car = SensorFusion_map[id_front_left];
				distance_front = front_car[5] - car_s;
			}
		}else if(car_lane == 1){
			if(SensorFusion_map.count(id_front_center)){
				front_car = SensorFusion_map[id_front_center];
				distance_front = front_car[5] - car_s;
			}
		}else if(car_lane == 2){
			if(SensorFusion_map.count(id_front_right)){
				front_car = SensorFusion_map[id_front_right];
				distance_front = front_car[5] - car_s;
			}
		}

		if(!front_car.empty()){

			front_car_speed = sqrt(pow(front_car[3],2)+pow(front_car[4],2));
			//cout<<"Front Car distance:"<<distance_front<<endl;
			relative_speed = car_speed - front_car_speed;
			acceleration = (car_speed - old_speed)/dt;
			//cout<<"Acceleration:"<<acceleration<<endl;
			old_speed = car_speed;

			if (relative_speed > 0 && distance_front < distance_treshold){
				 set_speed = car_speed - relative_speed;
			}

			front_car_new_s = (front_car_speed*TotalTime+front_car[5]);
			car_new_s = (car_s + (set_speed+car_speed)/2*TotalTime);

			double distance_new = front_car_new_s - car_new_s;

			if(distance_new < min_distance){
				avg_speed = (front_car_new_s - min_distance - car_s)/TotalTime;
				set_speed = 2*avg_speed - car_speed;
			}
		}
		return set_speed;
}

vector<vector<double>> PathPlanning::predict(double &car_id,int &path_len){

	// constant acceleration model prediction

	vector<vector<double>> trajectory;
	vector<double> x_vals;
	vector<double> y_vals;
	vector<double> car;

	car = SensorFusion_map[car_id];
	double vx = car[3];
	double vy = car[4];
	double s = car[5];
	double d = car[6];
	double ax = car[7];
	double ay = car[8];
	double lane = car[8];
	double v = sqrt(vx*vx+vy+vy);
	double yaw;
	vector<double> init_pos_xy;
	vector<double> pos_xy;

	init_pos_xy = tools.getXY(s,d,maps_s,maps_x,maps_y);
	yaw = init_pos_xy[2];

	double vs = vx*cos(yaw)-vy*sin(yaw);
	double vd = vx*sin(yaw)-vy*cos(yaw);

	double as = vx*cos(yaw)-vy*sin(yaw);
	double ad = vx*sin(yaw)-vy*cos(yaw);

	//x_vals.push_back(car[1]);
	//y_vals.push_back(car[2]);

	/*if(lane == 0){
		d = 2;
	}else if(lane == 1){
		d = 6;
	}else if(lane == 2){
		d = 10;
	}*/

	for(int i = 0;i<path_len;i++){
		vs = vs + as*dt;
		s = s + vs*dt;
		pos_xy = tools.getXY(s,d,maps_s,maps_x,maps_y);
		x_vals.push_back(pos_xy[0]);
		y_vals.push_back(pos_xy[1]);
	}

	trajectory.push_back(x_vals);
	trajectory.push_back(y_vals);

	return trajectory;

}

