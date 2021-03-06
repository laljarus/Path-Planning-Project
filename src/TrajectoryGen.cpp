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

vector<vector<double>> PathPlanning::GenerateTrajectory(double &set_speed,double &d,int &path_len){

	//double acc = 6;
	//double set_speed = PathPlanning::state_machine(); // velocity in m/s
	//double d = 6;
	double speed_limit = 21.5;
	//int path_len = 75;
	double Total_time = (path_len-1)*0.02; // time in seconds
	double avg_speed;
	//static int counter = 0;

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
	double ds_dot_limit;
	double final_s;
	vector<double> car_pos_init_xy;
	vector<double> car_pos_final_xy;
	vector<double> coefficients_x,coefficients_y;


	int prev_path_weight = path_len*3/4;

	int prev_path_size = previous_path_x.size();

	if(path_len > 80){
		ds_dot_limit = 8;
	}else {
		ds_dot_limit = 6;
	}


	if((set_speed-car_speed)>ds_dot_limit){
		set_speed = car_speed+ds_dot_limit;
	}else if ((car_speed - set_speed)>ds_dot_limit){
		set_speed = car_speed-ds_dot_limit;
	}

	avg_speed = (car_speed+set_speed)/2;

	if(avg_speed > speed_limit){
		avg_speed = speed_limit;
	}

	if(set_speed > speed_limit){
		set_speed = speed_limit;
	}

	if((d - car_d)>4){
		d = car_d +4;
	}else if((car_d -d)>4 ){
		d = car_d -4;
	}

	//double TimeToTurn = 20;

	/*if(counter > int(TimeToTurn/dt)){
		d = 2;
	}
	counter++;*/

	//cout<<"counter: "<<counter<<endl;
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

		//final_s = avg_speed*Total_time + end_path_s-abs(car_d-d);
		final_s = avg_speed*Total_time + end_path_s;

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

		if(dist_xy > speed_limit*Total_time){
			final_s = avg_speed*Total_time - abs(car_d - d) + end_path_s;
		}

		car_pos_final_xy = tools.getXY(final_s,d,maps_s,maps_x,maps_y);

		ds = final_s - end_path_s;
		dx = car_pos_final_xy[0]-x_k;
		dy = car_pos_final_xy[1]-y_k;

		dist_xy = sqrt(dx*dx+dy*dy);
		dist_sd = sqrt(ds*ds +(car_d-d)*(car_d-d));

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

		for(int i = 1;i<(path_len-prev_path_weight);i++){

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
	static double lane_change_speed = 10,cost_diff = 0;
	double target_speed = 21;
	int path_len_kl = 75,path_len_lane_change = 101;
	enum States {Init,KL,TR,TL};
	vector<vector<double>> trajectory;
	vector<States> possible_states;
	static States state = Init;

	if(!initialized){
		car_lane = 1;
	}

	bool toggle = false;
	static int prev_lane = 1;
	static int counter = 0;
	static int counter_turn = 0;
	static int new_lane = car_lane;
	static int lane_id = 0;
	static double set_d = 6;
	static int next_lane;
	static int init_lane = 1;

	double weightInefficiencyCost = 1;
	double weightAccelerationCost = 5;
	double weightCollisionCost   = 10;

	if((car_d>0 and car_d<=4) and car_lane !=0){
		new_lane = 0;
		counter++;

	}else if((car_d>4 and car_d<=8) and car_lane !=1){
		new_lane = 1;
		counter++;

	}else if((car_d>8 and car_d<=12) and car_lane !=2){
		new_lane = 2;
		counter++;

	}

	if(counter > 100){
		car_lane = new_lane;
		cout<<"Car Lane:"<<car_lane<<endl;
		counter = 0;
	}

	prev_lane = car_lane;
	//cout<<"Car_s:"<<car_s<<endl;

	if(state == KL){
		possible_states.push_back(KL);
		if(car_lane == 1){
			possible_states.push_back(TL);
			possible_states.push_back(TR);
		}else if(car_lane == 0){
			possible_states.push_back(TR);
		}else if(car_lane == 2){
			possible_states.push_back(TL);
		}
	}else{
		possible_states.push_back(KL);
	}

	vector<double> lane_d = {2,6,10};

	/*if(counter_turn > 500){
		counter_turn = 0;
		lane_id =(lane_id+1)%3;
		set_d = lane_d[lane_id];
	}
	counter_turn++;*/

	//cout<<"set_d"<<set_d;
	sensor_fusion_processing();

	if(state == Init) {
		double init_speed = 10;


		static int counter_init = 0;
		set_speed = target_speed;
		trajectory = GenerateTrajectory(set_speed,lane_d[init_lane],path_len_kl);

		if(car_speed > init_speed){
			counter_init++;
			if(counter_init > 100){
				state = KL;
				counter_init = 0;
			}

		}

	}else if (state == KL){

		vector<vector<double>> trajectory_kl,trajectory_tl,trajectory_tr;
		double cost_kl,cost_tl,cost_tr;
		double min_cost  = 1000;
		double front_car_id, front_car_distance = 1000;
		vector<double> front_car;
		static int counter_kl = 0;
		front_car_id = lane_info[car_lane][1];
		front_car = SensorFusion_map[front_car_id];

		if(!front_car.empty()){
			front_car_distance = front_car[5]-car_s;
		}

		if(front_car.empty() || front_car_distance > 60 || front_car_distance < 20 ){
			if(!front_car.empty()){
				cout<<"Sub state: KeepLane : "<< front_car_distance<<endl;
			}else{
				cout<<"Sub state: KeepLane : "<<endl;
			}
			
			set_speed = keep_lane(target_speed,car_lane);
			trajectory = GenerateTrajectory(set_speed,lane_d[car_lane],path_len_kl);
			state = KL;

		}else{
			cout<<"Sub state: Look for changing lane:"<<front_car_distance<<endl;
			
			for(int i = 0; i < possible_states.size();i++) {

						States possible_state = possible_states[i];
						double car_lane_double = (double)car_lane;
						double car_lane_tl = fmod ((car_lane_double - 1.0), 3.0);
						double car_lane_tr = fmod ((car_lane_double + 1.0), 3.0);
						int lane_tl = (car_lane-1)%3;
						int lane_tr = (car_lane+1)%3;

						switch(possible_state){

						case KL:

							set_speed = keep_lane(target_speed,car_lane);
							trajectory_kl = GenerateTrajectory(set_speed,lane_d[car_lane],path_len_kl);
							cost_kl = weightInefficiencyCost * InefficiencyCost(target_speed,car_lane_double) + weightAccelerationCost * AccelerationAndJerkCost(trajectory_kl) +
									  weightCollisionCost*CollisionCost(trajectory_kl,car_lane_double);
							cout<<"Cost KL:"<<cost_kl<<endl;

							if(cost_kl < min_cost){
								trajectory = trajectory_kl;
								state = KL;
								next_lane = car_lane;
								lane_change_speed  = car_speed;
								min_cost = cost_kl;
							}
							break;
						case TL:

							set_speed = keep_lane(car_speed,lane_tl);
							trajectory_tl = GenerateTrajectory(set_speed,lane_d[lane_tl],path_len_lane_change);
							cost_tl = weightInefficiencyCost * InefficiencyCost(target_speed,car_lane_tl) + weightAccelerationCost * AccelerationAndJerkCost(trajectory_tl)
										+ weightCollisionCost*CollisionCost(trajectory_tl,car_lane_tl);
							cout<<"Cost TL:"<<cost_tl<<endl;
							if((min_cost - cost_tl) > 0.1){
								trajectory = trajectory_tl;
								state = TL;
								lane_change_speed  = car_speed;
								next_lane = (car_lane - 1) % 3;
								cost_diff = cost_kl - cost_tl;
								min_cost = cost_tl;
							}
							break;
						case TR:
							set_speed = keep_lane(car_speed,lane_tr);
							trajectory_tr = GenerateTrajectory(set_speed,lane_d[lane_tr],path_len_lane_change);
							cost_tr = weightInefficiencyCost * InefficiencyCost(target_speed,car_lane_tr) + weightAccelerationCost*AccelerationAndJerkCost(trajectory_tr)
										+ weightCollisionCost*CollisionCost(trajectory_tr,car_lane_tr);
							cout<<"Cost TR:"<<cost_tr<<endl;
							if((min_cost - cost_tr) > 0.1){
								trajectory = trajectory_tr;
								state = TR;
								lane_change_speed  = car_speed;
								next_lane = (car_lane + 1) % 3;
								cost_diff = cost_kl - cost_tr;
								min_cost = cost_tr;
							}
							break;
						}
					}
			cout<<"Min cost:"<<min_cost<<endl;
		}




	}else {
		trajectory = GenerateTrajectory(lane_change_speed,lane_d[next_lane],path_len_lane_change);
		cout<<"cost difference:"<<cost_diff<<endl;
		if(car_lane == next_lane) {
			state = KL;
			//min_cost  = 1000;
		}
	}

	//cout<<"set speed:"<<set_speed<<endl;

	/*cout<<"Car Lane:"<<car_lane<<endl;
	cout<<"New Lane:"<<new_lane<<endl;
	cout<<"counter:"<<counter<<endl;*/

	//set_speed = keep_lane();

	//trajectory = GenerateTrajectory(set_speed,set_d);


	return trajectory;
}


void PathPlanning::sensor_fusion_processing(){

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
			if((d>0) && (d<=4)){
				sensor_fusion[i].push_back(0); // left lane
				if(s>car_s){
					n_left +=1;
					total_speed_left += v;
					average_speed_left = total_speed_left/n_left;
				}

				if(((s - car_s) < distance_front_left) and s>car_s){
					distance_front_left = s -car_s;
					id_front_left = id;
				}

				if(((car_s - s) < distance_back_left) and s<car_s) {
					distance_back_left = car_s -s;
					id_back_left = id;
				}
			}

			if(d>4 && d<=8) {
				sensor_fusion[i].push_back(1); // center lane
				if(s>car_s) {
					n_center +=1;
					total_speed_center += v;
					average_speed_center = total_speed_center/n_center;
				}

				if(((s - car_s) < distance_front_center)and s>car_s) {
					distance_front_center = s -car_s;
					id_front_center = id;
				}

				if(((car_s - s) < distance_back_center) and s<car_s) {
					distance_back_center = car_s -s;
					id_back_center = id;
				}

			}

			if(d>8 and d<=12){
				sensor_fusion[i].push_back(2); // right lane.
				if(s>car_s) {
					n_right +=1;
					total_speed_right += v;
					average_speed_right = total_speed_right/n_right;
				}
				if(((s - car_s)<distance_front_right) and s>car_s) {
					distance_front_right = s -car_s;
					id_front_right = id;
				}
				if(((car_s - s)<distance_back_right) and s<car_s) {
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


double PathPlanning::keep_lane(double &target_speed, int &new_lane){

		double set_speed = target_speed;
		double distance_front,front_car_speed,relative_speed,acceleration;
		vector<double> front_car;
		double path_len = 75;
		double TotalTime = dt*path_len;
		double distance_treshold = 50;
		double min_distance = 25;
		double front_car_new_s, car_new_s,avg_speed,distance_new;
		double front_car_id;
		static double old_speed = 0;

		front_car_id 	= lane_info[new_lane][1];
		front_car = SensorFusion_map[front_car_id];


		if(!front_car.empty()){
			distance_front = front_car[5] - car_s;
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

			if(distance_new < min_distance) {
				avg_speed = (front_car_new_s - min_distance - car_s)/TotalTime;
				set_speed = 2*avg_speed - car_speed;
			}
		}

		return set_speed;
}

vector<vector<double>> PathPlanning::predict(double &car_id,int &path_len) {

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

	//double vs = vx*cos(yaw)+vy*sin(yaw);
	double vs = sqrt(vx*vx+vy*vy);
	//double vd = -vx*sin(yaw)+vy*cos(yaw);

	//double as = vx*cos(yaw)+vy*sin(yaw);
	double as = sqrt(ax*ax+ay*ay);
	//double ad = -vx*sin(yaw)+vy*cos(yaw);

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
		//vs = vs + as*dt;
		s = s + vs*dt;
		pos_xy = tools.getXY(s,d,maps_s,maps_x,maps_y);
		x_vals.push_back(pos_xy[0]);
		y_vals.push_back(pos_xy[1]);
	}

	trajectory.push_back(x_vals);
	trajectory.push_back(y_vals);

	return trajectory;

}

double PathPlanning::InefficiencyCost(double &target_speed,double &intended_lane){
	double cost_speed,cost_distance,cost;

	double avg_speed_current_lane = lane_info[car_lane][0];
	double avg_speed_new_lane = lane_info[intended_lane][0];
	double front_car_new_lane_id = lane_info[intended_lane][1];
	double distance_new_lane = 100;
	vector<double> front_car = SensorFusion_map[front_car_new_lane_id];

	if(!front_car.empty()){

		distance_new_lane = front_car[5] - car_s;
	}

	cost_speed = (target_speed - avg_speed_new_lane)/target_speed;

	cost_distance = exp(-0.05*distance_new_lane);


	cost = 0.7*cost_distance + 0.3*cost_speed;

	return cost;
}

double PathPlanning::AccelerationAndJerkCost(vector<vector<double>> &trajectory){
	double cost,cost_a,cost_j;
	vector<double> x_vals = trajectory[0];
	vector<double> y_vals = trajectory[1];

	double max_a = 0;
	double max_j = 0;


	double old_vx = 0;
	double old_vy = 0;

	double old_ax = 0;
	double old_ay = 0;

	double vx,vy,ax,ay,jx,jy,a,j;

	for(int i = 0;i<x_vals.size();i++){
		if(i == 0){
			vx = 0;
			vy = 0;
			ax = 0;
			ay = 0;
			old_vx = 0;
			old_vy = 0;
			old_ax = 0;
			old_ay = 0;
		}else{
			vx = (x_vals[i]-x_vals[i-1])/dt;
			vy = (y_vals[i]-y_vals[i-1])/dt;
			ax = (vx - old_vx)/dt;
			ay = (vy - old_vy)/dt;
			jx = (ax - old_ax)/dt;
			jy = (ay - old_ay)/dt;

			a = sqrt(ax*ax+ay*ay);
			j = sqrt(jx*jx+jy*jy);

			if(a > max_a){
				max_a = a;
			}

			if(j > max_j){
				max_j = j;
			}

			old_vx = vx;
			old_vy = vy;
			old_ax = ax;
			old_ay = ay;
		}
	}

	cost_a = 1 - exp(-0.5*a);
	cost_j = 1 - exp(-0.5*j);
	cost = max(cost_a,cost_j);

	return cost;
}

double PathPlanning::CollisionCost(vector<vector<double>> &trajectory,double &new_lane){
	double cost;
	double front_car_id,new_lane_front_car_id,new_lane_back_car_id,car_id;
	vector<double> car;
	front_car_id = lane_info[car_lane][1];
	new_lane_front_car_id = lane_info[new_lane][1];
	new_lane_back_car_id = lane_info[new_lane][2];
	int path_len = trajectory[0].size();
	double min_distance_1 = 1000,min_distance_2 = 1000,min_distance_3 = 1000,min_distance = 1000,distance = 1000;
	vector<vector<double>> trajectory_next_car;

	/*for(auto it = SensorFusion_map.begin();it != SensorFusion_map.end(); it++){
		car_id = it->first;
		trajectory_next_car = predict(car_id,path_len);
		distance = MinDistance(trajectory,trajectory_next_car);

		if(distance < min_distance){
			min_distance = distance;
		}

	}*/

	car = SensorFusion_map[front_car_id];
	if(!car.empty()){
		vector<vector<double>> trajectory_front_car = predict(front_car_id,path_len);
		min_distance_1 = MinDistance(trajectory,trajectory_front_car);
	}

	car = SensorFusion_map[new_lane_front_car_id];
	if(!car.empty()){
		vector<vector<double>> trajectory_new_lane_front_car = predict(new_lane_front_car_id,path_len);
		min_distance_2 = MinDistance(trajectory,trajectory_new_lane_front_car);
	}

	car = SensorFusion_map[new_lane_back_car_id];
	if(!car.empty()){
		vector<vector<double>> trajectory_new_lane_back_car  = predict(new_lane_back_car_id,path_len);
		min_distance_3 = MinDistance(trajectory,trajectory_new_lane_back_car);
	}

	min_distance = min(min_distance_1,min_distance_2);
	min_distance = min(min_distance,min_distance_3);

	/*if(min_distance < 30){
		cost = 1-(min_distance/30);
	}else{
		cost = 0;
	}*/
	cout<<"Min Distance:"<<min_distance<<endl;

	cost = exp(-0.1*min_distance);

	return cost;
}

double PathPlanning::MinDistance(vector<vector<double>> &trajectory_1, vector<vector<double>> &trajectory_2) {
	double min_distance = 10000;
	vector<double> x_vals_1,x_vals_2,y_vals_1,y_vals_2;

	x_vals_1 = trajectory_1[0];
	y_vals_1 = trajectory_1[1];

	x_vals_2 = trajectory_2[0];
	y_vals_2 = trajectory_2[1];

	double path_len = min(x_vals_1.size(),x_vals_2.size());

	double x1,x2,y1,y2;
	for(int i = 0;i<path_len;i++){
		x1 = x_vals_1[i];
		y1 = y_vals_1[i];
		x2 = x_vals_2[i];
		y2 = y_vals_2[i];

		double distance = sqrt(pow((x1-x2),2)+pow((y1-y2),2));

		if(distance<min_distance){
			min_distance = distance;
		}
	}

	return min_distance;
}


