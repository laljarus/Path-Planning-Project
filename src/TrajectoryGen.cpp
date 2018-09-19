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

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Tools tools;

TrajectoryGen::TrajectoryGen(){}
TrajectoryGen::~TrajectoryGen(){}

vector<vector<double>> TrajectoryGen::KeepLane(const double &car_s,const double &car_d,const double &car_speed,const vector<double> &maps_s,
		const vector<double> &maps_x, const vector<double> &maps_y,const vector<double> &previous_path_x, const vector<double> &previous_path_y, double end_path_s, double end_path_d){

	//double acc = 6;
	double set_speed = 20; // velocity in m/s
	double speed_limit = 20;
	int path_len = 75;
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
	double dt = 0.02;

	vector<double> pos;
	double x,y;
	double d = 6;
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

	double TimeToTurn = 20;

	if(counter > int(TimeToTurn/dt)){
		d = 10;
	}
	counter++;

	cout<<"Previous Path Size: "<<prev_path_size<<endl;
	cout<<"set speed:"<<set_speed<<endl;

	double dx,dy;

	if(prev_path_size == 0){

		car_pos_init_xy = tools.getXY(car_s,car_d,maps_s,maps_x,maps_y);
		final_s = avg_speed*Total_time + car_s;
		car_pos_final_xy = tools.getXY(final_s,car_d,maps_s,maps_x,maps_y);

		initial_state_x.push_back(car_pos_init_xy[0]);
		initial_state_x.push_back(car_speed*cos(car_pos_init_xy[2]));
		initial_state_x.push_back(0);

		initial_state_y.push_back(car_pos_init_xy[1]);
		initial_state_y.push_back(car_speed*sin(car_pos_init_xy[2]));
		initial_state_y.push_back(0);


		final_state_x.push_back(car_pos_final_xy[0]);
		final_state_x.push_back(set_speed*cos(car_pos_final_xy[2]));
		final_state_x.push_back(0);

		final_state_y.push_back(car_pos_final_xy[1]);
		final_state_y.push_back(set_speed*sin(car_pos_final_xy[2]));
		final_state_y.push_back(0);

		cout<<"ds:"<<(final_s-car_s)<<endl;
		cout<<"ds_dot:"<<(set_speed - car_speed)<<endl;


		coefficients_x = TrajectoryGen::JMT(initial_state_x,final_state_x,Total_time);
		coefficients_y = TrajectoryGen::JMT(initial_state_y,final_state_y,Total_time);

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

		avg_speed = (car_speed_k+set_speed)/2;

		if(avg_speed > speed_limit){
			avg_speed = speed_limit;
		}

		final_s = avg_speed*Total_time + end_path_s;

		cout<<"Car speed:"<<car_speed_k<<endl;
		cout<<"Car S:"<<car_s<<endl;
		cout<<"Final S:"<<final_s<<endl;
		cout<<"End Path S:"<<end_path_s<<endl;

		cout<<"ds:"<<(final_s-end_path_s)<<endl;
		cout<<"ds_dot:"<<(set_speed - car_speed_k)<<endl;

		car_pos_final_xy = tools.getXY(final_s,d,maps_s,maps_x,maps_y);

		cout<<"x_diff:"<<(next_x_vals[prev_path_size-1]- x_k)<<endl;
		cout<<"y_diff:"<<(next_y_vals[prev_path_size-1]- y_k)<<endl;

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

		coefficients_x = TrajectoryGen::JMT(initial_state_x,final_state_x,Total_time);
		coefficients_y = TrajectoryGen::JMT(initial_state_y,final_state_y,Total_time);

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

vector<double> TrajectoryGen::JMT(vector< double> start, vector <double> end, double T)
{
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


