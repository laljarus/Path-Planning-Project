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

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Tools tools;

TrajectoryGen::TrajectoryGen(){}
TrajectoryGen::~TrajectoryGen(){}

vector<vector<double>> TrajectoryGen::KeepLane(const double &car_s,const double &car_d,const double &car_speed,const vector<double> &maps_s,
		const vector<double> &maps_x, const vector<double> &maps_y,const vector<double> &previous_path_x, const vector<double> &previous_path_y){

	//double acc = 6;
	double set_speed = 20; // velocity in m/s
	double Total_time = 50*0.02; // time in seconds
	//static int counter = 0;

	/*set_speed += counter*0.02*acc;
	counter++;

	if(set_speed>20){
		set_speed = 20;
	}*/

	double set_speed_jmt;

	if((set_speed - car_speed)> 6)
	{
		set_speed_jmt = car_speed + 6;
	}
	else if(((set_speed - car_speed))<-6)
	{
		set_speed_jmt = car_speed - 6;
	}else {
		set_speed_jmt = set_speed;
	}

	vector<double> car_pos_init_xy = tools.getXY(car_s,car_d,maps_s,maps_x,maps_y);
	double final_s = set_speed_jmt*Total_time + car_s;

	vector<double> car_pos_final_xy = tools.getXY(final_s,car_d,maps_s,maps_x,maps_y);

	vector<double> initial_state_x;
	vector<double> initial_state_y;
	vector<double> final_state_x;
	vector<double> final_state_y;


	initial_state_x.push_back(car_pos_init_xy[0]);
	initial_state_x.push_back(car_speed*cos(car_pos_init_xy[2]));
	initial_state_x.push_back(0);

	initial_state_y.push_back(car_pos_init_xy[1]);
	initial_state_y.push_back(car_speed*sin(car_pos_init_xy[2]));
	initial_state_y.push_back(0);


	final_state_x.push_back(car_pos_final_xy[0]);
	final_state_x.push_back(set_speed_jmt*cos(car_pos_final_xy[2]));
	final_state_x.push_back(0);

	final_state_y.push_back(car_pos_final_xy[1]);
	final_state_y.push_back(set_speed_jmt*sin(car_pos_final_xy[2]));
	final_state_y.push_back(0);

	vector<double> coefficients_x,coefficients_y;

	coefficients_x = TrajectoryGen::JMT(initial_state_x,final_state_x,Total_time);
	coefficients_y = TrajectoryGen::JMT(initial_state_y,final_state_y,Total_time);

	double time;
	double SampleTime = 0.02;

	vector<double> pos;
	double x,y;
	double d = 6;
	vector<double> next_x_vals;
	vector<double> next_y_vals;

	int prev_path_weight = previous_path_x.size();

	if(previous_path_x.size() != 0){

		for(int i = 0; i < prev_path_weight; i++)
			{
			    next_x_vals.push_back(previous_path_x[i]);
			    next_y_vals.push_back(previous_path_y[i]);
			}
	}
	for(int i = prev_path_weight;i<51;i++)
	{
		time = SampleTime*i;
		x = coefficients_x[0] + coefficients_x[1]*time+ coefficients_x[2]*pow(time,2)
				+ coefficients_x[3]*pow(time,3)+coefficients_x[4]*pow(time,4)+coefficients_x[5]*pow(time,5);

		y = coefficients_y[0] + coefficients_y[1]*time+ coefficients_y[2]*pow(time,2)
						+ coefficients_y[3]*pow(time,3)+coefficients_y[4]*pow(time,4)+coefficients_y[5]*pow(time,5);

		next_x_vals.push_back(x);
		next_y_vals.push_back(y);
	}





	cout<<"Previous Path Size: "<<prev_path_weight<<endl;
	cout<<"Path Size:"<<next_x_vals.size()<<endl;

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


