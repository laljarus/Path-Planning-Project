/*
 * TrajectoryGen.cpp
 *
 *  Created on: Aug 30, 2018
 *      Author: laljarus
 */

#include <iostream>
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "tools.h"
#include "TrajectoryGen.h"
using namespace std;

Tools tools;

TrajectoryGen::TrajectoryGen(){}
TrajectoryGen::~TrajectoryGen(){}

vector<vector<double>> TrajectoryGen::MinimumJerkTrajectory(const double &car_s,const vector<double> &maps_s,
		const vector<double> &maps_x, const vector<double> &maps_y,const vector<double> &previous_path_x, const vector<double> &previous_path_y){

	double jerk = 4;
	double time;
	static double acc = 0;
	static double vel = 0;
	//static int counter = 0;
	double sampleTime = 0.02;

	vector<double> pos;
	double s;
	double d = 10;
	vector<double> next_x_vals;
	vector<double> next_y_vals;

	int path_size = previous_path_x.size();

	 for(int i = 0; i < path_size; i++)
	 {
	     next_x_vals.push_back(previous_path_x[i]);
	     next_y_vals.push_back(previous_path_y[i]);
	 }

	//counter +=1;

	if (acc < 10){
		acc += jerk*sampleTime;
	}
	if(vel<21){
		vel += acc*sampleTime;
	}

	for(int i = 0;i< 25;i++)
	{
		time = sampleTime*i;
		s = car_s + vel*time;
		pos = tools.getXY(s,d,maps_s,maps_x,maps_y);
		next_x_vals.push_back(pos[0]);
		next_y_vals.push_back(pos[1]);
	}

	vector<vector<double>> result;

	result.push_back(next_x_vals);
	result.push_back(next_y_vals);

	return result;
}


