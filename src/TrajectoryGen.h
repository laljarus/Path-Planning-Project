/*
 * TrajectoryGen.h
 *
 *  Created on: Aug 30, 2018
 *      Author: laljarus
 */

#ifndef SRC_TRAJECTORYGEN_H_
#define SRC_TRAJECTORYGEN_H_

#include <iostream>
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "tools.h"
#include <unordered_map>

using namespace std;

class PathPlanning{
public:

	double car_s,car_d,car_speed,end_path_s,end_path_d,car_yaw;
	double dt = 0.02;
	vector<double> maps_s,maps_x,maps_y,previous_path_x,previous_path_y;
	vector<vector<double>> sensor_fusion;

	unordered_map<double,vector<double>> SensorFusion_map;
	vector<vector<double>> lane_info;

	string state;
	bool initialized;

	PathPlanning();
	void Init(const double &car_s_in,const double &car_d_in,const double &car_speed_in,const vector<double> &maps_s_in,
			const vector<double> &maps_x_in,const vector<double> &maps_y_in,const vector<double> &previous_path_x_in,const vector<double> &previous_path_y_in,
			const double end_path_s_in,const double end_path_d_in,const vector<vector<double>> &sensor_fusion_in,const double &car_yaw_in,const bool &initialize_in);

	virtual ~PathPlanning();

	vector<vector<double>> GenerateTrajectory();

	vector<double> JMT(vector< double> start, vector <double> end, double T);

	double state_machine();

	void sensor_fusion_processing();

	double keep_lane();



};


#endif /* SRC_TRAJECTORYGEN_H_ */
