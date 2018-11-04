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
	int car_lane;
	vector<double> maps_s,maps_x,maps_y,previous_path_x,previous_path_y;
	vector<vector<double>> sensor_fusion;

	unordered_map<double,vector<double>> SensorFusion_map;
	vector<vector<double>> lane_info;

	bool initialized;

	PathPlanning();
	void Init(const double &car_s_in,const double &car_d_in,const double &car_speed_in,const vector<double> &maps_s_in,
			const vector<double> &maps_x_in,const vector<double> &maps_y_in,const vector<double> &previous_path_x_in,const vector<double> &previous_path_y_in,
			const double end_path_s_in,const double end_path_d_in,const vector<vector<double>> &sensor_fusion_in,const double &car_yaw_in,const bool &initialize_in);

	virtual ~PathPlanning();

	vector<vector<double>> GenerateTrajectory(double &set_speed,double &d,int &path_len);

	vector<double> JMT(vector<double> &start, vector<double> &end, double T);

	vector<vector<double>> state_machine();
	vector<vector<double>> predict(double &car_id,int &path_len);

	void sensor_fusion_processing();

	double keep_lane();

	double InefficiencyCost(double &target_speed,double &intended_lane);
	double CollisionCost(vector<vector<double>> &trajectory,double &intended_lane);
	double AccelerationAndJerkCost(vector<vector<double>> &trajectory);
	double MinDistance(vector<vector<double>> &trajectory_1, vector<vector<double>> &trajectory_2);



};


#endif /* SRC_TRAJECTORYGEN_H_ */
