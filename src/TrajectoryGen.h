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

using namespace std;

class TrajectoryGen{
public:

	TrajectoryGen();

	virtual ~TrajectoryGen();

	vector<vector<double>> KeepLane(const double &car_s,const double &car_d,const double &car_speed,const vector<double> &maps_s,
		const vector<double> &maps_x, const vector<double> &maps_y,const vector<double> &previous_path_x, const vector<double> &previous_path_y);

	vector<double> JMT(vector< double> start, vector <double> end, double T);


};


#endif /* SRC_TRAJECTORYGEN_H_ */
