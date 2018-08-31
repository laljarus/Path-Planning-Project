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

	vector<vector<double>> MinimumJerkTrajectory(const double &car_s,const vector<double> &maps_s,
		const vector<double> &maps_x, const vector<double> &maps_y,const vector<double> &previous_path_x, const vector<double> &previous_path_y);
};


#endif /* SRC_TRAJECTORYGEN_H_ */
