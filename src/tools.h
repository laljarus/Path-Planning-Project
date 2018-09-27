/*
 * tools.h
 *
 *  Created on: Aug 30, 2018
 *      Author: laljarus
 */

#ifndef SRC_TOOLS_H_
#define SRC_TOOLS_H_

#include <iostream>
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

class Tools{
public:
	// constructor
	Tools();

	// destructor
	virtual ~Tools();


	double deg2rad(double x);
	double rad2deg(double x);

	// distance calculation
	double distance(double x1, double y1, double x2, double y2);

	// Find closest waypoint
	int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

	// Find Next Waypoint
	int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

	// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
	vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

	// Transform from Frenet s,d coordinates to Cartesian x,y
	vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

	



};



#endif /* SRC_TOOLS_H_ */
