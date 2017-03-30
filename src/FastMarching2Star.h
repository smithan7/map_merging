/*
 * FastMarching2Star.h
 *
 *  Created on: Mar 29, 2017
 *      Author: rdml
 */

#ifndef FASTMARCHING2STAR_H_
#define FASTMARCHING2STAR_H_

#include <iostream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/video/tracking.hpp"

#include <vector>

using namespace std;
using namespace cv;


class FastMarching2Star {
public:
	FastMarching2Star( Mat &map, vector<Point2f> &walls );
	void inflateWalls_to_crashRadius();
	void inflateWalls_to_inflationRadius();
	void displayMap();
	void displaySpeedMap();
	void displayTime();
	bool wavePropagation();
	Point2f getGradients( Point2f p );
	vector<Point2f> findPath();
	void displayPath_over_time( vector<Point2f> &path );
	void displayPath_over_map( vector<Point2f> &path );
	void displayPath_over_speedMap( vector<Point2f> &path );


	Mat map, speedMap, time;
	float cell2Meter;
	float fm_tol;
	float crashRadius;
	float inflationRadius;
	float maxSpeed;
	Point2f start, goal;

	virtual ~FastMarching2Star();
};

#endif /* FASTMARCHING2STAR_H_ */
