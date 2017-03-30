/*
 * KeypointMatcher.h
 *
 *  Created on: Jan 26, 2017
 *      Author: rdml
 */

#ifndef KEYPOINTMATCHER_H_
#define KEYPOINTMATCHER_H_

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

class KeypointMatcher{
public:
	KeypointMatcher();
	virtual ~KeypointMatcher();

	void getWallPts(Mat &mat, vector<Point2f> &pts);
	void plotMatches(Mat &set, vector<Point2f> &p_set, vector<Point2f> &res, vector<Point2f> &pair);
	Mat generateStartingConfig(Mat &set, Mat &sub_in);
	double linearDist(vector<Point2f> &p_sub, vector<Point2f> &p_set, vector<Point2f> &pair, vector<Point2f> &m_sub, float tol);

	Mat getRotationMatrix(vector<Point2f> &src, vector<Point2f> &dst);

	inline float sqErr_3Dof(Point2f p1, Point2f p2, float cos_alpha, float sin_alpha, Point2f T);
	void fit3DofQUADRATIC(const vector<Point2f>& src_, const vector<Point2f>& dst_, float* param, const Point2f center);

};

#endif /* KEYPOINTMATCHER_H_ */
