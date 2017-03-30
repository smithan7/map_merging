//============================================================================
// Name        : map_align.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "KeypointMatcher.h"
#include "FastMarching2Star.h"

#include <vector>

using namespace std;
using namespace cv;

int main() {

	KeypointMatcher kpMatcher;

	Mat sub_mat = imread("/home/rdml/git/inference_coordination/maps/gmapping_split1.jpg", CV_8UC3);
	Mat set_mat = imread("/home/rdml/git/inference_coordination/maps/gmapping.jpg", CV_8UC3);

	cout << "sub_mat.size(): " << sub_mat.size() << endl;
	cout << "set_mat.size(): " << set_mat.size() << endl;

	//resize(sub, sub, Size(), 0.2, 0.2);
	//resize(set, set, Size(), 0.2, 0.2);

	//threshold(set,set,127,255,0);
	//threshold(sub, sub, 127, 255,0);



	//sub_mat = kpMatcher.generateStartingConfig(set_mat, sub_mat);
	//cout << "sub_mat.size(): " << sub_mat.size() << endl;
	//cout << "set_mat.size(): " << set_mat.size() << endl;

	vector<Point2f> set_pts, sub_pts;
	kpMatcher.getWallPts(set_mat, set_pts);
	kpMatcher.getWallPts(sub_mat, sub_pts);
	vector<Point2f> sub_raw = sub_pts;
	cout << "set.size(): " << set_pts.size() << endl;
	cout << "sub.size(): " << sub_pts.size() << endl;


	FastMarching2Star fm2( set_mat, set_pts );
	//fm2.displayMap();
	fm2.inflateWalls_to_crashRadius();
	//fm2.displayMap();
	fm2.inflateWalls_to_inflationRadius();
	//fm2.displayMap();
	fm2.wavePropagation();
	vector<Point2f> path = fm2.findPath();
	fm2.displayPath_over_map( path );
	fm2.displayPath_over_time( path );
	fm2.displayPath_over_speedMap( path );

	Mat lastGood;
	Mat transform;
	vector<float> dists;
	double lastDist = INFINITY;
	int test = 0;

	Mat H_kept = cv::Mat::eye(3,3,CV_64FC1);
	float theta = 3.14159*10/360;
	H_kept.at<double>(0,0) = cos(theta);
	H_kept.at<double>(0,1) = -sin(theta);
	H_kept.at<double>(0,2) = 230;
	H_kept.at<double>(1,0) = sin(theta);
	H_kept.at<double>(1,1) = cos(theta);
	H_kept.at<double>(1,2) = 160;
	H_kept.at<double>(2,0) = 0.0;
	H_kept.at<double>(2,1) = 0.0;
	H_kept.at<double>(2,2) = 1.0;

	Mat rot_temp = Mat(sub_raw, CV_32FC1);
	Mat rot_matches = Mat::zeros(rot_temp.size(), CV_32FC1);
	perspectiveTransform(rot_temp, rot_matches, H_kept);

	sub_pts = rot_matches;

	while(true) {
		vector<Point2f> set_matches, sub_matches;

	    double distSum = kpMatcher.linearDist(sub_pts, set_pts, set_matches, sub_matches, 300.0);

	    kpMatcher.plotMatches(set_mat, set_pts, sub_matches, set_matches);

	    lastDist = distSum;
		float param[3] = {0,0,0};
		kpMatcher.fit3DofQUADRATIC(set_matches, sub_matches, param, set_matches[0]);

		if(abs(param[0]) + abs(param[1]) + abs(param[2]) < 0.01){
			break;
		}

		cv::Mat H = cv::Mat(3,3,CV_64FC1);
		H.at<double>(0,0) = cos(param[0]);
		H.at<double>(0,1) = -sin(param[0]);
		H.at<double>(0,2) = param[1];
		H.at<double>(1,0) = sin(param[0]);
		H.at<double>(1,1) = cos(param[0]);
		H.at<double>(1,2) = param[2];
		H.at<double>(2,0) = 0.0;
		H.at<double>(2,1) = 0.0;
		H.at<double>(2,2) = 1.0;

		H_kept = H_kept * H;

		Mat rot_temp = Mat(sub_raw, CV_32FC1);
		Mat rot_matches = Mat::zeros(rot_temp.size(), CV_32FC1);
		perspectiveTransform(rot_temp, rot_matches, H_kept);

		sub_pts = rot_matches;
	}
	cerr << "done" <<endl;
	waitKey(0);

	return 0;
}
