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

#include <vector>

using namespace std;
using namespace cv;

int main() {


	// points
	std::vector<cv::Point2f> p1;
	p1.push_back(cv::Point2f(0,0));
	p1.push_back(cv::Point2f(1,0));
	p1.push_back(cv::Point2f(0,1));

	// simple translation from p1 for testing:
	std::vector<cv::Point2f> p2;
	p2.push_back(cv::Point2f(1,1));
	p2.push_back(cv::Point2f(2,1));
	p2.push_back(cv::Point2f(1,2));

	cv::Mat R = cv::estimateRigidTransform(p1,p2,false);

	// extend rigid transformation to use perspectiveTransform:
	cv::Mat H = cv::Mat(3,3,R.type());
	H.at<double>(0,0) = R.at<double>(0,0);
	H.at<double>(0,1) = R.at<double>(0,1);
	H.at<double>(0,2) = R.at<double>(0,2);

	H.at<double>(1,0) = R.at<double>(1,0);
	H.at<double>(1,1) = R.at<double>(1,1);
	H.at<double>(1,2) = R.at<double>(1,2);

	H.at<double>(2,0) = 0.0;
	H.at<double>(2,1) = 0.0;
	H.at<double>(2,2) = 1.0;

	// compute perspectiveTransform on p1
	std::vector<cv::Point2f> result;
	cv::perspectiveTransform(p1,result,H);

	for(unsigned int i=0; i<result.size(); ++i){
	    std::cout << result[i] << std::endl;
	}

	Mat sub = imread("/home/rdml/git/inference_coordination/maps/gmapping_split1.jpg", CV_8UC3);
	Mat set = imread("/home/rdml/git/inference_coordination/maps/gmapping.jpg", CV_8UC3);

	//resize(sub, sub, Size(), 0.2, 0.2);
	//resize(set, set, Size(), 0.2, 0.2);

	//threshold(set,set,127,255,0);
	//threshold(sub, sub, 127, 255,0);

	KeypointMatcher kpMatcher;
	char detectorType[] = "FAST"; // FAST, STAR, SIFT, SURF, ORB, BRISK, MSE
	char extractorType[] = "ORB"; // SIFT, STAR, BRIEF, BRISK, ORB
	char matchType[] = "BRUTE_FORCE"; // BRUTE_FORCE, FLANN

	//kpMatcher.computeKeyPts(set, sub, detectorType, false);
	//kpMatcher.computePoints();
	kpMatcher.getWallPts(set, sub);
	//kpMatcher.computeDescriptors(set, sub, extractorType);
	//kpMatcher.computeknnMatches(set, sub, extractorType, matchType, 1);
	//kpMatcher.computeHomography(); // needs smae number of pts :(

	cerr << "kpMatcher.p_set.size(): " << kpMatcher.p_set.size() << endl;
	cerr << "kpMatcher.p_sub.size(): " << kpMatcher.p_sub.size() << endl;

	Mat lastGood;

	vector<Point2f> res = kpMatcher.p_sub;
	vector<Point2f> resBest = res;
	vector<Point2f> pair;
	vector<float> dists;
	double lastDist = INFINITY;
	while(true) {
	    pair.clear(); dists.clear();

	    //double dist = kpMatcher.flann_knn(destination, X, pair, dists);
	    vector<Point2f> m_sub;
	    double dist = kpMatcher.linearDist(res, kpMatcher.p_set, pair, m_sub, dists);
	    cerr << "dist: " << dist << endl;

	    kpMatcher.plotMatches(set, m_sub, res, pair);

	    if(lastDist <= dist) {
	    	resBest = res;
	        break;  //converged?
	    }

	    lastDist = dist;

	    cerr << "res.size()/pair.size(): " << res.size() << " / " << pair.size() << endl;

	    //Mat r = cv::estimateRigidTransform(res, pair, false);
	    Mat h = findHomography( m_sub, pair, CV_RANSAC);

	    Mat resMat = Mat(res, CV_32FC2);
	    Mat resDest = Mat::zeros(resMat.size(), CV_32FC2);
	    cerr << "resMat.size(): " << resMat.cols << " / " << resMat.rows << " , "<< resMat.type() << endl;
	    cerr << "r.size(): " << h.cols << " / " << h.rows << endl;
	    perspectiveTransform(resMat, resDest, h);

	    res = vector<Point2f>(resDest);

	}


	imshow("set", set);
	waitKey(1);

	imshow("sub", sub);
	waitKey(0);


	return 0;
}
