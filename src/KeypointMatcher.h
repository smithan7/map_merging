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
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/video/tracking.hpp"

#include <vector>

using namespace std;
using namespace cv;

class KeypointMatcher{
public:
	KeypointMatcher();
	virtual ~KeypointMatcher();
	void computeKeyPts(Mat set, Mat sub, char* detectorType, bool showKpts);
	void computeDescriptors(Mat set, Mat sub, char* extractorType);
	void computeknnMatches(Mat set, Mat sub, char* extractorType, char* matcherType, int k);
	void homographyFilterMatches(char* method);
	void computeHomography();
	void getWallPts(Mat &set, Mat &sub);
	void plotMatches(Mat &set, vector<Point2f> &p_set, vector<Point2f> &res, vector<Point2f> &pair);

	void findBestReansformSVD(Mat& _m, Mat& _d);
	double flann_knn(Mat& m_destinations, Mat& m_object, vector<int>& ptpairs, vector<float>& dists);
	double linearDist(vector<Point2f> &p_sub, vector<Point2f> &p_set, vector<Point2f> &pair, vector<Point2f> &m_sub, vector<float> &dist);
	void computePoints();
	char* detectorType, extractorType, matcherType, alignType;
	cv::Ptr<cv::DescriptorExtractor> extractor;
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorMatcher> matcher;

	vector<KeyPoint> kp_set, kp_sub;
	vector<Point2f> p_set, p_sub;
	Mat ds_set, ds_sub;
	vector< vector<DMatch> > matches;
	bool matched;
	std::vector<bool> matchesMask;
};

#endif /* KEYPOINTMATCHER_H_ */
