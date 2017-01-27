/*
 * KeypointMatcher.cpp
 *
 *  Created on: Jan 26, 2017
 *      Author: rdml
 */

#include "KeypointMatcher.h"

KeypointMatcher::KeypointMatcher(){
	// TODO Auto-generated constructor stub

}

KeypointMatcher::~KeypointMatcher() {
	// TODO Auto-generated destructor stub
}

void KeypointMatcher::computeKeyPts(Mat set, Mat sub, char* detectorType, bool showKpts){

    detector = cv::FeatureDetector::create(detectorType);

    detector->detect(set, kp_set);
    detector->detect(sub, kp_sub);

    if( showKpts ){
		Mat kp_sub_mat, kp_set_mat;
		drawKeypoints( sub, kp_sub, kp_sub_mat);
		imshow("kp_sub",kp_sub_mat);

		drawKeypoints( set, kp_set, kp_set_mat);
		imshow("kp_set",kp_set_mat);
		waitKey(0);
    }
}

double KeypointMatcher::flann_knn(Mat& m_destinations, Mat& m_object, vector<int>& ptpairs, vector<float>& dists) {
    // find nearest neighbors using FLANN
    cv::Mat m_indices(m_object.rows, 1, CV_32S);
    cv::Mat m_dists(m_object.rows, 1, CV_32F);

    Mat dest_32f; m_destinations.convertTo(dest_32f,CV_32FC2);
    Mat obj_32f; m_object.convertTo(obj_32f,CV_32FC2);

    assert(dest_32f.type() == CV_32F);

    cerr << "b" << endl;

    cv::flann::Index flann_index(dest_32f, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees
    cerr << "b+" << endl;
    flann_index.knnSearch(obj_32f, m_indices, m_dists, 1, cv::flann::SearchParams(64) );

    cerr << "c" << endl;

    int* indices_ptr = m_indices.ptr<int>(0);
    //float* dists_ptr = m_dists.ptr<float>(0);
    for (int i=0;i<m_indices.rows;++i) {
        ptpairs.push_back(indices_ptr[i]);
    }

    dists.resize(m_dists.rows);
    m_dists.copyTo(Mat(dists));

    return cv::sum(m_dists)[0];
}

void KeypointMatcher::computePoints(){
	for(size_t i=0; i<kp_set.size(); i++){
		p_set.push_back(kp_set[i].pt);
	}

	for(size_t i=0; i<kp_sub.size(); i++){
		p_sub.push_back(kp_sub[i].pt);
	}
}

double KeypointMatcher::linearDist(vector<Point2f> &p_sub, vector<Point2f> &p_set, vector<Point2f> &pair, vector<Point2f> & m_sub, vector<float> &dist){
	double distSum = 0;
	for(size_t i=0; i<p_sub.size(); i++){
		double minDist = INFINITY;
		int mindex = -1;
		for(size_t j=0; j<p_set.size(); j++){
			double d = abs(p_set[j].x - p_sub[i].x) + abs(p_set[j].y - p_sub[i].y);
			if( d < minDist ){
				minDist = d;
			}
		}
		if(minDist < 10){
			distSum += minDist;
			m_sub.push_back( p_sub[i] ); // only use pts within a set tolerance
			pair.push_back( p_set[mindex] );
			dist.push_back( minDist );
		}
	}
	return distSum;
}

void KeypointMatcher::plotMatches(Mat &set, vector<Point2f> &p_set, vector<Point2f> &res, vector<Point2f> &pair){
	Mat t = Mat::zeros( set.size(), CV_8UC3);

	Vec3b red(255,0,0);
	Vec3b blue(0,255,0);
	Vec3b white(255,255,255);
	for(int i=0; i<p_set.size(); i++){
		t.at<Vec3b>(p_set[i]) = white;
	}

	for(int i=0; i<res.size(); i++){
		t.at<Vec3b>(res[i]) = red;
		t.at<Vec3b>(pair[i]) = blue;
	}

	namedWindow("t", WINDOW_NORMAL);
	imshow("t", t);
	waitKey(0);


}

void KeypointMatcher::getWallPts(Mat &set, Mat &sub){

	for(int i=0; i<set.cols; i++){
		for(int j=0; j<set.rows; j++){
			Point p(i,j);
			if(set.at<uchar>(p) <= 127){
				p_set.push_back( p );
			}
		}
	}

	for(int i=0; i<sub.cols; i++){
		for(int j=0; j<sub.rows; j++){
			Point p(i,j);
			if(sub.at<uchar>(p) <= 127){
				p.x += 250;
				p.y += 175;
				p_sub.push_back( p );
			}
		}
	}
}

void KeypointMatcher::findBestReansformSVD(Mat& _m, Mat& _d) {
    Mat m; _m.convertTo(m,CV_32F);
    Mat d; _d.convertTo(d,CV_32F);

    Scalar d_bar = mean(d);
    Scalar m_bar = mean(m);
    Mat mc = m - m_bar;
    Mat dc = d - d_bar;

    mc = mc.reshape(1); dc = dc.reshape(1);

    Mat H(2,2,CV_32FC1);
    for(int i=0;i<mc.rows;i++) {
        Mat mci = mc(Range(i,i+1),Range(0,2));
        Mat dci = dc(Range(i,i+1),Range(0,2));
        H = H + mci.t() * dci;
    }

    cv::SVD svd(H);

    Mat R = svd.vt.t() * svd.u.t();
    double det_R = cv::determinant(R);
    if(abs(det_R + 1.0) < 0.0001) {
        float _tmp[4] = {1,0,0,cv::determinant(svd.vt*svd.u)};
        R = svd.u * Mat(2,2,CV_32FC1,_tmp) * svd.vt;
    }
#ifdef BTM_DEBUG
    //for some strange reason the debug version of OpenCV is flipping the matrix
    R = -R;
#endif
    float* _R = R.ptr<float>(0);
    Scalar T(d_bar[0] - (m_bar[0]*_R[0] + m_bar[1]*_R[1]),d_bar[1] - (m_bar[0]*_R[2] + m_bar[1]*_R[3]));

    m = m.reshape(1);
    m = m * R;
    m = m.reshape(2);
    m = m + T;// + m_bar;
    m.convertTo(_m,CV_32S);
}


void KeypointMatcher::computeHomography(){
	Mat H = findHomography(p_set, p_sub, CV_RANSAC);
}

void KeypointMatcher::computeDescriptors(Mat set, Mat sub, char* extractorType){


    extractor = cv::DescriptorExtractor::create(extractorType);

    extractor->compute(set, kp_set, ds_set);
    extractor->compute(sub, kp_sub, ds_sub);

}

void KeypointMatcher::computeknnMatches(Mat set, Mat sub, char* extractorType, char* matcherType, int k){


    if (!ds_set.empty() && !ds_sub.empty()) {
    	cerr << "sets not empty" << endl;
    	if(strcmp(matcherType, "BRUTE_FORCE" ) !=0 ){
            if ( strcmp(extractorType, "ORB") !=0 || strcmp(extractorType, "BRISK") !=0 || strcmp(extractorType, "BRIEF") !=0 ){
                matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
            } else {
                matcher = cv::DescriptorMatcher::create("BruteForce");
            }
    	}
    	else if(strcmp(matcherType, "FLANN" ) !=0 ){
        	matcher = cv::DescriptorMatcher::create("FlannBased");
        }
        else{
        	matcher = cv::DescriptorMatcher::create("BruteForce");
        }

    	matcher = cv::DescriptorMatcher::create("BruteForce");
    	matcher->knnMatch(ds_set, ds_sub, matches, k);
        if (matches.size() > 0) {
        	for(int i=0; i< matches.size(); i++){
				matchesMask = std::vector<bool>(matches.size(), true);
				matched = true;

				Mat img_matches;
				drawMatches( sub, kp_sub, set, kp_set, matches[i], img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
				imshow("m",img_matches);
				waitKey(0);
        	}
        }
        else {
            std::cout << "No matching features could be found." << std::endl;
        }
    }
    else {
        matched = false;
        std::cout << "No descriptors could be extracted." << std::endl;
    }
}

void KeypointMatcher::homographyFilterMatches(char* method) {
    if (!matched) {
        std::cout << "Matching was not yet executed or no matches have been found." << std::endl;
        return;
    }
    std::vector<cv::Point2d> matchingPoints1, matchingPoints2;
    matchingPoints1.reserve(matches.size());
    matchingPoints2.reserve(matches.size());
    std::vector<int> indices;
    indices.reserve(matches.size());
    for (size_t i = 0; i < matchesMask.size(); ++i) {
        if (matchesMask[i] == true) {
            matchingPoints1.push_back(kp_set[matches[i][0].queryIdx].pt);
            matchingPoints2.push_back(kp_sub[matches[i][0].trainIdx].pt);
            indices.push_back(i);
        }
    }


/*
    int meth;
    if (!strcmp(method, "RANSAC")){
    	meth = cv::RANSAC;
    }
    else{
    	meth = cv::LMEDS;
    }
    */
    Mat mask;
    double ransacReprojThreshold=3;
    //Mat H = findHomography(matchingPoints1, matchingPoints2, CV_RANSAC, ransacReprojThreshold, mask);

    for (int i = 0; i < mask.rows; ++i) {
        matchesMask[indices[i]] = matchesMask[indices[i]] && (mask.at<uchar>(i, 0) == 1);
    }
}
