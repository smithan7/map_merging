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

double KeypointMatcher::linearDist(vector<Point2f> &sub_pts, vector<Point2f> &set_pts, vector<Point2f> &sub_matches, vector<Point2f> &set_matches, float tol){
	double distSum = 0;

	int numPts = sub_pts.size();//50;

	for(size_t i=0; i<numPts; i++){
		int iRand = rand() % sub_pts.size();
		double minDist = INFINITY;
		int mindex = -1;
		for(size_t j=0; j<set_pts.size(); j++){
			double d = pow(set_pts[j].x - sub_pts[iRand].x,2) + pow(set_pts[j].y - sub_pts[iRand].y,2);
			if( d < minDist ){
				minDist = d;
				mindex = j;
			}
		}
		if(minDist < tol){
			distSum += minDist;
			sub_matches.push_back( sub_pts[iRand] ); // only use pts within a set tolerance
			set_matches.push_back( set_pts[mindex] );
		}
		else{
			distSum += 2*tol;
			i--;
		}
	}
	return distSum;
}

void KeypointMatcher::plotMatches(Mat &set_mat, vector<Point2f> &set_pts, vector<Point2f> &sub_matches, vector<Point2f> &set_matches){
	Mat t = Mat::zeros( set_mat.size(), CV_8UC3);

	Vec3b red(0,0,255);
	Vec3b blue(0,255,0);
	Vec3b white(255,255,255);

	for(size_t i=0; i<set_pts.size(); i++){
		t.at<Vec3b>(set_pts[i]) = white;
	}

	for(size_t i=0; i<sub_matches.size(); i++){
		t.at<Vec3b>(sub_matches[i]) = red;
		t.at<Vec3b>(set_matches[i]) = blue;
	}

	namedWindow("t", WINDOW_NORMAL);
	imshow("t", t);
	waitKey(1);
}

Mat KeypointMatcher::getRotationMatrix(vector<Point2f> &src, vector<Point2f> &dst){
	// compute a covariance matrix

	bool debug = true;
	float Cxx = 0.0, Cxy = 0.0, Cyx = 0.0, Cyy = 0.0;
	for (int i=0; i<src.size(); i++) {
	    Cxx += src[i].x*dst[i].x;
	    Cxy += src[i].x*dst[i].y;
	    Cyx += src[i].y*dst[i].x;
	    Cyy += src[i].y*dst[i].y;
	}
	Mat Mcov = (Mat_<float>(2, 2)<<Cxx, Cxy, Cyx, Cyy);
	if (debug)
	    cerr<<"Covariance Matrix "<<Mcov<<endl;

	// SVD
	cv::SVD svd;
	svd = SVD(Mcov, SVD::FULL_UV);
	if (debug) {
	    cerr<<"U = "<<svd.u<<endl;
	    cerr<<"W = "<<svd.w<<endl;
	    cerr<<"V transposed = "<<svd.vt<<endl;
	}

	// rotation = V*Ut
	Mat V = svd.vt.t();
	Mat Ut = svd.u.t();
	float det_VUt = determinant(V*Ut);
	Mat W = (Mat_<float>(2, 2)<<1.0, 0.0, 0.0, det_VUt);
	float rot[4];
	Mat R_est(2, 2, CV_32F, rot);
	R_est = V*W*Ut;
    cerr<<"Rotation matrix: "<<R_est<<endl;

	return R_est;
}

// fits 3DOF (rotation and translation in 2D) with least squares.
void KeypointMatcher::fit3DofQUADRATIC(const vector<Point2f>& src_, const vector<Point2f>& dst_, float* param, const Point2f center) {
// http://stackoverflow.com/questions/35765546/convert-from-mat-to-point2f-or-point3f
    const bool debug = false;                   // print more debug info
    assert(dst_.size() == src_.size());
    int N = src_.size();

    // collect inliers
    vector<Point2f> src, dst;
    int ninliers;
	ninliers = N;
	src = src_; // copy constructor
	dst = dst_;

    if (ninliers<2) {
        param[0] = 0.0f; // default return when there is not enough points
        param[1] = 0.0f;
        param[2] = 0.0f;
    }

    /* Algorithm: Least-Square Rigid Motion Using SVD by Olga Sorkine
     * http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
     *
     * Subtract centroids, calculate SVD(cov),
     * R = V[1, det(VU')]'U', T = mean_q-R*mean_p
     */

    // Calculate data centroids
    Scalar centroid_src = mean(src);
    Scalar centroid_dst = mean(dst);
    Point2f center_src(centroid_src[0], centroid_src[1]);
    Point2f center_dst(centroid_dst[0], centroid_dst[1]);
    if (debug)
        cerr<<"Centers: "<<center_src<<", "<<center_dst<<endl;

    // subtract centroids from data
    for (int i=0; i<ninliers; i++) {
        src[i] -= center_src;
        dst[i] -= center_dst;
    }

    // compute a covariance matrix
    float Cxx = 0.0, Cxy = 0.0, Cyx = 0.0, Cyy = 0.0;
    for (int i=0; i<ninliers; i++) {
        Cxx += src[i].x*dst[i].x;
        Cxy += src[i].x*dst[i].y;
        Cyx += src[i].y*dst[i].x;
        Cyy += src[i].y*dst[i].y;
    }
    Mat Mcov = (Mat_<float>(2, 2)<<Cxx, Cxy, Cyx, Cyy);
    Mcov /= (ninliers-1);
    if (debug)
        cerr<<"Covariance-like Matrix "<<Mcov<<endl;

    // SVD of covariance
    cv::SVD svd;
    svd = SVD(Mcov, SVD::FULL_UV);
    if (debug) {
        cerr<<"U = "<<svd.u<<endl;
        cerr<<"W = "<<svd.w<<endl;
        cerr<<"V transposed = "<<svd.vt<<endl;
    }

    // rotation (V*Ut)
    Mat V = svd.vt.t();
    Mat Ut = svd.u.t();
    float det_VUt = determinant(V*Ut);
    Mat W = (Mat_<float>(2, 2)<<1.0, 0.0, 0.0, det_VUt);
    float rot[4];
    Mat R_est(2, 2, CV_32F, rot);
    R_est = V*W*Ut;
    if (debug)
        cerr<<"Rotation matrix: "<<R_est<<endl;

    float cos_est = rot[0];
    float sin_est = rot[2];
    float ang = atan2(sin_est, cos_est);

    // translation (mean_dst - R*mean_src)
    Point2f center_srcRot = Point2f(
            cos_est*center_src.x - sin_est*center_src.y,
            sin_est*center_src.x + cos_est*center_src.y);
    Point2f T_est = center_dst - center_srcRot;

    // Final estimate msg
    if (debug)
        cerr<<"Estimate = "<< ang*360/6.26 <<"deg., T = "<<T_est<<endl;

    param[0] = ang; // rad
    param[1] = T_est.x;
    param[2] = T_est.y;
} // fit3DofQUADRATIC()

// calculates squared error from two point mapping; assumes rotation around Origin.
inline float KeypointMatcher::sqErr_3Dof(Point2f p1, Point2f p2,
        float cos_alpha, float sin_alpha, Point2f T) {

    float x2_est = T.x + cos_alpha * p1.x - sin_alpha * p1.y;
    float y2_est = T.y + sin_alpha * p1.x + cos_alpha * p1.y;
    Point2f p2_est(x2_est, y2_est);
    Point2f dp = p2_est-p2;
    float sq_er = dp.dot(dp); // squared distance

    //cerr<<dp<<endl;
    return sq_er;
}

Mat KeypointMatcher::generateStartingConfig(Mat &set, Mat &sub_in){
	Mat sub_out = Mat::ones(set.size(), CV_8UC3);;
	for(int i=0; i<set.cols; i++){
		for(int j=0; j<set.rows; j++){
			Point p(i,j);
			sub_out.at<Vec3b>(p) = Vec3b(255,255,255);
		}
	}

	int dx = 230;//rand() % set.cols - sub_in.cols;
	int dy = 160;//rand() % set.rows - sub_in.rows;

	for(int i=0; i<sub_in.cols; i++){
		for(int j=0; j<sub_in.rows; j++){
			Point p(i,j);
			Point pp(i+dx, j+dy);
			sub_out.at<uchar>(pp) = sub_in.at<uchar>(p);
		}
	}
	Point2f center(dx+sub_in.cols/2, dy+sub_in.rows/2);
	double angle = 0;//6.28 * double(rand() % 360) / 360;
	Mat rotMat = getRotationMatrix2D(center, angle, 1.0);
	//warpAffine( sub_out, sub_out, rotMat, sub_out.size());

	return sub_out;
}

void KeypointMatcher::getWallPts(Mat &mat, vector<Point2f> &pts){
	for(int i=0; i<mat.cols; i++){
		for(int j=0; j<mat.rows; j++){
			Point p(i,j);
			if(mat.at<uchar>(p) <= 127){
				pts.push_back( p );
			}
		}
	}
}
