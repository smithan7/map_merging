/*
 * SatImg.h
 *
 *  Created on: Apr 3, 2017
 *      Author: rdml
 */

#ifndef SATIMG_H_
#define SATIMG_H_

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


#include <fstream>
//#include "vertex.h"
//#include <numeric>


class SatImg {
public:

	public:
		SatImg( string image_name, int numVerts, vector<double> corners ) ;
		virtual ~SatImg() ;

		int GetNumVertices() const {return numVertices ;}
		void SetNumVertices(int nVerts) {numVertices = nVerts ;}
		int GetNumEdges() const {return numEdges ;}
		void SetNumEdges(int nEdges) {numEdges = nEdges ;}

	private:
		int numVertices ;
		int numEdges ;
		vector< vector< double > > makeVertices(double x, double y, int numVerts, string mapName);
		vector < vector < int > > Bresenham(double x1, double y1, double x2, double y2) ;
		vector < double > CalcMeanVar(vector< int > points) ;
		vector< vector<double> > RadiusConnect(vector< vector<double> > vertices, double radius, cv::Mat, string mapName) ;
		double EuclideanDistance(vector<double> v1, vector<double> v2) ;
		void show_graph( cv::Mat img, vector<vector<double> > &vertices, vector<vector<double> > &edges );
};

#endif /* SATIMG_H_ */
