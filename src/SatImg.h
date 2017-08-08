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
#include <iomanip>

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
		SatImg( string image_dir, string image_name, vector<double> corners, vector<double> start, vector<double> goal ) ;
		virtual ~SatImg() ;

		int GetNumVertices() const {return numVertices ;}
		int GetNumEdges() const {return numEdges ;}
		void SetNumEdges(int nEdges) {numEdges = nEdges ;}

	private:
		int numVertices ;
		int numEdges ;


		void get_task_set(cv::Mat map, cv::Mat img, std::vector<std::vector<double> > &vertices_pixels, string mapName);

		vector<vector<double> > importVertices();
		vector<vector<double> > gps_to_pixels( vector<double> corners, vector<vector<double> > &vertices, double x, double y);

		vector<double> ne_corner, nw_corner, se_corner, sw_corner;
		vector<double> map_size;
		double map_height_m, map_width_m;
		double mat_width_p, mat_height_p;
		double pixel_height_m, pixel_width_m;
		void setMapParams( vector<double> corners );
		void setNumVertices();


		vector< vector< double > > makeVertices(double x, double y, int numVerts, string mapName);
		vector< vector<double> > convertVerticesToLatLon( vector< vector<double> > vertices, vector<double> corners, vector<double> start, vector<double> goal, double x, double y);
		vector < vector < int > > Bresenham(double x1, double y1, double x2, double y2) ;
		double haversineDistance( vector<double> p1, vector<double> p2);
		vector<double> GPStoMap( vector<double> corners, vector<double> point, double x, double y);
		vector<double> mapToGPS( vector<double> corners, vector<double> point, double x, double y);
		vector < double > CalcMeanVar(vector< int > points) ;
		vector< vector<double> > RadiusConnect(vector< vector<double> > vertices_pixel, vector< vector<double> > vertices_gps, double radius, cv::Mat, string mapName) ;
		vector< vector<double> > GPSRadiusConnect(vector< vector<double> > vertices, double radius, cv::Mat img, string mapName, double flight_speed);
		double EuclideanDistance(vector<double> v1, vector<double> v2) ;
		void show_graph( cv::Mat img, vector<vector<double> > &vertices, vector<vector<double> > &vertices_gps, vector<vector<double> > &edges );
};

#endif /* SATIMG_H_ */
