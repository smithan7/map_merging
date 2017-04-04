/*
 * SatImg.cpp
 *
 *  Created on: Apr 3, 2017
 *      Author: rdml
 */

#include "SatImg.h"

SatImg::SatImg(string image_name, int numVerts, vector<double> corners){ // PRM-style graph connection{

	cv::Mat img = cv::imread(image_name, CV_LOAD_IMAGE_COLOR);
	double x, y;
	x = img.size().width-1;
	y = img.size().height-1;

	cv::namedWindow("sat_img_raw", cv::WINDOW_NORMAL);
	cv::imshow("sat_img_raw", img);
	cv::waitKey(1);

	cv::Mat blurred_img;
	cv::blur( img, blurred_img, Size( 20, 20 ), Point(-1,-1) );

	cv::namedWindow("sat_img_blurred", cv::WINDOW_NORMAL);
	cv::imshow("sat_img_blurred", blurred_img);
	cv::waitKey(1);


	cv::Mat thresholded;
	cv::inRange(blurred_img, cv::Scalar(0, 0, 0), cv::Scalar(80, 255, 80), thresholded);
	cv::namedWindow("sat_img_threshold", cv::WINDOW_NORMAL);
	cv::imshow("sat_img_threshold",thresholded);
	cv::waitKey(1);

    cout << "Generating Random Vertices in " << x << " by " << y << endl;
    vector< vector< double > > vertices;
    vertices = this->makeVertices(x,y,numVerts, image_name);

    double radius = sqrt(1.9098593171*x*y*(log((double)numVerts)/(double)numVerts)) ;
    cout << "Connecting with radius " << radius << endl;
	vector< vector<double> > edges = RadiusConnect(vertices, radius, thresholded, "test1") ;

	show_graph( img, vertices, edges );
}

void SatImg::show_graph( cv::Mat img, vector<vector<double> > &vertices, vector<vector<double> > &edges ){



	for(size_t i=0; i<vertices.size(); i++){
		cv::Point p( vertices[i][0], vertices[i][1] );
		cv::circle( img, p ,5, Scalar(0,0,255), -1);
	}

	cv::Mat img2 = img.clone();

	for(size_t i=0; i<edges.size(); i++){
		int v0 = edges[i][0];
		cv::Point p0( vertices[v0][0], vertices[v0][1] );
		int v1 = edges[i][1];
		cv::Point p1( vertices[v1][0], vertices[v1][1] );
		cv::line( img, p0, p1, Scalar(0,0,int(edges[i][2])), 1);
	}

	cv::namedWindow("sat_img_graph_mean", cv::WINDOW_NORMAL);
	cv::imshow("sat_img_graph_mean", img);
	cv::waitKey(1);

	double maxV = -1;
	for(size_t e=0; e<edges.size(); e++){
		if( edges[e][3] > maxV){
			maxV = edges[e][3];
		}
	}

	for(size_t i=0; i<edges.size(); i++){
		int v0 = edges[i][0];
		cv::Point p0( vertices[v0][0], vertices[v0][1] );
		int v1 = edges[i][1];
		cv::Point p1( vertices[v1][0], vertices[v1][1] );
		int val = 255*edges[i][3]/maxV;
		cv::line( img, p0, p1, Scalar(0,0,val), 1);
	}

	cv::namedWindow("sat_img_graph_var", cv::WINDOW_NORMAL);
	cv::imshow("sat_img_graph_var", img);
	cv::waitKey(1);

}


SatImg::~SatImg() {}



vector< vector< double > > SatImg::makeVertices(double x, double y, int numVerts, string mapName){

	vector< vector< double > > vertices;

	srand (time(NULL));
	int xx = x;
	int yy = y;

	for(int i = 0; i < numVerts; i++){
		vector<double> vert;
		if (i == 0){
			vert.push_back(0);
			vert.push_back(0);
		}
		else if (i == numVerts-1){
			vert.push_back( x );
			vert.push_back(y);
		}
		else{
			vert.push_back( rand() % xx );
			vert.push_back( rand() % yy );
		}
		vertices.push_back(vert);
	}
	return vertices;
}

vector< double > SatImg::CalcMeanVar(vector< int > points){

  double sum  = 0, mu = 0, sigma_sq = 0, sdev = 0;
  sum = accumulate(points.begin(), points.end(), 0.0);
  mu = sum / points.size();
  sdev = inner_product(points.begin(), points.end(), points.begin(), 0.0);
  sigma_sq = sdev/points.size() - mu*mu;
  vector< double > m_and_v;
  m_and_v.push_back(mu);
  m_and_v.push_back(sigma_sq);
  return m_and_v;
}

vector< vector< int > > SatImg::Bresenham(double x1, double y1, double x2, double y2){
	// Bresenham's line algorithm
	vector< vector< int > > pixels;
	const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));

	if(steep){
	  std::swap(x1, y1);
	  std::swap(x2, y2);
	}

	if(x1 > x2){
	  std::swap(x1, x2);
	  std::swap(y1, y2);
	}

	const float dx = x2 - x1;
	const float dy = fabs(y2 - y1);

	float error = dx / 2.0f;
	const int ystep = (y1 < y2) ? 1 : -1;
	int y = (int)y1;

	const int maxX = (int)x2;

	for(int x=(int)x1; x<maxX; x++){
	  vector < int > tmp;
	  if(steep){
		tmp.push_back(y);
		tmp.push_back(x);
	  }
	  else{
		tmp.push_back(x);
		tmp.push_back(y);
	  }

	  pixels.push_back(tmp);
	  error -= dy;
	  if(error < 0){
		y += ystep;
		error += dx;
	  }
	}
	return pixels;
}


// Connect vertices within specified radius variances based on images
vector< vector<double> > SatImg::RadiusConnect(vector< vector<double> > vertices, double radius, cv::Mat img, string mapName){
    cout << "IMAGE RADIUS CONNECT" << endl;
	srand(time(NULL));
	vector< vector<double> > edges(pow(vertices.size(),2), vector<double>(4)) ;
	int k = 0 ;
	for (size_t i = 0; i < vertices.size(); i++){
		for (size_t j = 0; j < vertices.size(); j++){
			double diff = EuclideanDistance(vertices[i], vertices[j]) ;
			if (diff <= radius && i != j){
				vector< vector < int > > pixels = Bresenham(vertices[i][0], vertices[i][1], vertices[j][0], vertices[j][1]);
				vector< int > color;
				for(size_t z = 0; z < pixels.size(); z++){
					color.push_back(double(img.at<unsigned char>(int(pixels[z][1]), int(pixels[z][0])))/255.0);
				}
				vector< double > MandV = CalcMeanVar(color); // change this to pixel values
				edges[k][0] = (double)i ;
				edges[k][1] = (double)j ;
				edges[k][2] = diff * MandV[0]; // EuclideanDistance(vertices[i], vertices[j]) + MandV[0]
				edges[k][3] = MandV[1]*20; // MandV[1] might need to scale this somehow... maybe based on euclidean distance
				k++ ;
			}
		}
	}
	edges.resize(k) ;

	// Write edges to txt file
	stringstream eFileName ;
	eFileName << "/home/rdml/git/map_align/SatGraph/" << mapName << "_edges.txt" ;
	ofstream edgesFile ;
	edgesFile.open(eFileName.str().c_str()) ;

	for (int i = 0; i < int( edges.size() ); i++){
		edgesFile << edges[i][0] << "," << edges[i][1] << "," << edges[i][2] << "," << edges[i][3] << "\n" ;
	}
	edgesFile.close() ;

	// Write vertices to txt file
	stringstream vFileName ;
	vFileName << "/home/rdml/git/map_align/SatGraph/" << mapName << "_verts.txt" ;
	ofstream vertsFile ;
	vertsFile.open(vFileName.str().c_str()) ;

	for (size_t i = 0; i < vertices.size(); i++){
		vertsFile << vertices[i][0] << "," << vertices[i][1] << "\n" ;
	}
	vertsFile.close() ;

	return edges ;
}

double SatImg::EuclideanDistance(vector<double> v1, vector<double> v2){
	double diff_x = v1[0] - v2[0];
	double diff_y = v1[1] - v2[1] ;
	double diff = sqrt(pow(diff_x,2) + pow(diff_y,2)) ;

	return diff;
}


