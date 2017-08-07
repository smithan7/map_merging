/*
 * SatImg.cpp
 *
 *  Created on: Apr 3, 2017
 *      Author: rdml
 */

#include "SatImg.h"

SatImg::SatImg(string image_dir, string image_name, vector<double> corners, vector<double> start, vector<double> goal){ // PRM-style graph connection{

    double radius = 20.0; //sqrt(1.9098593171*this->map_width_m*this->map_height_m*(log((double)this->numVertices)/(double)this->numVertices)) ;
    cout << "Connecting with radius " << radius << endl;


	cv::Mat img = cv::imread(image_dir + image_name + ".png", CV_LOAD_IMAGE_COLOR);
	//cv::Mat img = cv::imread("/home/rdml/git/map_align/short_hardware/easy1.PNG", CV_LOAD_IMAGE_COLOR);
	this->mat_width_p = img.size().width-1;
	this->mat_height_p = img.size().height-1;

	this->setMapParams( corners );
	this->setNumVertices( );

	cout << "numVertices: " << this->numVertices << endl;

	cv::namedWindow("sat_img_raw", cv::WINDOW_NORMAL);
	cv::imshow("sat_img_raw", img);
	cv::waitKey(1);

	cv::Mat blurred_img;
	cv::blur( img, blurred_img, Size( 20, 20 ), Point(-1,-1) );

	cv::namedWindow("sat_img_blurred", cv::WINDOW_NORMAL);
	cv::imshow("sat_img_blurred", blurred_img);
	cv::waitKey(1);


	cv::Mat thresholded = cv::Mat::zeros( img.rows, img.cols, CV_8UC1 );
	cv::Mat thresholded_blurred, max_mat;
	cv::inRange(blurred_img, cv::Scalar(0, 0, 0), cv::Scalar(50, 180, 50), thresholded);

	//cv::blur( thresholded, thresholded_blurred, Size( 5, 5 ), Point(-1,-1) );
	//for(int i=0; i<30; i++){
	//	cv::blur( thresholded_blurred, thresholded_blurred, Size( 100, 100 ));
	//	thresholded_blurred = cv::max(thresholded, thresholded_blurred);
	//}
	//thresholded = cv::max(thresholded, thresholded_blurred);

	//cv::cvtColor(thresholded,thresholded,CV_RGB2GRAY);
	cv::namedWindow("sat_img_threshold", cv::WINDOW_NORMAL);
	cv::imshow("sat_img_threshold", thresholded);
	cv::waitKey(1);

	vector< vector< double > > vertices_pixel, vertices_gps;
    /*
    cout << "Generating Random Vertices in " << mat_width_p << " by " << mat_height_p << endl;
    vertices_pixel = this->makeVertices(mat_width_p, mat_height_p,this->numVertices, image_name);
    vertices_pixel[0] = this->GPStoMap( corners, start, mat_width_p, mat_height_p);
    vertices_pixel[vertices_pixel.size()-1] = this->GPStoMap( corners, goal, mat_width_p, mat_height_p);

    cout << "vert: " << vertices_pixel[0][0] << " , " << vertices_pixel[0][1] << endl;
    cout << "vert: " << vertices_pixel[vertices_pixel.size()-1][0] << " , " << vertices_pixel[vertices_pixel.size()-1][1] << endl;


    vertices_gps = this->convertVerticesToLatLon( vertices_pixel, corners, start, goal, mat_width_p, mat_height_p );
	*/
    vertices_gps = importVertices();
    vertices_pixel = gps_to_pixels( corners, vertices_gps, mat_width_p, mat_height_p );
    cout << "n vertices: " << vertices_gps.size() << endl;

	vector< vector<double> > edges = RadiusConnect(vertices_pixel, vertices_gps, radius, thresholded, image_name) ;

	show_graph( img, vertices_pixel, vertices_gps, edges );
	waitKey(0);
}

vector<vector<double> > SatImg::gps_to_pixels( vector<double> corners, vector<vector<double> > &vertices, double x, double y){

	vector<vector<double> > pixel_list;
	for(size_t i=0; i<vertices.size(); i++){
		vector<double> vert = this->GPStoMap( corners, vertices[i], x, y);
		pixel_list.push_back( vert );
	}

	return pixel_list;
}

vector<vector<double> > SatImg::importVertices(){

	std::ifstream verts_infile("/home/rdml/git/map_align/hardwarePotentialDiffPath/actualDiffPath0/hardware0_verts.txt");
	double lat, lon;
	vector<vector<double> > vertices_gps;
	char comma;
	while (verts_infile >> lat >> comma >> lon){
	    vector<double> vert;
	    vert.push_back( lat );
	    vert.push_back( lon );
	    vertices_gps.push_back(vert);
	}

	return vertices_gps;

}


void SatImg::setMapParams( vector<double> corners ){

	double max_lat, min_lat, max_lon, min_lon;

	if(corners[0] < corners[2]){
		min_lat = corners[0];
		max_lat = corners[2];
	}
	else{
		min_lat = corners[2];
		max_lat = corners[0];
	}

	if(corners[1] < corners[3]){
		min_lon = corners[1];
		max_lon = corners[3];
	}
	else{
		min_lon = corners[3];
		max_lon = corners[1];
	}

	this->nw_corner = {max_lat, min_lon};
	this->ne_corner = {max_lat, max_lon};
	this->sw_corner = {min_lat,min_lon};
	this->se_corner = {min_lat, max_lon};

	this->map_height_m = ( this->haversineDistance( this->nw_corner, this->ne_corner) + this->haversineDistance( this->sw_corner, this->se_corner) ) / 2.0;
	this->map_width_m = ( this->haversineDistance( this->nw_corner, this->sw_corner) + this->haversineDistance( this->ne_corner, this->se_corner) ) / 2.0;

	cout << "map size: " << map_width_m << " , " << map_height_m << endl;
}

void SatImg::setNumVertices(){

	double aMap = this->map_height_m * this->map_width_m;
	cout << "aMap: " << aMap << endl;
	double sense_area = 314.159265/6;
	this->numVertices = round( aMap / sense_area );

}

vector< vector<double> > SatImg::convertVerticesToLatLon( vector< vector<double> > vertices, vector<double> corners, vector<double> start, vector<double> goal, double x, double y){

	vector<vector<double> > lat_lon_list;
	for(size_t i=0; i<vertices.size(); i++){
		vector<double> vert = this->mapToGPS( corners, vertices[i], x, y);
		lat_lon_list.push_back( vert );
	}

	lat_lon_list[0] = start;
	lat_lon_list[vertices.size() - 1] = goal;

	return lat_lon_list;
}

void SatImg::show_graph( cv::Mat img, vector<vector<double> > &vertices, vector<vector<double> > &vertices_gps, vector<vector<double> > &edges ){

	cout << "vertices start: " << vertices[0][0] << " , " << vertices[0][1] << endl;
	cout << "vertices goal: " << vertices[this->numVertices - 1][0] << " , " << vertices[this->numVertices - 1][1] << endl;

	cv::Point p( vertices[0][0], vertices[0][1] );
	cv::circle( img, p , 10, Scalar(0,255,0), -1);

	cv::Point p2( vertices[this->numVertices-1][0], vertices[this->numVertices - 1][1] );
	cv::circle( img, p2 ,10, Scalar(0,0,255), -1);

	cv::Mat img2 = img.clone();

	for(size_t i=0; i<edges.size(); i++){
		int v0 = edges[i][0];
		cv::Point p0( vertices[v0][0], vertices[v0][1] );
		int v1 = edges[i][1];
		cv::Point p1( vertices[v1][0], vertices[v1][1] );

		cv::circle( img, p1 ,2, Scalar(0,0,0), -1);
		cv::circle( img, p2 ,2, Scalar(0,0,0), -1);

		float dist = this->haversineDistance( vertices_gps[v1], vertices_gps[v0] );
		float mean = edges[i][2] / dist - 1;

		cv::line( img, p0, p1, Scalar(0,0,255*mean), 1);
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

vector<double> SatImg::GPStoMap( vector<double> corners, vector<double> point, double x, double y){

	vector<double> indices;
	double mx = (point[0] - corners[0])/(corners[2]-corners[0]);
	indices.push_back( round( mx * x ) );
	double my = (point[1] - corners[1])/(corners[3]-corners[1]);
	indices.push_back( round( my * y ) );

	return indices;
}

vector<double> SatImg::mapToGPS( vector<double> corners, vector<double> point, double x, double y){

	double mx = point[0]/x;
	double my = point[1]/y;

	vector<double> lat_lon;
	lat_lon.push_back( corners[0] + mx*double(corners[2] - corners[0]) );
	lat_lon.push_back( corners[1] + my*double(corners[3] - corners[1]) );

	return lat_lon;
}

double SatImg::haversineDistance( vector<double> p1, vector<double> p2){
	double R = 6378136.6; // radius of earth in meters
	double toRad = 0.01745329251;
	double lat1 = p1[1]*toRad;
	double lon1 = p1[0]*toRad;
	double lat2 = p2[1]*toRad;
	double lon2 = p2[0]*toRad;

	double d_lat = lat2-lat1;
	double d_lon = lon2-lon1;

	double a = pow(sin(d_lat/2.0),2) + cos(lat1)*cos(lat2) * pow(sin(d_lon/2.0),2);
	double c = 2*atan2( sqrt(a), sqrt(1-a) );

	double dist = R * c;

	return dist;
}

// Connect vertices within specified radius variances based on images
vector< vector<double> > SatImg::RadiusConnect(vector< vector<double> > vertices_pixel, vector< vector<double> > vertices_gps, double radius, cv::Mat img, string mapName){
    cout << "IMAGE RADIUS CONNECT" << endl;
	srand(time(NULL));
	vector< vector<double> > edges(pow(vertices_pixel.size(),2), vector<double>(4)) ;
	int k = 0 ;
	for (size_t i = 0; i < vertices_pixel.size(); i++){
		for (size_t j = 0; j < vertices_pixel.size(); j++){
			//double distance = EuclideanDistance(vertices_pixel[i], vertices_pixel[j]) ;
			double distance_meters = this->haversineDistance(vertices_gps[i], vertices_gps[j]);
			if (distance_meters <= radius && i != j){
				vector< vector < int > > pixels = Bresenham(vertices_pixel[i][0], vertices_pixel[i][1], vertices_pixel[j][0], vertices_pixel[j][1]);
				vector< int > color;
				for(size_t z = 0; z < pixels.size(); z++){
					int co = img.at<uchar>(int(pixels[z][1]), int(pixels[z][0]));
					//cout << "co: " << co << endl;
					double res = double(co)/255.0;
					color.push_back(res);
				}
				vector< double > MandV = CalcMeanVar(color); // change this to pixel values
				edges[k][0] = (double)i ;
				edges[k][1] = (double)j ;
				//cout << "MandV: " << MandV[0] << ", " << MandV[1] << endl;
				edges[k][2] = distance_meters * (1+5.0*MandV[0]);
				edges[k][3] = distance_meters * MandV[1];
				k++ ;
			}
		}
	}
	edges.resize(k) ;

	// Write edges to txt file
	stringstream eFileName ;
	eFileName << "/home/rdml/git/map_align/hardwarePaths/" << mapName << "_edges.txt" ;
	ofstream edgesFile ;
	edgesFile.open(eFileName.str().c_str()) ;

	for (int i = 0; i < int( edges.size() ); i++){
		edgesFile << edges[i][0] << "," << edges[i][1] << "," << edges[i][2] << "," << edges[i][3] << "\n" ;
	}
	edgesFile.close() ;

	/*
	// Write vertices to txt file
	stringstream vFileName ;
	vFileName << "/home/rdml/git/map_align/SatGraphPaths/" << mapName << "_verts.txt" ;
	ofstream vertsFile ;
	vertsFile.open(vFileName.str().c_str()) ;

	for (size_t i = 0; i < vertices_gps.size(); i++){
		vertsFile << vertices_pixel[i][0] << "," << vertices_pixel[i][1] << "\n" ;
	}
	vertsFile.close() ;
	*/

	// Write vertices to txt file
	stringstream vFileName ;
	vFileName << "/home/rdml/git/map_align/hardwarePaths/" << mapName << "_verts.txt" ;
	ofstream vertsFile ;
	vertsFile.open(vFileName.str().c_str()) ;

	for (size_t i = 0; i < vertices_gps.size(); i++){
		vertsFile << std::fixed << std::setprecision(12) << vertices_gps[i][0] << "," << vertices_gps[i][1] << "\n" ;
	}
	vertsFile.close() ;

	// Write to yaml file for ros
	stringstream vFileName_ros ;
	vFileName_ros << "/home/rdml/git/map_align/hardwarePaths/" << mapName << "_rosFile.yaml" ;
	ofstream rosFile ;
	rosFile.open(vFileName_ros.str().c_str()) ;

	// header info
	rosFile << "num_vertices: " << vertices_gps.size() << "\n";
	rosFile << "num_edges: " << edges.size() << "\n";
	rosFile << "edge_cost_rate: 3.0\n";

	// vertices
	for (size_t i = 0; i < vertices_gps.size(); i++){
		char rosPrint[100] ;
		sprintf(rosPrint,"vertex%d: [%0.12f,%0.12f]", i, vertices_gps[i][0], vertices_gps[i][1]) ;
		rosFile << rosPrint << endl; //std::fixed << std::setprecision(12) << "vertex" << i << ":[" << vertices_gps[i][0] << "," << vertices_gps[i][1] << "]\n" ;
	}
	// edges
	for (size_t i = 0; i < edges.size() ; i++){
		char rosPrint[100] ;
		sprintf(rosPrint,"edge%d: [%d,%d,%d,%d]", i, int(edges[i][0]), int(edges[i][1]), int(edges[i][2]), int(edges[i][3]) ) ;
		rosFile << rosPrint << endl;//rosFile << "edge" << i << ": [" << int(edges[i][0]) << "," << int(edges[i][1]) << "," << edges[i][2] << "," << edges[i][3] << "]\n" ;
	}

	rosFile.close() ;

	return edges ;
}

double SatImg::EuclideanDistance(vector<double> v1, vector<double> v2){
	double diff_x = v1[0] - v2[0];
	double diff_y = v1[1] - v2[1] ;
	double diff = sqrt(pow(diff_x,2) + pow(diff_y,2)) ;

	return diff;
}


