/*
 * SatImg.cpp
 *
 *  Created on: Apr 3, 2017
 *      Author: rdml
 */

#include "SatImg.h"

SatImg::SatImg(string image_dir, string image_name, vector<double> corners, vector<double> start, vector<double> goal){ // PRM-style graph connection{

    double radius_m = 20.0; //sqrt(1.9098593171*this->map_width_m*this->map_height_m*(log((double)this->numVertices)/(double)this->numVertices)) ;
    cout << "Connecting with radius " << radius_m << " meters" << endl;


	cv::Mat img = cv::imread(image_dir + image_name + ".png", CV_LOAD_IMAGE_COLOR);
	cv::Mat img_raw;
	img.copyTo(img_raw);
	//cv::Mat img = cv::imread("/home/rdml/git/map_align/short_hardware/easy1.PNG", CV_LOAD_IMAGE_COLOR);
	this->mat_width_p = img.size().width;
	this->mat_height_p = img.size().height;

	this->setMapParams( corners );
	this->setNumVertices( );

	cout << "map size pixels: " << img.cols << ", " << img.rows << endl;
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
	cv::inRange(blurred_img, cv::Scalar(0, 0, 0), cv::Scalar(70, 190, 70), thresholded);
	//cv::inRange(blurred_img, cv::Scalar(0, 0, 0), cv::Scalar(50, 180, 50), thresholded);
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

    cout << "Generating Random Vertices in " << mat_width_p << " by " << mat_height_p << endl;
    vertices_pixel = this->makeVertices(mat_width_p, mat_height_p,this->numVertices, image_name);
    vertices_pixel[0] = this->GPStoMap( corners, start, mat_width_p, mat_height_p);
    vertices_pixel[vertices_pixel.size()-1] = this->GPStoMap( corners, goal, mat_width_p, mat_height_p);

    cout << "vert: " << vertices_pixel[0][0] << " , " << vertices_pixel[0][1] << endl;
    cout << "vert: " << vertices_pixel[vertices_pixel.size()-1][0] << " , " << vertices_pixel[vertices_pixel.size()-1][1] << endl;


    vertices_gps = this->convertVerticesToLatLon( vertices_pixel, corners, start, goal, mat_width_p, mat_height_p );

    //vertices_gps = importVertices();
    //vertices_pixel = gps_to_pixels( corners, vertices_gps, mat_width_p, mat_height_p );
    cout << "n vertices_gps: " << vertices_gps.size() << endl;
    cout << "n vertices_pixels: " << vertices_pixel.size() << endl;

	vector< vector<double> > edges = RadiusConnect(vertices_pixel, vertices_gps, radius_m, thresholded, image_name, corners) ;

	show_graph( img, vertices_pixel, vertices_gps, edges );

	cout << "getting task set" << endl;
	this->get_task_set(thresholded, img_raw, vertices_pixel, image_name);

	waitKey(0);
}

void SatImg::get_task_set(cv::Mat map, cv::Mat img, std::vector<std::vector<double> > &vertices_pixels, string mapName){

	this->pixel_height_m = this->map_height_m / this->mat_height_p;
	this->pixel_width_m = this->map_width_m / this->mat_width_p;

	std::vector<Point> task_locs;

	cv::Mat reward_map = Mat::ones(map.size(), CV_8UC1)*255;
	double radius_m = 10;
	cout << "min pixel_dim in meters = " << std::min(this->pixel_height_m, this->pixel_width_m) << endl;
	int radius_p = radius_m / std::min(this->pixel_height_m, this->pixel_width_m);
	double tree_thresh = 0.1;

	cv::Scalar temp = 0.01*cv::sum(reward_map);
	double thresh = temp[0];
	double reward = double(INFINITY);
	while( reward > thresh){
		double max_r = -1;
		int max_i = -1;

		for(int i=0; i<this->numVertices; i++){
			// get rect with circle
			//printf("map_size p (%f/%f) and vertices (%f/%f)\n", this->mat_width_p, this->mat_height_p, vertices_pixels[i][0], vertices_pixels[i][1]);


			Point2i cen(vertices_pixels[i][0], vertices_pixels[i][1]);

			int left = std::max(0,cen.x-radius_p);
			int right = std::min(cen.x+radius_p, int(this->mat_width_p)-1);
			int top = std::max(0,cen.y-radius_p);
			int bottom = std::min(cen.y+radius_p, int(this->mat_height_p)-1);

			cv::Rect r(Point(left,top),Point(right,bottom));

			//printf("l/r(%i/%i) and t/b (%i/%i)\n", left,right,top,bottom);
			//cv::waitKey(100);

			// obtain roi
			Mat roi(reward_map, r);

			// make a mask over roi
			Mat mask(roi.size(), roi.type(), Scalar::all(0));
			// white area in center
			cv::circle(mask, Point(radius_p, radius_p), radius_p, Scalar::all(255), -1);

			// combine masks
			Mat masked_roi = roi & mask;
			//cv::namedWindow("masked roi", WINDOW_NORMAL);
			//cv::imshow("masked roi", masked_roi);

			// get the reward in the masked area
			Scalar temp = cv::sum(masked_roi);
			double reward_cropped = temp[0];

			/*if(reward_cropped >= pow(double(radius_p),2)*3.1){
				max_i = i;
				max_r = reward_cropped;
				break;
			}*/

			// is it the best?
			if(reward_cropped > max_r){
				max_r = reward_cropped;
				max_i = i;
			}
		}
		task_locs.push_back(Point(vertices_pixels[max_i][0],vertices_pixels[max_i][1]));

		//printf("map_size p (%f/%f) and vertices (%f/%f)\n", this->mat_width_p, this->mat_height_p, vertices_pixels[max_i][0], vertices_pixels[max_i][1]);
		//printf("max_r: %f \n", max_r);

		cv::circle(reward_map, Point(vertices_pixels[max_i][0],vertices_pixels[max_i][1]) , radius_p, Scalar::all(0), -1);

		//cv::namedWindow("Reward Map", WINDOW_NORMAL);
		//cv::imshow("Reward Map", reward_map);
		//cv::waitKey(100);

		Scalar temp = cv::sum(reward_map);
		reward = temp[0];
	}

	std::vector<int> uav_tasks;
	std::vector<int> human_tasks;

	Mat uav_map, human_map;
	img.copyTo(uav_map);
	img.copyTo(human_map);

	for(size_t i=0; i<task_locs.size(); i++){
		Point2i cen = task_locs[i];

		int left = std::max(0,cen.x-radius_p);
		int right = std::min(cen.x+radius_p, int(this->mat_width_p)-1);
		int top = std::max(0,cen.y-radius_p);
		int bottom = std::min(cen.y+radius_p, int(this->mat_height_p)-1);

		cv::Rect r(Point(left,top),Point(right,bottom));

		//printf("l/r(%i/%i) and t/b (%i/%i)\n", left,right,top,bottom);
		//cv::waitKey(100);

		// obtain roi
		Mat roi(map, r);

		// make a mask over roi
		Mat mask(roi.size(), roi.type(), Scalar::all(0));
		// white area in center
		cv::circle(mask, Point(radius_p, radius_p), radius_p, Scalar::all(255), -1);

		//Mat temp_mat;
		//map.copyTo(temp_mat);
		//cv::circle(temp_mat, task_locs[i], radius_p, Scalar::all(127), 2);
		//cv::namedWindow("temp", WINDOW_NORMAL);
		//cv::imshow("temp", temp_mat);
		//cv::waitKey(10);

		// combine masks
		Mat masked_roi = roi & mask;
		//cv::namedWindow("masked roi", WINDOW_NORMAL);
		//cv::imshow("masked roi", masked_roi);
		//cv::waitKey(10);

		// get the reward in the masked area
		Scalar temp = cv::sum(masked_roi);
		double tree_count_in_roi = temp[0];
		double max_count = pow(double(radius_p),2)*3.1*255.0;
		double mean_tree = tree_count_in_roi / max_count;
		if(mean_tree > tree_thresh){
			human_tasks.push_back(i);
			cv::circle(human_map, task_locs[i] , radius_p, Scalar(0,255,0), -1);
			//cout << "human_task" << endl;
			//cv::namedWindow("human map 1", WINDOW_NORMAL);
			//cv::imshow("human map 1", human_map);
			//cv::waitKey(10);
		}
		else{
			uav_tasks.push_back(i);
			cv::circle(uav_map, task_locs[i] , radius_p, Scalar(0,0,255), -1);
			//cout << "uav_task" << endl;
			//cv::namedWindow("uav map 1", WINDOW_NORMAL);
			//cv::imshow("uav map 1", uav_map);
			//cv::waitKey(10);
		}

		//cout << "mean tree: " <<  mean_tree << " = " << tree_count_in_roi << " / " << max_count << endl;
		//waitKey(10);

	}
	cv::namedWindow("uav map 1", WINDOW_NORMAL);
	cv::imshow("uav map 1", uav_map);
	cv::waitKey(10);

	cv::namedWindow("human map 1", WINDOW_NORMAL);
	cv::imshow("human map 1", human_map);
	cv::waitKey(10);

	Mat team_map;
	cv::addWeighted(human_map, 0.5, uav_map, 0.5, 0, team_map);

	cv::namedWindow("team map 1", WINDOW_NORMAL);
	cv::imshow("team map 1", team_map);
	cv::waitKey(10);

	cv::addWeighted(team_map, 0.5, img, 0.5, 0, team_map);

	cv::namedWindow("team map", WINDOW_NORMAL);
	cv::imshow("team map", team_map);
	cv::waitKey(10);

	// Write human tasks to txt file
	string eFileName = "/home/rdml/git/map_align/dist_planner_files/" + mapName + "_human_tasks.xml";
	FileStorage fh(eFileName, FileStorage::WRITE);
	fh << "n_tasks" << int(human_tasks.size());
	for (int i = 0; i < int(human_tasks.size()); i++){
		char nm[100];
		sprintf(nm, "task%i",i);
		fh << nm << human_tasks[i];
	}
	fh.release();

	// Write uav tasks to txt file
	eFileName = "/home/rdml/git/map_align/dist_planner_files/" + mapName + "_uav_tasks.xml";
	FileStorage fu(eFileName, FileStorage::WRITE);
	fu << "n_tasks" << int(uav_tasks.size());
	for (int i = 0; i < int( uav_tasks.size() ); i++){
		char nm[100];
		sprintf(nm, "task%i",i);
		fu << nm << uav_tasks[i];
	}
	fu.release();

	cout << "human tasks: " << human_tasks.size() << endl;
	cout << "uav_tasks: " << uav_tasks.size() << endl;
	cout << "total_tasks: " << human_tasks.size() + uav_tasks.size() << endl;
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
	//cv::circle( img, p , 10, Scalar(0,255,0), -1);

	cv::Point p2( vertices[this->numVertices-1][0], vertices[this->numVertices - 1][1] );
	//cv::circle( img, p2 ,10, Scalar(0,0,255), -1);

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
vector< vector<double> > SatImg::RadiusConnect(vector< vector<double> > vertices_pixel, vector< vector<double> > vertices_gps, double radius, cv::Mat img, string mapName, vector<double> corners){
    cout << "IMAGE RADIUS CONNECT" << endl;
	srand(time(NULL));
	vector< vector<double> > edges(pow(vertices_pixel.size(),2), vector<double>(5)) ;
	int k = 0 ;
	for (size_t i = 0; i < vertices_pixel.size(); i++){
		for (size_t j = 0; j < vertices_pixel.size(); j++){
			if(i == j){
				continue;
			}
			//cout << "j: " << j << endl;
			//double distance = EuclideanDistance(vertices_pixel[i], vertices_pixel[j]) ;
			double distance_meters = this->haversineDistance(vertices_gps[i], vertices_gps[j]);
			//cout << "dist: " << distance_meters << endl;
			if (distance_meters <= radius && i != j){
				//printf("vertices_pixel[%i][0],[1]: %d, %d \n", i, vertices_pixel[i][0], vertices_pixel[i][1]);
				//printf("vertices_pixel[%i][0],[1]: %d, %d \n", j, vertices_pixel[j][0], vertices_pixel[j][1]);

				vector< vector < int > > pixels = Bresenham(vertices_pixel[i][0], vertices_pixel[i][1], vertices_pixel[j][0], vertices_pixel[j][1]);
				vector< int > color;
				for(size_t z = 0; z < pixels.size(); z++){
					int co = img.at<uchar>(int(pixels[z][1]), int(pixels[z][0]));
					double res = double(co)/255.0;
					color.push_back(res);
				}
				vector< double > MandV = CalcMeanVar(color); // change this to pixel values
				edges[k][0] = (double)i ;
				edges[k][1] = (double)j ;
				//cout << "MandV: " << MandV[0] << ", " << MandV[1] << endl;
				edges[k][2] = distance_meters * (1+5.0*MandV[0]);
				edges[k][3] = distance_meters * MandV[1];
				edges[k][4] = distance_meters;
				k++ ;
			}
		}
	}
	edges.resize(k) ;
	std::cout << "Edges Complete" << std::endl;

	// Write edges to txt file
	string eFileName = "/home/rdml/git/map_align/dist_planner_files/" + mapName + "_edges.xml";
	FileStorage fe(eFileName, FileStorage::WRITE);
	fe << "n_edges" << int(edges.size());
	for (int i = 0; i < int( edges.size() ); i++){
		char nm[100];
		sprintf(nm, "edge%i",i);
		fe << nm << edges[i];
	}
	fe.release();

	// Write vertices to txt file
	eFileName = "/home/rdml/git/map_align/dist_planner_files/" + mapName + "_vertices.xml";
	FileStorage fv(eFileName, FileStorage::WRITE);
	fv << "n_vertices" << int(vertices_gps.size());
	fv << "corners" << corners;
	for (int i = 0; i < int(vertices_gps.size()); i++){
		char nm[100];
		sprintf(nm, "vertex%i_gps",i);
		fv << nm << vertices_gps[i];
	}
	fv.release();

	/*
	// Write to yaml file for ros
	stringstream vFileName_ros ;
	vFileName_ros << "/home/rdml/git/map_align/dist_planner_files/" << mapName << "_rosFile.yaml" ;
	ofstream rosFile ;
	rosFile.open(vFileName_ros.str().c_str()) ;

	// header info
	rosFile << "num_vertices: " << vertices_gps.size() << "\n";
	rosFile << "num_edges: " << edges.size() << "\n";
	rosFile << "edge_cost_rate: 3.0\n";
	char rosPrint[100] ;
	sprintf(rosPrint,"corners: [%0.12f,%0.12f,%0.12f,%0.12f]", corners[0], corners[1], corners[2], corners[3]) ;
	rosFile << rosPrint << endl;

	// vertices
	for (size_t i = 0; i < vertices_gps.size(); i++){
		char rosPrint[100] ;
		sprintf(rosPrint,"vertex%d: [%0.12f,%0.12f]", i, vertices_gps[i][0], vertices_gps[i][1]) ;
		rosFile << rosPrint << endl; //std::fixed << std::setprecision(12) << "vertex" << i << ":[" << vertices_gps[i][0] << "," << vertices_gps[i][1] << "]\n" ;
	}
	// edges
	for (size_t i = 0; i < edges.size() ; i++){
		char rosPrint[100] ;
		sprintf(rosPrint,"edge%d: [%d,%d,%d,%d,%d]", i, int(edges[i][0]), int(edges[i][1]), int(edges[i][2]), int(edges[i][3]), int(edges[i][4]) ) ;
		rosFile << rosPrint << endl;//rosFile << "edge" << i << ": [" << int(edges[i][0]) << "," << int(edges[i][1]) << "," << edges[i][2] << "," << edges[i][3] << "]\n" ;
	}

	rosFile.close();
	*/

	return edges ;
}

double SatImg::EuclideanDistance(vector<double> v1, vector<double> v2){
	double diff_x = v1[0] - v2[0];
	double diff_y = v1[1] - v2[1] ;
	double diff = sqrt(pow(diff_x,2) + pow(diff_y,2)) ;

	return diff;
}


