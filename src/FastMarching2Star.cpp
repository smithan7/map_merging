/*
 * FastMarching2Star.cpp
 *
 *  Created on: Mar 29, 2017
 *      Author: rdml
 */

#include "FastMarching2Star.h"

FastMarching2Star::FastMarching2Star( Mat &map, vector<Point2f> &walls ) {

	start = Point2f(80,100);
	goal = Point2f(450, 300);

	maxSpeed = 1.0;

	this->speedMap = Mat::ones(map.size(), CV_32FC1)*maxSpeed;
	this->map = Mat::zeros(map.size(), CV_8UC1);
	this->time = Mat::zeros(this->speedMap.size(), CV_32FC1);

	cell2Meter = 0.5;
	crashRadius = 1.0;
	inflationRadius = 10.0;
	fm_tol = 1.0;

	for(size_t i=0; i<walls.size(); i++){
		this->speedMap.at<float>(walls[i]) = 0.0;
		this->map.at<uchar>(walls[i]) = 255;
	}


	//fm2.displayMap();
	inflateWalls_to_crashRadius();
	//fm2.displayMap();
	inflateWalls_to_inflationRadius();
	//fm2.displayMap();
	wavePropagation();
	vector<Point2f> path = findPath();
	displayPath_over_map( path );
	displayPath_over_time( path );
	displayPath_over_speedMap( path );


}

vector<Point2f> FastMarching2Star::findPath(){

	Point2f c;
	c.x = this->goal.x;
	c.y = this->goal.y;

	vector<Point2f> path;

	float dist = sqrt( pow(start.x - c.x, 2) + pow(start.y-c.y,2) );
	float prior_dist = dist + 1;
	bool flag = true;
	float mod_scale = 0.2;
	while( flag || dist < prior_dist ){
		if( dist < 10* this->maxSpeed ){
			flag = false;
		}
		path.push_back( c );
		Point2f g = getGradients( c );

		float mod = mod_scale * sqrt(pow(g.x,2) + pow(g.y,2));
		float alpha = atan2(g.y, g.x);
		c.x = c.x - mod*cos( alpha );
		c.y = c.y - mod*sin( alpha );
		prior_dist = dist;
		dist = sqrt( pow(start.x - c.x, 2) + pow(start.y-c.y,2) );
	}
	return path;
}

Point2f FastMarching2Star::getGradients( Point2f p ){

	float delta[2] = {-1, 1};

	float nx[2];
	float ny[2];

	nx[0] = this->time.at<float>(Point(p.x-1, p.y) );
	nx[1] = this->time.at<float>(Point(p.x+1, p.y) );

	ny[0] = this->time.at<float>(Point(p.x, p.y-1) );
	ny[1] = this->time.at<float>(Point(p.x, p.y+1) );

	if(nx[0]==0.0 || nx[1]==0.0 || ny[0]==0.0 || ny[1]==0.0 ){ // should only happen the first iteration
		double min, max;
		minMaxLoc( this->time, &min, &max);
		for(int i=0; i<2; i++){
			if( nx[i] == 0.0 ){
				nx[i] = max;
			}
			if( ny[i] == 0.0 ){
				ny[i] = max;
			}
		}
	}

	Point2f g(0.0, 0.0);

	for(int i=0; i<2; i++){
		g.x += delta[i]*nx[i];
		g.y += delta[i]*ny[i];
	}

	return g;
}

bool FastMarching2Star::wavePropagation(){
	if( abs(start.x - goal.x) + abs(start.y - goal.y) < fm_tol ){
		return true;
	}

	Mat cSet = Mat::zeros(this->speedMap.size(), CV_16S); // 1 means in closed set, 0 means not
	Mat oSet = Mat::zeros(this->speedMap.size(), CV_16S); // 1 means in open set, 0 means not

	Mat gScore = Mat::zeros(this->speedMap.size(), CV_32F); // known cost from initial node to n
	Mat fScore = Mat::ones(this->speedMap.size(), CV_32F)*INFINITY; // known cost from initial node to n

	vector<Point> oVec;
	oVec.push_back(this->start);
	oSet.at<short>(this->start) = 1; // starting node has score 0
	gScore.at<float>(this->start)  = 0; // starting node in open set
	fScore.at<float>(this->start) = sqrt(pow(this->start.x-goal.x,2) + pow(this->start.y-goal.y,2));
	fScore.at<float>(goal) = 0;

	// for nbrs
	int nx[4] = {-1,1,0,0};
	int ny[4] = {0,0,-1,1};

	while(oVec.size() > 0){
		/////////////////// this finds node with lowest fScore and makes current
		float min = INFINITY;
		int mindex = -1;

		for(size_t i=0; i<oVec.size(); i++){
			if(fScore.at<float>(oVec[i]) < min){
				min = fScore.at<float>(oVec[i]);
				mindex = i;
			}
		}

		if( mindex == -1){
			this->time = gScore.clone();
			return false;
		}

		Point cLoc = oVec[mindex];
		oVec.erase(oVec.begin() + mindex);
		oSet.at<short>(cLoc) = 0;
		cSet.at<short>(cLoc) = 1;

		if( cLoc.x == goal.x && cLoc.y == goal.y ){ // if the current node equals goal, construct path
			this->time = gScore.clone();
			return true;
		}

		for(int ni = 0; ni<4; ni++){
			Point nbr;
			nbr.x += cLoc.x + nx[ni];
			nbr.y += cLoc.y + ny[ni];
			if( nbr.x >= 0 && nbr.x < this->speedMap.cols && nbr.y >= 0 && nbr.y < this->speedMap.rows ){ // is nbr on mat?
				if(cSet.at<short>(nbr) == 1){ // has it already been eval? in cSet
					continue;
				}
				float dist = sqrt(pow(cLoc.x-nbr.x,2) + pow(cLoc.y-nbr.y,2));
				float avg_vel = (this->speedMap.at<float>(cLoc) + this->speedMap.at<float>(nbr))/2;
				float ngScore = gScore.at<float>(cLoc) + dist / avg_vel;
				if(oSet.at<short>(nbr) == 0){
					oSet.at<short>(nbr) = 1;  // add nbr to open set
					oVec.push_back(nbr);
				}
				else if(ngScore >= gScore.at<float>(nbr) ){ // is temp gscore worse than stored g score of nbr
					continue;
				}

				gScore.at<float>(nbr) = ngScore;
				if( this->speedMap.at<float>(nbr) > 0.0 ){
					dist = sqrt(pow(nbr.x-goal.x,2) + pow(nbr.y-goal.y,2));
					fScore.at<float>(nbr) = gScore.at<float>(nbr) + dist / this->maxSpeed;
				}
				else{
					fScore.at<float>(nbr)= INFINITY;
				}
			}
		}
	}

	this->time = gScore.clone();
	return false;
}

void FastMarching2Star::displayMap(){

	Mat display = this->map.clone();


	circle( display, start, 10, Scalar(127), -1);
	circle( display, goal, 10, Scalar(127), -1);

	namedWindow("fm2 Map", WINDOW_NORMAL);
	imshow("fm2 Map", display);
	waitKey(0);
}

void FastMarching2Star::displaySpeedMap(){

	Mat display = Mat::zeros(this->speedMap.size(), CV_8UC1);
	for(int i=1; i<this->speedMap.cols-1; i++){
		for(int j=1; j<this->speedMap.rows-1; j++){
			Point p(i,j);
			float val = (this->maxSpeed - this->speedMap.at<float>(p)) / (this->maxSpeed);
			int shade = 255 - round( val*255 );

			display.at<uchar>(p) = shade;
		}
	}

	circle( display, start, 10, Scalar(127), -1);
	circle( display, goal, 10, Scalar(127), -1);

	namedWindow("fm2 speedMap", WINDOW_NORMAL);
	imshow("fm2 speedMap", display);
	waitKey(0);
}

void FastMarching2Star::displayTime(){

	double min, max;
	minMaxLoc( this->time, &min, &max);

	Mat display = Mat::zeros(this->speedMap.size(), CV_8UC1);
	for(int i=1; i<this->speedMap.cols-1; i++){
		for(int j=1; j<this->speedMap.rows-1; j++){
			Point p(i,j);
			float val = (max - this->time.at<float>(p)) / (max);
			int shade = 255 - round( val*255 );

			display.at<uchar>(p) = shade;
		}
	}

	circle( display, start, 10, Scalar(127), -1);
	circle( display, goal, 10, Scalar(127), -1);

	namedWindow("fm2 time", WINDOW_NORMAL);
	imshow("fm2 time", display);
	waitKey(0);
}

void FastMarching2Star::displayPath_over_time( vector<Point2f> &path ){

	double min, max;
	minMaxLoc( this->time, &min, &max);

	Mat display = Mat::zeros(this->speedMap.size(), CV_8UC3);


	for(int i=1; i<this->speedMap.cols-1; i++){
		for(int j=1; j<this->speedMap.rows-1; j++){
			Point p(i,j);
			float val = (max - this->time.at<float>(p)) / (max);
			int shade = 255 - round( val*255 );

			display.at<Vec3b>(p) = Vec3b(shade, shade, shade);
		}
	}

	for(size_t i=0; i<path.size(); i++){
		circle( display, path[i], 1, Scalar(0,0,255), -1);
	}

	circle( display, start, 5, Scalar(0,0,255), -1);
	circle( display, goal, 5, Scalar(0,0,255), -1);

	namedWindow("fm2 path", WINDOW_NORMAL);
	imshow("fm2 path", display);
	waitKey(0);
}

void FastMarching2Star::displayPath_over_map( vector<Point2f> &path ){

	Mat display = Mat::zeros(this->map.size(), CV_8UC3);
	for(int i=0; i<this->map.cols; i++){
		for(int j=0; j<this->map.rows; j++){
			Point p(i,j);
			if(this->map.at<uchar>(p) == 255){
				display.at<Vec3b>(p) = Vec3b(0,0,0);
			}
			else{
				display.at<Vec3b>(p) = Vec3b(255,255,255);
			}
		}
	}

	for(size_t i=0; i<path.size(); i++){
		circle( display, path[i], 1, Scalar(0,0,255), -1);
	}

	circle( display, start, 5, Scalar(0,0,255), -1);
	circle( display, goal, 5, Scalar(0,0,255), -1);

	namedWindow("fm2 speedMap", WINDOW_NORMAL);
	imshow("fm2 speedMap", display);
	waitKey(0);
}

void FastMarching2Star::displayPath_over_speedMap( vector<Point2f> &path ){

	Mat display = Mat::zeros(this->speedMap.size(), CV_8UC3);
	for(int i=1; i<this->speedMap.cols-1; i++){
		for(int j=1; j<this->speedMap.rows-1; j++){
			Point p(i,j);
			float val = (this->maxSpeed - this->speedMap.at<float>(p)) / (this->maxSpeed);
			int shade = 255 - round( val*255 );

			display.at<Vec3b>(p) = Vec3b(shade,shade,shade);
		}
	}

	for(size_t i=0; i<path.size(); i++){
		circle( display, path[i], 1, Scalar(0,0,255), -1);
	}

	circle( display, start, 5, Scalar(0,0,255), -1);
	circle( display, goal, 5, Scalar(0,0,255), -1);

	namedWindow("fm2 speedMap", WINDOW_NORMAL);
	imshow("fm2 speedMap", display);
	waitKey(0);
}


void FastMarching2Star::inflateWalls_to_crashRadius(){
	// this leaves just a value of 0.0

	float nx[] = {-1.0, 1.0, 0.0, 0.0};
	float ny[] = {0.0, 0.0, -1.0, 1.0};

	for(float s=0; s<this->crashRadius; s+=this->cell2Meter){
		Mat toInflate = Mat::zeros(this->speedMap.size(), CV_8UC1);
		for(int i=1; i<this->speedMap.cols-1; i++){
			for(int j=1; j<this->speedMap.rows-1; j++){
				Point p(i,j);
				if( this->speedMap.at<float>(p) == 0.0){
					for(int n=0; n<4; n++){
						Point pp;
						pp.x = p.x + nx[n];
						pp.y = p.y + ny[n];
						toInflate.at<uchar>(pp) = 1;
					}
				}
			}
		}
		for(int i=1; i<this->speedMap.cols-1; i++){
			for(int j=1; j<this->speedMap.rows-1; j++){
				Point p(i,j);
				if( toInflate.at<uchar>(p) == 1){
					this->speedMap.at<float>(p) = 0.0;
				}
			}
		}
	}
}

void FastMarching2Star::inflateWalls_to_inflationRadius(){
	// this decrements out to a set radius from 0.0 where it drops to max speed

	float nx[] = {-1.0, 1.0, 0.0, 0.0};
	float ny[] = {0.0, 0.0, -1.0, 1.0};


	float inflateVal = 0.0;
	float inflateVal_prior = 0.0;
	float iters = this->inflationRadius / this->cell2Meter;
	cout << "iters: " << iters << endl;
	float inflateIncrement = this->maxSpeed / iters;



	while( inflateVal < this->maxSpeed ){
		inflateVal += inflateIncrement;
		Mat toInflate = Mat::zeros(this->speedMap.size(), CV_8UC1);
		for(int i=1; i<this->speedMap.cols-1; i++){
			for(int j=1; j<this->speedMap.rows-1; j++){
				Point p(i,j);
				if( this->speedMap.at<float>(p) == inflateVal_prior){
					for(int n=0; n<4; n++){
						Point pp;
						pp.x = p.x + nx[n];
						pp.y = p.y + ny[n];
						toInflate.at<uchar>(pp) = 1;
					}
				}
			}
		}
		for(int i=1; i<this->speedMap.cols-1; i++){
			for(int j=1; j<this->speedMap.rows-1; j++){
				Point p(i,j);
				if( toInflate.at<uchar>(p) == 1 && this->speedMap.at<float>(p) == this->maxSpeed ){
					this->speedMap.at<float>(p) = inflateVal;
				}
			}
		}
		inflateVal_prior = inflateVal;
	}
}

FastMarching2Star::~FastMarching2Star() {
	// TODO Auto-generated destructor stub
}

