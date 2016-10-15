#include <deque>
#include <math.h>
#include "opencv2/opencv.hpp"
#include "result_values_class.h"
#include "lane_detect_processor.h"
#include "lane_constant_class.h"

double Average( std::deque<double> &values )
{
	double value{0.0};
	if ( values.size() < 1 ) return value;
	for ( double d : values ) {
		value += d;
	}
	value /= values.size();
	return value;
}

Polygon AveragePolygons ( std::deque<Polygon> &polygons)
{
	double x0{0.0}, y0{0.0}, x1{0.0}, y1{0.0}, x2{0.0}, y2{0.0}, x3{0.0}, y3{0.0};
	if ( polygons.size() < 1 ) {
		return Polygon{cv::Point(x0,y0), cv::Point(x1,y1), cv::Point(x2,y2),
			cv::Point(x3,y3)};
	}
	for (Polygon &p : polygons) {
		x0 += p[0].x;
		y0 += p[0].y;
		x1 += p[1].x;
		y1 += p[1].y;
		x2 += p[2].x;
		y2 += p[2].y;
		x3 += p[3].x;
		y3 += p[3].y;
	}
	x0 /= polygons.size();
	y0 /= polygons.size();
	x1 /= polygons.size();
	y1 /= polygons.size();
	x2 /= polygons.size();
	y2 /= polygons.size();
	x3 /= polygons.size();
	y3 /= polygons.size();
	
	return Polygon{cv::Point(x0,y0), cv::Point(x1,y1), cv::Point(x2,y2),
		cv::Point(x3,y3)};
}

double StandardDeviation( std::deque<Polygon> &polygons )
{
	if ( polygons.size() < 1 ) return 0.0;
	Polygon mean{AveragePolygons(polygons)};
	double x0{0.0}, y0{0.0}, x1{0.0}, y1{0.0}, x2{0.0}, y2{0.0}, x3{0.0}, y3{0.0};
	for (Polygon &p : polygons) {
		x0 += (p[0].x - mean[0].x) * (p[0].x - mean[0].x);
		y0 += (p[0].y - mean[0].y) * (p[0].y - mean[0].y);
		x1 += (p[1].x - mean[1].x) * (p[1].x - mean[1].x);
		y1 += (p[1].y - mean[1].y) * (p[1].y - mean[1].y);
		x2 += (p[2].x - mean[2].x) * (p[2].x - mean[2].x);
		y2 += (p[2].y - mean[2].y) * (p[2].y - mean[2].y);
		x3 += (p[3].x - mean[3].x) * (p[3].x - mean[3].x);
		y3 += (p[3].y - mean[3].y) * (p[3].y - mean[3].y);
	}
	x0 /= polygons.size();
	y0 /= polygons.size();
	x1 /= polygons.size();
	y1 /= polygons.size();
	x2 /= polygons.size();
	y2 /= polygons.size();
	x3 /= polygons.size();
	y3 /= polygons.size();
	//X and lowest points more critical
	double sum{ x0 + x1 + 0.75*(x2 + x3) }; 
	//double sum{ x0 + y0 + x1 + y1 + x2 + y2 + x3 + y3 };
	
	return sum;
}

ResultValues::ResultValues( uint32_t totalframes ):
							totalframes_{totalframes},
							detectedframes_{0},
							previousscore_{0.0},
							score_{0.0},
							polygondev_{0.0},
							lanedetectmultiplier_{0.0},
							firstpass_{true}
{
	
}

void ResultValues::NewPattern()
{
	polygonqueue_.clear();
	
	return;
}

void ResultValues::NewIteration()
{
	NewPattern();
	detectedframes_ = 0;
	polygondevqueue_.clear();
	return;
}

void ResultValues::NewVariable()
{
	NewIteration();
	
	return;
}

void ResultValues::Push(Polygon polygon)
{
	int valuestokeep{30};
	if ( polygon[0] != cv::Point(0,0) ) {
		detectedframes_++;
		polygonqueue_.push_back(polygon);
		if ( polygonqueue_.size() > valuestokeep ) polygonqueue_.pop_front();
		polygondevqueue_.push_back(StandardDeviation(polygonqueue_));
	}
	
	return;
}

void ResultValues::Update(LaneConstant& laneconstant)
{
	//Check for first iteration for this variable
	if ( laneconstant.firstpass_ ) {
		laneconstant.bestscore_ = score_;
		laneconstant.firstpass_ = false;
	}
	
	//Score
	polygondev_ = Average(polygondevqueue_);
	if ( firstpass_ ) { 	//Adjust detected frame multiplier to bring inital score to 0!
		lanedetectmultiplier_ = polygondev_ * (static_cast<double>(totalframes_)
			/ static_cast<double>(detectedframes_));
		firstpass_ = false;
	}
	score_= (lanedetectmultiplier_ * detectedframes_)/totalframes_ - polygondev_;
	outputscore_ = score_;


	//Figure it out
	if ( laneconstant.hitlimit_ ) {
		if ( (laneconstant.reversedcount_ == 0) && (score_ == previousscore_ )) {
			laneconstant.Reverse();
			score_ = previousscore_;
			laneconstant.value_ = laneconstant.bestvalue_;
			laneconstant.hitlimit_ = false;
		} else if ( score_ > previousscore_ ) {
			laneconstant.finished_ = true;
		} else {
			laneconstant.value_ = laneconstant.bestvalue_;
			score_ = laneconstant.bestscore_ ;
			laneconstant.finished_ = true;	
		}
	} else if ( score_ > previousscore_  ) {
		if ( score_ > laneconstant.bestscore_ ) {
			laneconstant.bestscore_ = score_;
			laneconstant.bestvalue_ = laneconstant.value_;
		}
	} else if ( score_ < previousscore_ ) {
		if ( laneconstant.reversedcount_ > 0 ) {
			score_ = laneconstant.bestscore_ ;
			laneconstant.value_ = laneconstant.bestvalue_;
			laneconstant.finished_ = true;
		} else {
			laneconstant.Reverse();
			score_ = previousscore_;
		}
	}
	previousscore_ = score_;
	if ( laneconstant.finished_ ) return;
	laneconstant.Modify();
	
	return;
}
