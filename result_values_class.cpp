#include <deque>
#include <Math.h>
#include "opencv2/opencv.hpp"
#include "result_values_class.h"
#include "lane_detect_processor.h"

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
	double sum{ x0 + y0 + x1 + y2 + x2 + y2 + x3 + y3 };
	
	return sum;
}

ResultValues::ResultValues()
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
	bestpassed_ = false;
	previousscore_ = score_;
	polygondevqueue_.clear();
	
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

void ResultValues::SetPreviousScore()
{
	hasimproved_ = false;
	score_ = previousscore_;
	
	return;
}

void ResultValues::Update(uint32_t totalframes)
{
	polygondev_ = Average(polygondevqueue_);
	score_= 20000.0 + (20000.0 * detectedframes_)/totalframes - 1.0 * polygondev_;
	if ( score_ >= previousscore_ ) {
		improved_ = hasimproved_ = true;
	} else {
		improved_ = false;
	}
	if ( hasimproved_ && !improved_ ) bestpassed_ = true;
	
	return;
}
