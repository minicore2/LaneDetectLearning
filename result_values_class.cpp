#include <deque>
#include <math.h>
#include "opencv2/opencv.hpp"
#include "result_values_class.h"
#include "lane_detect_processor.h"
#include "lane_constant_class.h"

#define POLYGONSCALING 1.0f

double Average( std::deque<float> &values )
{
	double value{0.0};
	if ( values.size() < 1 ) return value;
	for ( double d : values ) {
		value += d;
	}
	value /= values.size();
	return value;
}

ResultValues::ResultValues( uint32_t totalframes ):
							totalframes_{totalframes},
							detectedframes_{0},
							previousscore_{0.0},
							score_{0.0},
							averagematch_{0.0},
							lanedetectmultiplier_{0.0},
							firstpass_{true},
							optimalmat_{static_cast<int>(POLYGONSCALING * 480),
										static_cast<int>(POLYGONSCALING * 640),
										CV_8UC1,
										cv::Scalar(0)}
{
	Polygon optimalpolygon{ cv::Point(100,400),
							cv::Point(540,400),
							cv::Point(340,250),
							cv::Point(300,250) };
	cv::Point cvpointarray[4];
	for  (int i =0; i < 4; i++ ) {
		cvpointarray[i] = cv::Point( POLYGONSCALING * optimalpolygon[i].x,
									 POLYGONSCALING * optimalpolygon[i].y);
	}
	cv::fillConvexPoly( optimalmat_, cvpointarray, 4,  cv::Scalar(1) );
}

void ResultValues::NewIteration()
{
	detectedframes_ = 0;
	matchqueue_.clear();
	return;
}

void ResultValues::NewVariable()
{
	NewIteration();
	
	return;
}

void ResultValues::Push(Polygon polygon)
{
	if ( polygon[0] != cv::Point(0,0) ) {
		detectedframes_++;
		matchqueue_.push_back(PercentMatch(polygon, optimalmat_));
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
	averagematch_ = Average(matchqueue_);
	if ( firstpass_ ) {
		//Hardcoded now to tip balance to good average match
		lanedetectmultiplier_ = 0.35;
		/*
		//Adjust detected frame multiplier to bring inital score to 0!
		lanedetectmultiplier_ = averagematch_ * (static_cast<double>(totalframes_)
			/ static_cast<double>(detectedframes_));
		firstpass_ = false;
		*/
	}
	score_= lanedetectmultiplier_ * ((100.0 * detectedframes_) / (1.0 * totalframes_)) +
			(1.0 - lanedetectmultiplier_) * averagematch_;
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
