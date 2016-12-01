#include <deque>
#include <math.h>
#include "opencv2/opencv.hpp"
#include "result_values_class.h"
#include "lane_detect_processor.h"
#include "lane_detect_constants.h"
#include "lane_constant_class.h"

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

/*****************************************************************************************/
float PercentMatch( const Polygon& polygon,
					const cv::Mat& optimalmat )
{
	//Create blank mat
	cv::Mat polygonmat{ cv::Mat(optimalmat.rows,
								optimalmat.cols,
								CV_8UC1,
								cv::Scalar(0)) };
	
	//Draw polygon
	cv::Point cvpointarray[4];
	for  (int i =0; i < 4; i++ ) {
		cvpointarray[i] = polygon[i];
	}
	cv::fillConvexPoly( polygonmat, cvpointarray, 4,  cv::Scalar(2) );

	//Add together
	polygonmat += optimalmat;
	
	//Evaluate result
	uint32_t excessarea{ 0 };
	uint32_t overlaparea{ 0 };
	for ( int i = 0; i < polygonmat.rows; i++ ) {
		uchar* p { polygonmat.ptr<uchar>(i) };
		for ( int j = 0; j < polygonmat.cols; j++ ) {
			switch ( p[j] )
			{
				case 1:
					excessarea++;
					break;
				case 2:
					excessarea++;
					break;
				case 3:
					overlaparea++;
					break;
			}
		}
	}
	return (100.0f * overlaparea) / (overlaparea + excessarea);
}

ResultValues::ResultValues( uint32_t totalframes ):
							totalframes_{totalframes},
							detectedframes_{0},
							previousscore_{0.0},
							score_{0.0},
							averagematch_{0.0},
							lanedetectmultiplier_{0.0},
							firstpass_{true},
							optimalmat_{480,
										800,
										CV_8UC1,
										cv::Scalar(0)}
{
	cv::Point cvpointarray[4];
	Polygon optimalpolygon{ cv::Point(110,480),
							cv::Point(690,480),
							cv::Point(390,250),
							cv::Point(410,250) };
	for  (int i =0; i < 4; i++ ) {
		cvpointarray[i] = optimalpolygon[i];
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
		lanedetectmultiplier_ = 0.30;
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

	//Temporary code just to iterate through span of all variables
/*
	if ( laneconstant.hitlimit_ ) {
			laneconstant.finished_ = true;	
			laneconstant.value_ = laneconstant.initialvalue_;
			return;
	} else {
		laneconstant.Modify();
	}
*/
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
