#ifndef RESULTVALUES_H
#define RESULTVALUES_H

#include <deque>
#include "opencv2/opencv.hpp"
#include "lane_detect_processor.h"

class LaneConstant;
class ResultValues
{
	public:
		ResultValues( uint32_t totalframes );
		void Push(Polygon polygon);
		void Update( LaneConstant& laneconstant );
		void NewIteration();
		void NewVariable();
		double averagematch_;
		double outputscore_;
		uint32_t detectedframes_;
		cv::Mat optimalmat_;

	protected:

	private:
		double score_;
		double previousscore_;
		bool firstpass_;
		double lanedetectmultiplier_;
		uint32_t totalframes_;
		std::deque<float> matchqueue_;
};

#endif // RESULTVALUES_H
