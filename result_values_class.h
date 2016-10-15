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
		void NewPattern();
		void Push(Polygon polygon);
		void Update( LaneConstant& laneconstant );
		void NewIteration();
		void NewVariable();
		double polygondev_;
		double outputscore_;
		uint32_t detectedframes_;

	protected:

	private:
		double score_;
		double previousscore_;
		bool firstpass_;
		double lanedetectmultiplier_;
		uint32_t totalframes_;
		std::deque<Polygon> polygonqueue_;
		std::deque<double> polygondevqueue_;
};

#endif // RESULTVALUES_H
