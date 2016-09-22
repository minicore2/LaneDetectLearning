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
		void Update( LaneConstant laneconstant );
		void NewIteration();
		void NewVariable();
		void SetPrevious();
		bool bestpassed_;
		bool improved_;
		bool hasimproved_;
		double polygondev_;
		double score_;
		uint32_t detectedframes_;

	protected:

	private:
	uint32_t totalframes_;
		double previousscore_;
		int samescore_;
		std::deque<Polygon> polygonqueue_;
		std::deque<double> polygondevqueue_;
		bool firstpass_;
};

#endif // RESULTVALUES_H
