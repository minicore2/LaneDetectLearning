#ifndef RESULTVALUES_H
#define RESULTVALUES_H

#include <deque>
#include "opencv2/opencv.hpp"
#include "lane_detect_processor.h"

class ResultValues
{
	public:
		ResultValues();
		void NewPattern();
		void Push(Polygon polygon);
		void Update(uint32_t totalframes);
		void SetPreviousScore();
		void NewIteration();
		bool bestpassed_;
		bool improved_;
		bool hasimproved_;
		double polygondev_;
		double score_;
		uint32_t detectedframes_;

	protected:

	private:
		double previousscore_;
		int samescore_;
		std::deque<Polygon> polygonqueue_;
		std::deque<double> polygondevqueue_;
};

#endif // RESULTVALUES_H
