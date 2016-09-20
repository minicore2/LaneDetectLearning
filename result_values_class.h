#ifndef RESULTVALUES_H
#define RESULTVALUES_H

#include <queue>
#include "opencv2/opencv.hpp"
#include "lane_detect_processor.h"

class ResultValues
{
	public:
		ResultValues();
		void NewPattern();
		void Push(Polygon polygon);
		void Update();
		double score_;
		bool bestpassed_;
		bool improved_;

	protected:

	private:
};

#endif // RESULTVALUES_H
