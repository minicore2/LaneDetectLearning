#ifndef RESULTVALUES_H
#define RESULTVALUES_H

#include <queue>
#include "opencv2/opencv.hpp"
#include "lane_detect_processor.h"

class ResultValues
{
	public:
		ResultValues();
		virtual ~ResultValues();
		void NewPattern();
		void Update(Polygon polygon);
		double Score();
		bool BestReached();

	protected:

	private:
};

#endif // RESULTVALUES_H
