#include <queue>
#include "opencv2/opencv.hpp"
#include "result_values_class.h"
#include "lane_detect_processor.h"

ResultValues::ResultValues()
{
	
}

void ResultValues::NewPattern()
{
	return;
}

void ResultValues::Push(Polygon polygon)
{
	return;
}

void ResultValues::Update()
{
	score_= 0.0;
	bestpassed_ = true;
	improved_ = false;
	return;
}
