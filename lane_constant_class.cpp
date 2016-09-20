#include <string>
#include "lane_constant_class.h"

LaneConstant::LaneConstant( std::string variablename,
					  double initialvalue,
					  double minvalue,
					  double maxvalue,
					  double increment ):
					  variablename_{ variablename },
					  value_{ initialvalue },
					  previousvalue_ { initialvalue },
					  minvalue_{ minvalue },
					  maxvalue_{ maxvalue },
					  increment_{ increment },
					  direction_{1}
{
	range_ = maxvalue - minvalue;
}

void LaneConstant::Modify()
{
	previousvalue_ = value_;
	value_ += direction_*range_*increment_;
	if (value_ < minvalue_) value_ = minvalue_;
	if (value_ > maxvalue_) value_ = maxvalue_;
	return;
}

void LaneConstant::Reverse()
{
	direction_ *= -1.0;
	return;
}

void LaneConstant::SetPrevious()
{
	value_ = previousvalue_;
	return;
}
