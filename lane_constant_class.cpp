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
							bestvalue_{ initialvalue },
							minvalue_{ minvalue },
							maxvalue_{ maxvalue },
							increment_{ increment },
							direction_{1.0},
							reversedcount_{0},
							hitlimit_{false},
							finished_{false},
							firstpass_{true},
							bestscore_{0.0}
{
	range_ = maxvalue - minvalue;
	if ( (increment_*direction_) > 0 ) {
		if (value_ == maxvalue) Reverse();
	} else {
		if (value_ == minvalue_) Reverse();
	}
}

void LaneConstant::Modify()
{
	previousvalue_ = value_;
	value_ += (direction_ * range_ * increment_);
	if (value_ < minvalue_) {
		hitlimit_ = true;
		value_ = minvalue_;
	}
	if (value_ > maxvalue_) {
		hitlimit_ = true;
		value_ = maxvalue_;
	}
	return;
}

void LaneConstant::Reverse()
{
	reversedcount_++;
	direction_ *= -1.0;
	SetPrevious();
	return;
}

void LaneConstant::SetPrevious()
{
	value_ = previousvalue_;
	return;
}
