#include <string>
#include "lane_constant_class.h"

LaneConstant::LaneConstant( std::string variablename,
					  double initialvalue,
					  double minvalue,
					  double maxvalue,
					  double increment ):
					  variablename_{ variablename },
					  value_{ initialvalue },
					  minvalue_{ minvalue },
					  maxvalue_{ maxvalue },
					  increment_{ increment }
{
	
}
