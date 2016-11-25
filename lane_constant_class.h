#ifndef LANECONSTANT_H
#define LANECONSTANT_H

#include <string>
#include "result_values_class.h"

class LaneConstant
{
	friend class ResultValues;
	public:
		LaneConstant( std::string variablename,
					  double initialvalue,
					  double minvalue,
					  double maxvalue,
					  double increment );
		void Modify();
		std::string variablename_;
		double value_;
		bool finished_;
	protected:

	private:
		void Reverse();
		double minvalue_;
		double maxvalue_;
		double increment_;
		double direction_;
		double range_;
		double bestscore_;
		double bestvalue_;
		double initialvalue_;
		void SetPrevious();
		double previousvalue_;
		bool hitlimit_;
		bool firstpass_;
		int reversedcount_;
};

#endif // LANECONSTANT_H
