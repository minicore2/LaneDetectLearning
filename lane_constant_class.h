#ifndef LANECONSTANT_H
#define LANECONSTANT_H

#include <string>

class LaneConstant
{
	public:
		LaneConstant( std::string variablename,
					  double initialvalue,
					  double minvalue,
					  double maxvalue,
					  double increment );
		void Modify();
		void Reverse();
		void SetPrevious();
		std::string variablename_;
		double value_;

	protected:

	private:
		double minvalue_;
		double maxvalue_;
		double increment_;
		double direction_;
		double range_;
		double previousvalue_;
};

#endif // LANECONSTANT_H
