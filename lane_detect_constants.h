#ifndef LANEDETECTCONSTANTS_H
#define LANEDETECTCONSTANTS_H

namespace lanedetectconstants {
	
	extern uint16_t ksegmentellipseheight;
	extern float ksegmentanglewindow;
	extern float ksegmentlengthwidthratio;
	
	//Construct from segments filters
	extern float ksegmentsanglewindow;
	
	//Final contour filters
	//uint16_t klength;
	extern uint16_t kellipseheight;
	extern float kanglewindow;
	extern float klengthwidthratio;
	
	//Scoring variables
    extern double kcommonanglewindow;
    extern uint16_t kminroadwidth;
    extern uint16_t kmaxroadwidth;
	extern uint16_t koptimumwidth;
	//weighting for best grade
	extern double klengthweight;
	extern double kangleweight;
	extern double kcenteredweight;
	extern double kwidthweight;		//-1.0;
	extern double klowestpointweight;	//-0.25;	//Should be higher but I have bad test videos
	extern double klowestscorelimit;
	
}

#endif // LANEDETECTCONSTANTS_H
