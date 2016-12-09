/******************************************************************************************
  Date:    12.08.2016
  Author:  Nathan Greco (Nathan.Greco@gmail.com)

  Project:
      DAPrototype: Driver Assist Prototype
	  http://github.com/NateGreco/DAPrototype.git

  License:
	  This software is licensed under GNU GPL v3.0
	  
******************************************************************************************/

//Header guard
#ifndef LANEDETECTCONSTANTS_H
#define LANEDETECTCONSTANTS_H

//Project libraries
#include "lane_detect_processor.h"

/*****************************************************************************************/
namespace lanedetectconstants {
	//Default polygon
	extern Polygon defaultpolygon;

	//Image evaluation
	extern float k_contrastscalefactor;
	
	//Segment filtering
	extern uint16_t k_segmentminimumsize;
	extern uint16_t k_verticalsegmentlimit;
	extern float k_maxvanishingpointangle;
	extern uint16_t k_vanishingpointx;
	extern uint16_t k_vanishingpointy;
		
	//Line filtering
	extern uint16_t k_verticallimit;
	extern uint16_t k_minimumsize;
	extern uint16_t k_maxlinegap;
	extern uint16_t k_threshold;
	
	//Polygon filtering
    extern uint16_t k_minroadwidth;
    extern uint16_t k_maxroadwidth;
	
	//Scoring
	extern float k_lowestscorelimit;
	extern float k_weightedheightwidth;
	extern float k_weightedangleoffset;
	extern float k_weightedcenteroffset;
	
}

#endif // LANEDETECTCONSTANTS_H
