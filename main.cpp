/******************************************************************************************
  Date:    19.09.2016
  Author:  Nathan Greco (Nathan.Greco@gmail.com)

  Project:
      LaneDetectLearning: Machine learning algorithm to determine parameters for consistent
	  lane detection.

  Description:
      This application will process multiple video files to determine the best lande detect
	  parameters for consistent lane detection.  It is a tool to determine the best values
	  for the DAPrototype project's lane detection system.

      OpenCV 3.1.0 -> Compiled with OpenGL support, www.opencv.org

  Other notes:
      Style is following the Google C++ styleguide

  History:
      Date         Author      Description
-------------------------------------------------------------------------------------------
      19.09.2016   N. Greco    Initial creation
******************************************************************************************/

//Standard libraries
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>

//3rd party libraries
#include "opencv2/opencv.hpp"

//Project headers
#include "lane_detect_constants.h"
#include "lane_detect_processor.h"
#include "lane_constant_class.h"
#include "result_values_class.h"

//Forward declations
void UpdateLaneConstants(std::vector<LaneConstant> &laneconstants);

namespace lanedetectconstants {

	//Processing methods to perform
	bool enableblobcontour{false};
	bool enablesegmentblobcontour{false};
	
	//Blob detection parameters
	cv::SimpleBlobDetector::Params klanedetectblobparams() {
        cv::SimpleBlobDetector::Params params;
        params.thresholdStep = 5;
        params.minThreshold = 60;
        params.maxThreshold = 255;
        params.minRepeatability = 10;
        params.minDistBetweenBlobs = 10;
        params.filterByColor = true;
        params.blobColor = 255;
        params.filterByArea = true;
        params.minArea = 2.0f;
        params.maxArea = 200.0f;
        params.filterByCircularity = false;
        params.minCircularity = 0.9f;
        params.maxCircularity = 1.0f;
        params.filterByInertia = false;
        params.minInertiaRatio = 0.1f;
        params.maxInertiaRatio = 1.0f;
        params.filterByConvexity = true;
        params.minConvexity = 0.7f;
        params.maxConvexity = 1.0f;
        return params;
	}
	
	//Segment filters
	//uint16_t ksegmentlength{10};
	uint16_t ksegmentellipseheight{5};
	float ksegmentanglewindow{80.0f};
	float ksegmentlengthwidthratio{1.5f};
	
	//Construct from blob filters
	float kblobslopewindow{0.10f};
	
	//Construct from segment and blob filters
	float ksegmentblobanglewindow{20.0f};
	
	//Construct from segments filters
	float ksegmentsanglewindow{45.0f};
	
	//Final contour filters
	//uint16_t klength{100};
	uint16_t kellipseheight{60};
	float kanglewindow{80.0f};
	float klengthwidthratio{7.0f};
	
	//Scoring variables
    double kcommonanglewindow = 45.0;
    uint16_t kminroadwidth = 200.0;
    uint16_t kmaxroadwidth = 700.0;
	uint16_t koptimumwidth = 400;
	//weighting for best grade
	double klengthweight = 5.0;
	double kangleweight = -5.0;
	double kcenteredweight = -2.5;
	double kwidthweight = 0.0;		//-1.0;
	double klowestpointweight = 0.0;	//-0.25;	//Should be higher but I have bad test videos
	double klowestscorelimit = -DBL_MAX;
	
}

int main(int argc,char *argv[])
{
	//Check arguments passed
	if (argc < 2) {
		std::cout << "No arguments passed, press ENTER to exit..." << std::endl;
		std::cin.get();
		return 0;
	}
	
	
	//Create results file
	std::ofstream resultsfile("resultsfile.csv");
	if (!resultsfile.is_open()) {
		std::cout << "Results file failed to open, press ENTER to exit..." << std::endl;
		std::cin.get();
		return 0;
	}
		
	
	
	//Find total frames in all video files
	uint32_t totalframes{0};
	for (int i = 1; i < argc; i++ ) {
		cv::VideoCapture capture(argv[i]);
		totalframes += capture.get(cv::CAP_PROP_FRAME_COUNT);
		capture.release();
		resultsfile << argv[i] << std::endl;
	}
	resultsfile << std::endl;
	std::cout << (argc - 1) << " files to evaluate with " << totalframes <<
		" total frames" << std::endl;
	//Set how often to message console
	uint32_t messagecount{totalframes/100};	//Every 0.1%

	//Create variable classes -> Evaluated in order that they're pushed in!!!
	std::vector<LaneConstant> laneconstants;
	//Weights
	laneconstants.push_back( LaneConstant( "klowestpointweight",
		lanedetectconstants::klowestpointweight, 0.0, 10.0, 0.02) );
	laneconstants.push_back( LaneConstant( "kwidthweight",
		lanedetectconstants::kwidthweight, 0.0, 10.0, 0.02) );
	laneconstants.push_back( LaneConstant( "kcenteredweight",
		lanedetectconstants::kcenteredweight, -10.0, 0.0, 0.02) );
	laneconstants.push_back( LaneConstant( "kangleweight",
		lanedetectconstants::kangleweight, -10.0, 0.0, 0.02) );
	laneconstants.push_back( LaneConstant( "klengthweight",
		lanedetectconstants::klengthweight, 0.0, 10.0, 0.02) );
	//Pair filters
	laneconstants.push_back( LaneConstant( "koptimumwidth",
		lanedetectconstants::koptimumwidth, 100, 1200, 0.02) );
	laneconstants.push_back( LaneConstant( "kmaxroadwidth",
		lanedetectconstants::kmaxroadwidth, 400, 1200, 0.02) );
	laneconstants.push_back( LaneConstant( "kminroadwidth",
		lanedetectconstants::kminroadwidth, 100, 400, 0.02) );
	laneconstants.push_back( LaneConstant( "kcommonanglewindow",
		lanedetectconstants::kcommonanglewindow, 0.0, 180.0, 0.02) );
	//Final contour filters
	laneconstants.push_back( LaneConstant( "klengthwidthratio",
		lanedetectconstants::klengthwidthratio, 2.0, 100.0, 0.02) );
	laneconstants.push_back( LaneConstant( "kanglewindow",
		lanedetectconstants::kanglewindow, 0.0, 180.0, 0.02) );
	laneconstants.push_back( LaneConstant( "kellipseheight",
		lanedetectconstants::kellipseheight, 20.0, 120.0, 0.02) );
	//Construct from segments filters
	laneconstants.push_back( LaneConstant( "ksegmentsanglewindow",
		lanedetectconstants::ksegmentsanglewindow, 0.0, 90.0, 0.02) );
	//Skip blobs
	//Segment filters
	laneconstants.push_back( LaneConstant( "ksegmentellipseheight",
		lanedetectconstants::ksegmentellipseheight, 0.0, 120.0, 0.02) );
	laneconstants.push_back( LaneConstant( "ksegmentlengthwidthratio",
		lanedetectconstants::ksegmentlengthwidthratio, 1.0, 10.0, 0.02) );
	laneconstants.push_back( LaneConstant( "ksegmentanglewindow",
		lanedetectconstants::ksegmentanglewindow, 0.0, 1800.0, 0.02) );
	
	//
	std::cout << laneconstants.size() << " variables to modify" << std::endl;
	
	//Create header of resultsfile file
	resultsfile << "Iteration" << ",";
	for( int i = 0; i < laneconstants.size(); i++ ) {
		resultsfile << laneconstants[i].variablename_ << ",";
	}
	resultsfile << "standard deviation" << ",";
	resultsfile << "frames detected" << "," << "total frames" << "," << "score" << ",";
	resultsfile << "runtime" << "," << "fps" << "," << std::endl;

	
	//Create resultsfile vector
	ResultValues resultvalues;
	int iterationcount{0};
	
	//Iterate through each variable
	for ( int i = 0; i < laneconstants.size(); i++ ) {
		if ( i != 0 ) laneconstants[i].Modify();
		for(;;) {
			resultvalues.NewIteration();
			UpdateLaneConstants(laneconstants);
			std::chrono::high_resolution_clock::time_point starttime;
			starttime =  std::chrono::high_resolution_clock::now();
			iterationcount++;
			uint32_t frameschecked{0};
			resultsfile << iterationcount << "," << std::fixed << std::setprecision(4);
			for( int j = 0; j < laneconstants.size(); j++ ) {
				resultsfile << laneconstants[j].value_ << ",";
			}
			
			//iterate through each file	
			for (int j = 1; j < argc; j++ ) {
				cv::VideoCapture capture(argv[j]);
				for( int k =0; k < capture.get(cv::CAP_PROP_FRAME_COUNT) - 1; k++  ){
					cv::Mat frame;
					Polygon polygon;
					capture >> frame;
					ProcessImage( frame, polygon );
					resultvalues.Push( polygon );
					frameschecked++;
					if (frameschecked%messagecount == 0) {
						std::cout << "Iteration " << iterationcount << ", file " << j << ", ";
						std::cout << std::fixed << std::setprecision(0);
						std::cout << ((100.0*k)/capture.get(cv::CAP_PROP_FRAME_COUNT));
						std::cout << "% file, " << ((100.0*frameschecked)/totalframes);
						std::cout << "% iteration, variable: ";
						std::cout << laneconstants[i].variablename_ << std::endl;
					}
				}
				capture.release();
				resultvalues.NewPattern();
			}
			
			//Evaluate iteration
			resultvalues.Update(totalframes);
			double runtime{std::chrono::duration_cast<std::chrono::microseconds>
				(std::chrono::high_resolution_clock::now() - starttime).count()/1000000.0};
			double fps{totalframes/runtime};
			
			//Update results file
			resultsfile << resultvalues.polygondev_ << ",";
			resultsfile << resultvalues.detectedframes_ << "," << totalframes << ",";
			resultsfile << resultvalues.score_ << ",";
			resultsfile << std::fixed << std::setprecision(2) << runtime << ",";
			resultsfile << fps << "," << std::endl;
		
			//Evaluate results and adjust
			if ( resultvalues.bestpassed_  || (laneconstants[i].reversedcount_ > 2)) {
				laneconstants[i].SetPrevious();
				resultvalues.SetPreviousScore();
				break;
			} else if ( laneconstants[i].hitlimit_ && resultvalues.improved_ ) {
				break;
			} else if ( resultvalues.improved_ ) {
				laneconstants[i].Modify();
			} else {
				resultvalues.SetPreviousScore();
				laneconstants[i].SetPrevious();
				laneconstants[i].Reverse();
				laneconstants[i].Modify();
			}
		}
	}
	
	//Close up shop
	resultsfile.close();
	return 1;
}

/*****************************************************************************************/
void UpdateLaneConstants(std::vector<LaneConstant> &laneconstants)
{
	//Boy, a string based switch statement would be nice...
	
	//This code is hideous, come back later with clever ideas
	lanedetectconstants::klowestpointweight = laneconstants[0].value_;
	lanedetectconstants::kwidthweight = laneconstants[1].value_;
	lanedetectconstants::kcenteredweight = laneconstants[2].value_;
	lanedetectconstants::kangleweight = laneconstants[3].value_;
	lanedetectconstants::klengthweight = laneconstants[4].value_;
	lanedetectconstants::koptimumwidth = laneconstants[5].value_;
	lanedetectconstants::kmaxroadwidth = laneconstants[6].value_;
	lanedetectconstants::kminroadwidth = laneconstants[7].value_;
	lanedetectconstants::kcommonanglewindow = laneconstants[8].value_;
	lanedetectconstants::klengthwidthratio = laneconstants[9].value_;
	lanedetectconstants::kanglewindow = laneconstants[10].value_;
	lanedetectconstants::kellipseheight = laneconstants[11].value_;
	lanedetectconstants::ksegmentsanglewindow = laneconstants[12].value_;
	lanedetectconstants::ksegmentellipseheight = laneconstants[13].value_;
	lanedetectconstants::ksegmentlengthwidthratio = laneconstants[14].value_;
	lanedetectconstants::ksegmentsanglewindow = laneconstants[15].value_;
	return;
}