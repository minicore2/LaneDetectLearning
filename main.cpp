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
     double kcommonanglewindow{45.0};
     uint16_t kminroadwidth {200};
     uint16_t kmaxroadwidth {700};
	 uint16_t koptimumwidth {400};
	//weighting for best grade
	 double klengthweight {5.0};
	 double kangleweight {-5.0};
	 double kcenteredweight {-2.5};
	 double kwidthweight {0.0};
	 double klowestpointweight {0.0};
	 double klowestscorelimit {-DBL_MAX};
	
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
		lanedetectconstants::klowestpointweight, 0.0, 10.0, 0.05) );
	laneconstants.push_back( LaneConstant( "kwidthweight",
		lanedetectconstants::kwidthweight, 0.0, 10.0, 0.05) );
	laneconstants.push_back( LaneConstant( "kcenteredweight",
		lanedetectconstants::kcenteredweight, 0.0, 10.0, 0.05) );
	laneconstants.push_back( LaneConstant( "kangleweight",
		lanedetectconstants::kangleweight, 0.0, 10.0, 0.05) );
	laneconstants.push_back( LaneConstant( "klengthweight",
		lanedetectconstants::klengthweight, 0.0, 10.0, 0.05) );
	//Pair filters
	laneconstants.push_back( LaneConstant( "koptimumwidth",
		lanedetectconstants::koptimumwidth, 0.0, 10.0, 0.05) );
	laneconstants.push_back( LaneConstant( "kmaxroadwidth",
		lanedetectconstants::kmaxroadwidth, 0.0, 10.0, 0.05) );
	laneconstants.push_back( LaneConstant( "kminroadwidth",
		lanedetectconstants::kminroadwidth, 0.0, 10.0, 0.05) );
	laneconstants.push_back( LaneConstant( "kcommonanglewindow",
		lanedetectconstants::kcommonanglewindow, 0.0, 10.0, 0.05) );
	//
	std::cout << laneconstants.size() << " variables to modify" << std::endl;
	
	//Create header of resultsfile file
	resultsfile << "Iteration" << ",";
	for( int i = 0; i < laneconstants.size(); i++ ) {
		resultsfile << laneconstants[i].variablename_ << ",";
	}
	resultsfile << "lane pos std" << "," << "lane width std" << "," << "lane angle std" << ",";
	resultsfile << "frames detected" << "," << "total frames" << "," << "score" << ",";
	resultsfile << "runtime" << "," << "fps" << "," << std::endl;

	
	//Create resultsfile vector
	int iterationcount{0};
	std::vector<double> scores;
	
	//Iterate through each variable
	for ( int i = 0; i < laneconstants.size(); i++ ) {
		bool bestnotreached{true};
		while (bestnotreached) {
			std::chrono::high_resolution_clock::time_point starttime;
			starttime =  std::chrono::high_resolution_clock::now();
			ResultValues resultvalues;
			iterationcount++;
			uint32_t frameschecked{0};
			uint32_t detectedframes{0};
			resultsfile << i << "," << std::fixed << std::setprecision(4);
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
					resultvalues.Update( polygon );
					if ( polygon[0] != cv::Point(0,0) ) detectedframes++;
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
			
			double runtime{std::chrono::duration_cast<std::chrono::microseconds>
				(std::chrono::high_resolution_clock::now() - starttime).count()/1000000.0};
			double fps{totalframes/runtime};
			
			//Update results file
			resultsfile << ",,,";	//ToDo - standard deviations
			resultsfile << detectedframes << "," << totalframes << ",";
			resultsfile << resultvalues.Score() << ",";
			resultsfile << std::fixed << std::setprecision(2) << runtime << ",";
			resultsfile << fps << "," << std::endl;
		
			//Evaluate results and adjust
			scores.push_back(resultvalues.Score());
			bestnotreached = !resultvalues.BestReached();
		}
	}
	
	//Close up shop
	resultsfile.close();
	return 1;
}


/*****************************************************************************************/