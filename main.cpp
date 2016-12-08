/******************************************************************************************
  Date:    19.09.2016
  Author:  Nathan Greco (Nathan.Greco@gmail.com)

  Project:
      LaneDetectLearning: Machine learning algorithm to determine parameters for consistent
	  lane detection.

  Description:
      This application will process multiple video files to determine the best lane detect
	  parameters for consistent lane detection.  It is a tool to determine the best values
	  for the DAPrototype project's lane detection system.

      OpenCV 3.1.0 -> Compiled with OpenGL support, www.opencv.org

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
#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

//3rd party libraries
#include "opencv2/opencv.hpp"

//Project headers
#include "lane_detect_constants.h"
#include "lane_detect_processor.h"
#include "lane_constant_class.h"
#include "result_values_class.h"

/*****************************************************************************************/
//Forward declations
void UpdateLaneConstants(std::vector<LaneConstant> &laneconstants);
void FrameLoaderThread( cv::VideoCapture* videocapture,
						std::mutex* framesmutex,
						std::queue<cv::Mat>* frames,
						std::atomic<bool>* done);
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
	uint32_t messagecount{totalframes/100};	//Every 1%

	//Create variable classes
	double increment{1.0};
	std::vector<LaneConstant> laneconstants;
	//Sort by sequence in code!
	//laneconstants.push_back( LaneConstant( "k_vanishingpointy",
	//	lanedetectconstants::k_vanishingpointy, 220.0, 260, 0.05*increment) );
	//laneconstants.push_back( LaneConstant( "k_segmentminimumsize",
	//	lanedetectconstants::k_segmentminimumsize, 20.0, 40.0, 0.05*increment) );
	//laneconstants.push_back( LaneConstant( "k_minimumsize",
	//	lanedetectconstants::k_minimumsize, 10.0, 80.0, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "k_threshold",
		lanedetectconstants::k_threshold, 20.0, 80.0, 0.05*increment) );
	//laneconstants.push_back( LaneConstant( "k_maxlinegap",
	//	lanedetectconstants::k_maxlinegap, 1.0, 8.0, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "k_weightedangleoffset",
		lanedetectconstants::k_weightedangleoffset, -10.0, -1.0, -0.05*increment) );
	laneconstants.push_back( LaneConstant( "k_weightedcenteroffset",
		lanedetectconstants::k_weightedcenteroffset,-10.0, -1.0, -0.05*increment) );
	laneconstants.push_back( LaneConstant( "k_weightedheightwidth",
		lanedetectconstants::k_weightedheightwidth, 100.0, 400.0, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "k_lowestscorelimit",
		lanedetectconstants::k_lowestscorelimit, -500.0, 500.0, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "k_maxvanishingpointangle",
		lanedetectconstants::k_maxvanishingpointangle, 18.0, 24.0, -0.05*increment) );
	laneconstants.push_back( LaneConstant( "k_minroadwidth",
		lanedetectconstants::k_minroadwidth, 400.0, 600.0, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "k_maxroadwidth",
		lanedetectconstants::k_maxroadwidth, 600.0, 800.0, 0.05*increment) );
	laneconstants.push_back( LaneConstant( "k_contrastscalefactor",
		lanedetectconstants::k_contrastscalefactor, 0.2, 0.4, 0.05*increment) );
	std::cout << laneconstants.size() << " variables to modify" << std::endl;
	
	//Create header of resultsfile file
	resultsfile << "Iteration" << ",";
	for( int i = 0; i < laneconstants.size(); i++ ) {
		resultsfile << laneconstants[i].variablename_ << ",";
	}
	resultsfile << "average match" << ",";
	resultsfile << "frames detected" << "," << "total frames" << "," << "percent detected";
	resultsfile << "," << "score" << "," << "runtime" << "," << "fps" << "," << std::endl;

	
	//Create resultsfile vector
	ResultValues resultvalues{totalframes};
	int iterationcount{0};
	bool first{true};
	
	//Iterate through each variable
	for ( int i = 0; i < laneconstants.size(); i++ ) {
	//for ( int i = laneconstants.size() - 1; i >= 0; i-- ) {
		if ( !first ) laneconstants[i].Modify();
		first = false;
		resultvalues.NewVariable();
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
				std::mutex framesmutex;
				std::queue<cv::Mat> frames;
				std::atomic<bool> done{false};
				//Multi-threading saves ~20% runtime
				std::thread t_imagequeue( FrameLoaderThread, &capture, &framesmutex, &frames, &done);
				t_imagequeue.detach();
				int framecount{0};
				while ( !(done && frames.empty()) ) {
					framesmutex.lock();
					if ( frames.empty() ) {
						framesmutex.unlock();
						continue;
					}
					framecount++;
					cv::Mat frame{frames.front()};
					frames.pop();
					framesmutex.unlock();
					Polygon polygon;
					ProcessImage( frame, polygon );
					resultvalues.Push( polygon );
					frameschecked++;
					if (frameschecked%messagecount == 0) {
						std::cout << "Iteration " << iterationcount << ", file "
								  << j << ", ";
						std::cout << std::fixed << std::setprecision(0);
						std::cout << ((100.0*framecount)/capture.get(cv::CAP_PROP_FRAME_COUNT));
						std::cout << "% file, " << ((100.0*frameschecked)/totalframes);
						std::cout << "% iteration, variable: ";
						std::cout << laneconstants[i].variablename_ << std::endl;
					}
				}
				capture.release();
			}
			
			//Update
			resultvalues.Update(laneconstants[i]);
			double runtime{std::chrono::duration_cast<std::chrono::microseconds>
				(std::chrono::high_resolution_clock::now() - starttime).count()/1000000.0};
			double fps{totalframes/runtime};
			resultsfile << resultvalues.averagematch_ << ",";
			resultsfile << resultvalues.detectedframes_ << "," << totalframes << ",";
			resultsfile << std::fixed << std::setprecision(2);
			resultsfile << ((resultvalues.detectedframes_ * 100.0) / totalframes) << ",";
			resultsfile << resultvalues.outputscore_ << ",";
			resultsfile << std::fixed << std::setprecision(3) << runtime << ",";
			resultsfile << fps << "," << std::endl;
			if (laneconstants[i].finished_) break;
		}
	}
	resultsfile << "Final" << ",";
	for( int i = 0; i < laneconstants.size(); i++ ) {
		resultsfile << laneconstants[i].value_ << ",";
	}
	resultsfile << std::endl;
	
	//Close up shop
	resultsfile.close();
	return 1;
}

/*****************************************************************************************/
void UpdateLaneConstants(std::vector<LaneConstant> &laneconstants)
{
	
	for ( LaneConstant &l : laneconstants) {
		if (l.variablename_ == "k_weightedheightwidth" ) {
			lanedetectconstants::k_weightedheightwidth = l.value_;
		} else if (l.variablename_ == "k_contrastscalefactor" ) {
			lanedetectconstants::k_contrastscalefactor = l.value_;
		} else if (l.variablename_ == "k_maxvanishingpointangle" ) {
			lanedetectconstants::k_maxvanishingpointangle = l.value_;
		//} else if (l.variablename_ == "k_segmentlengthwidthratio" ) {
		//	lanedetectconstants::k_segmentlengthwidthratio = l.value_;
		} else if (l.variablename_ == "k_minimumsize" ) {
			lanedetectconstants::k_minimumsize = l.value_;
		} else if (l.variablename_ == "k_maxlinegap" ) {
			lanedetectconstants::k_maxlinegap = l.value_;
		} else if (l.variablename_ == "k_threshold" ) {
			lanedetectconstants::k_threshold = l.value_;
		//} else if (l.variablename_ == "k_minimumangle" ) {
		//	lanedetectconstants::k_minimumangle = l.value_;
		//} else if (l.variablename_ == "k_lengthwidthratio" ) {
		//	lanedetectconstants::k_lengthwidthratio = l.value_;
		} else if (l.variablename_ == "k_minroadwidth" ) {
			lanedetectconstants::k_minroadwidth = l.value_;
		} else if (l.variablename_ == "k_maxroadwidth" ) {
			lanedetectconstants::k_maxroadwidth = l.value_;
		//} else if (l.variablename_ == "k_segmentsanglewindow" ) {
		//	lanedetectconstants::k_segmentsanglewindow = l.value_;
		} else if (l.variablename_ == "k_segmentminimumsize" ) {
			lanedetectconstants::k_segmentminimumsize = l.value_;
		} else if (l.variablename_ == "k_weightedcenteroffset" ) {
			lanedetectconstants::k_weightedcenteroffset = l.value_;
		} else if (l.variablename_ == "k_weightedangleoffset" ) {
			lanedetectconstants::k_weightedangleoffset = l.value_;
		} else if (l.variablename_ == "k_vanishingpointy" ) {
			lanedetectconstants::k_vanishingpointy = l.value_;
		} else if (l.variablename_ == "k_lowestscorelimit" ) {
			lanedetectconstants::k_lowestscorelimit = l.value_;
		} else {
			std::cout << "Programming error, variable does not exist!" << std::endl;
			std::cin.get();
			exit(0);
		}
	}

	return;
}

/*****************************************************************************************/
void FrameLoaderThread(cv::VideoCapture* videocapture, std::mutex* framesmutex, std::queue<cv::Mat>* frames, std::atomic<bool>* done)
{
	for( int i =0; i < videocapture->get(cv::CAP_PROP_FRAME_COUNT) - 1; i++  ){
		cv::Mat frame;
		for (;;) {			//Don't let queqed frames get too large or program crashes
			framesmutex->lock();
			unsigned long int framesqueqed { frames->size() };
			framesmutex->unlock();
			if (framesqueqed < 1000) {
				break;
			} else {
				std::this_thread::sleep_for(std::chrono::microseconds(500000)); //500ms
			}
		}
		*videocapture >> frame;
		framesmutex->lock();
		frames->push(frame);
		framesmutex->unlock();
	}
	*done = true;
	return;
}