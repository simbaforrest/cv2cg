/************************************************************************\

  Copyright 2011 The University of Michigan.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software
  and its documentation for educational, research and non-profit
  purposes, without fee, and without a written agreement is
  hereby granted, provided that the above copyright notice and
  the following paragraph appear in all copies.

  THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY OF MICHIGAN "AS IS" AND 
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF MICHIGAN
  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
  Authors:

			Chen Feng
            Laboratory for Interactive Visualization in Engineering (LIVE)
			Department of Civil and Environmental Engineering
            2350 Hayward Street, Suite 2340 GG Brown Building
            University of Michigan
            Ann Arbor, MI 48109-2125
			Phone:    (734)764-8495
			EMail:    simbaforrest@gmail.com
			WWW site: http://www.umich.edu/~cforrest
            
			Vineet R. Kamat
            Laboratory for Interactive Visualization in Engineering (LIVE)
			Department of Civil and Environmental Engineering
            2350 Hayward Street, Suite 2340 GG Brown Building
            University of Michigan
            Ann Arbor, MI 48109-2125
            Phone:    (734)764-4325
			EMail:    vkamat@umich.edu
			WWW site: http://pathfinder.engin.umich.edu

\************************************************************************/

#include <iostream>
#include <limits>
#include <string>
#include <fstream>
#include <sstream>

#include "Log.h"
#include "OpenCVHelper.h"
#include "OSGHelper.h"
#include "OpenCV2OSG.h"
#include "CV2CG.h"

#include "keg/KEGTracker.hpp"

#include "apriltag/apriltag.hpp"
#include "apriltag/TagFamilyFactory.hpp"

Log::Level Log::level = Log::LOG_INFO;

using namespace cv;
using namespace std;
using april::tag::INT64;
using april::tag::TagFamily;
using april::tag::TagFamilyFactory;
using april::tag::TagDetector;
using april::tag::TagDetection;
using helper::ImageSource;

struct KEGprocessor : public ImageHelper::ImageSource::Processor {
/////// Vars
	KEGTracker tracker;
	Mat gray;
	bool needToInit;
	bool doCapFrame;
	bool needToCapframe;
	bool videoFromWebcam;
	bool onlyApril;
	double threshKeydistance;
	int framecnt;

	cv::Ptr<TagFamily> tagFamily;
	cv::Ptr<TagDetector> detector;
	cv::Mat HI;
	cv::Mat iHI; //inverse of init Homography, X_tag = iHI * X_keg
	int targetid; //default target tag0

/////// Constructor
	KEGprocessor() {
		onlyApril = false;
		needToInit = true;
		doCapFrame = true;
		needToCapframe = false;
		videoFromWebcam = false;
		threshKeydistance = 200;
		framecnt = 0;
		targetid = 0;
	}

	bool findAprilTag(Mat &image, Mat &ret,
		bool draw=false, int errorThresh=0) {
		vector<TagDetection> detections;
		double opticalCenter[2] = { image.cols/2.0, image.rows/2.0 };
		detector->process(image, opticalCenter, detections);

		loglnd("[findAprilTag] #detections="<<detections.size());
		for(int id=0; id<(int)detections.size(); ++id) {
			TagDetection &dd = detections[id];
			if(dd.id!=targetid) continue;
			if(dd.hammingDistance>errorThresh) continue; //very strict!

			cv::Mat tmp(3,3,CV_64FC1, (double*)dd.homography[0]);
			double vm[] = {1,0,dd.hxy[0],0,1,dd.hxy[1],0,0,1};
			ret = cv::Mat(3,3,CV_64FC1,vm) * tmp;

			if(draw) {
				cv::putText( image, helper::num2str(dd.id),
					cv::Point(dd.cxy[0],dd.cxy[1]), CV_FONT_NORMAL,
					1, helper::CV_BLUE, 2 );
				static double crns[4][2]={
					{-1, -1},
					{ 1, -1},
					{ 1,  1},
					{-1,  1}
				};
				helper::drawHomography(image, ret, crns);
			}
			loglnd("[findAprilTag] find target with id="<<targetid);
			return true;
		}
		return false;
	}

/////// Override
	void operator()(cv::Mat& frame) {
		if(frame.empty()) return;

		cvtColor(frame, gray, CV_BGR2GRAY);
		double rms=-1, ncc=-1;
		double camR[3][3]={{1,0,0},{0,1,0},{0,0,1}},camT[3]={0};
		static double lastT[3]={0};

		if( onlyApril ) { //for test
			Mat tmpH;
			needToInit = true;
			if( findAprilTag(frame, tmpH, true) ) {
				Mat initH = tmpH * iHI;
				needToInit = !tracker.init(gray, initH, rms, ncc, 0, 1);
				if(!needToInit) tracker.draw3D(frame);
			}
		} else { // KEG
			if( needToInit ) {
				loglni("[KEGprocessor] INITing...");
				Mat tmpH;
				if( findAprilTag(frame, tmpH, true) ) {
					Mat initH = tmpH * iHI;
					needToInit = !tracker.init(gray, initH, rms, ncc);
//					needToInit=!tracker.init(gray,rms,ncc);
					if(!needToInit) loglni("[KEGprocessor] ...INITed");
				}
			} else {
				needToInit=!tracker(gray, &frame, rms, ncc);
			}
		}

		if(!needToInit)
			tracker.GetCameraPose(camR,camT);
		else if(doCapFrame && !videoFromWebcam) {
			for(int i=0; i<3; ++i) {
				camT[i]=numeric_limits<double>::quiet_NaN();
				for(int j=0; j<3; ++j)
					camR[i][j]=numeric_limits<double>::quiet_NaN();
			}
			rms = ncc = numeric_limits<double>::quiet_NaN();
		}
		if (doCapFrame) {
			if(!videoFromWebcam) {//if file, then save each frame
				std::copy(camT,camT+3,lastT);
				tracker.CapKeyFrame(framecnt++, frame, camR, camT, rms, ncc);
			} else if (!needToInit) {
				double diff[3]={camT[0]-lastT[0],camT[1]-lastT[1],camT[2]-lastT[2]};
				double dist = helper::normL2(3,1,diff);
				if(dist>=threshKeydistance && needToCapframe) {
					std::copy(camT,camT+3,lastT);
					tracker.CapKeyFrame(framecnt++, frame, camR, camT, rms, ncc);
				}
			}
		}
	}

	void handle(char key) {
		switch (key) {
		case '0':
			Log::level = Log::LOG_QUIET; break;
		case '1':
			Log::level = Log::LOG_ERROR; break;
		case '2':
			Log::level = Log::LOG_INFO; break;
		case '3':
			Log::level = Log::LOG_DEBUG; break;
		case ' ':
			needToInit=true; break;
		case 'c':
			needToCapframe = !needToCapframe;
			if(needToCapframe) loglni("[Capture Frame] Begin.");
			else loglni("[Capture Frame] End.");
			break;
		}
	}
}; //end of struct KEGprocessor
KEGprocessor processor;

////////////////////////////////////////////////////////////////////////
void usage( int argc, char **argv ) {
	cout<< "[usage] " <<argv[0]<<
		" <url>" //1
		" <K matrix file>" //2
		" <template file>" //3
		" [Target tag ID=0]" //4
		" [AprilTag Family ID=0]" //5
		" [keyframe saving path]"//6
		" [ExperimentMode=0]"<<endl;
	cout<< "Supported TagFamily ID List:\n";
	for(int i=0; i<(int)TagFamilyFactory::TAGTOTAL; ++i) {
		cout<<"\t"<<TagFamilyFactory::SUPPORT_NAME[i]<<" id="<<i<<endl;
	}
	cout<<"Example ImageSource url:\n";
	cout<<"photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/*\n";
	cout<<"camera://0\n";
	cout<<"video:///home/simbaforrest/Videos/Webcam/keg_april.ogv"<<endl;
	cout<<"ExperimentMode:"<<endl;
	cout<<"\t0 - KEG+AprilTag\n"
		  "\t1 - KG +AprilTag\n"
		  "\t2 - KE +AprilTag\n"
		  "\t3 - K  +AprilTag\n"
		  "\t4 -     AprilTag\n"
		  "\t5 -  E +AprilTag"<<endl;
}

int main( int argc, char **argv )
{
	if(argc<4) {
		usage(argc,argv);
		return 1;
	}

	//// ImageSource
	cv::Ptr<ImageSource> is = helper::createImageSource(argv[1]);
	if(is.empty() || is->done()) {
		loglne("[main] createImageSource failed or no valid imagesource!");
		return -1;
	}
	is->pause(false);
	is->reportInfo();
	processor.videoFromWebcam = false;
	if( is->classname() == "ImageSource_Camera" ) {
		processor.videoFromWebcam = true;
	} else {
		processor.needToCapframe = true;
		processor.needToInit = true;
		is->loop(false);
	}

	//// KEGTracker
	loglni("[main] loading K matrix from: "<<argv[2]);
	double K[9];
	std::ifstream kfile(argv[2]);
	for(int i=0; i<9; ++i) kfile >> K[i];
	processor.tracker.loadK(K);
	loglni("[main] K matrix loaded:");
	loglni(helper::PrintMat<>(3,3,K));

	loglni("[main] load template image from: "<<argv[3]);
	processor.tracker.loadTemplate(argv[3]);

	//// TagDetector
	int familyid = 0; //default tag16h5
	if(argc>4) processor.targetid = atoi(argv[4]);
	if(argc>5) familyid = atoi(argv[5]);
	processor.tagFamily = TagFamilyFactory::create(familyid);
	if(processor.tagFamily.empty()) {
		loglne("[main] create TagFamily fail!");
		return -1;
	}
	processor.detector = new TagDetector(processor.tagFamily);
	if(processor.detector.empty()) {
		loglne("[main] create TagDetector fail!");
		return -1;
	}

	// find apriltag on the template, calc iHI
	Mat temp = imread(argv[3]);
	if( processor.findAprilTag(temp, processor.HI, true) ) {
		namedWindow("template");
		imshow("template", temp);
		processor.iHI = processor.HI.inv();
	} else {
		loglne("[main error] detector did not find any apriltag on template image!");
		return -1;
	}

	processor.doCapFrame = argc>6;

	if(argc>7) {
		int experiment = atoi(argv[7]);
		switch(experiment) {
		case 1: //KG + AprilTag
			processor.tracker.refiner.setTermCrit(0, 1); break;
		case 2: //KE + AprilTag
			processor.tracker.doGstep=false; break;
		case 3: //K + AprilTag
			processor.tracker.doGstep=false;
			processor.tracker.refiner.setTermCrit(0, 1); break;
		case 4: //AprilTag
			processor.onlyApril=true; break;
		case 5: //E + AprilTag
			processor.tracker.refiner.setTermCrit(8,4);
			processor.tracker.doKstep = false;
			processor.tracker.doGstep = false; break;
		}
	}

	//// Main Loop
	is->run(processor,-1);

	//// Finish
	if(processor.doCapFrame) {
		loglni("[main] saving keyframes to: "<<argv[6]);
		processor.tracker.SaveKeyFrames(argv[6]);
	}

	loglni("[main] DONE...exit!");
	return 0;
}
