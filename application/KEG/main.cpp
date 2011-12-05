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

#define ESM_DEBUG 1
#define KEG_DEBUG 1

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
	double threshKeydistance;
	int framecnt;
	vector<keg::KeyFrame> keyframes;

	cv::Ptr<TagFamily> tagFamily;
	cv::Ptr<TagDetector> detector;

/////// Constructor
	KEGprocessor() {
		needToInit = true;
		doCapFrame = true;
		needToCapframe = false;
		videoFromWebcam = false;
		threshKeydistance = 200;
		framecnt = 0;
	}

/////// Override
	void operator()(cv::Mat& frame) {
		if(frame.empty()) return;
#if KEG_DEBUG
		static helper::PerformanceMeasurer PM(1000);
		PM.tic();
#endif

		cvtColor(frame, gray, CV_BGR2GRAY);
		double rms=-1, ncc=-1;
		double camR[3][3]={{1,0,0},{0,1,0},{0,0,1}},camT[3]={0};

		if( needToInit ) {
			loglni("[KEGprocessor] INITing...");
			vector<Mat> tmpH;
			vector<int> tmpid;
			tracker.recognize(*detector, gray, tmpH, tmpid);
			if((int)tmpid.size() == 1) {
				int id=tmpid.front();
				if(id<(int)tracker.tdata.size() && id>=0) {
					Mat initH = tmpH.front() * tracker.tdata[id].iHI;
					needToInit = !tracker.init(gray, initH, rms, ncc, id);
					if(!needToInit) {
						loglni("[KEGprocessor] ...INITed");
						tracker.draw3D(frame);
					}
				}
			}
		} else {
			needToInit=!tracker(gray, rms, ncc, &frame);
		}

		if(!needToInit) {
			cv::putText( frame, string("Currently around location ")+helper::num2str(tracker.curT),
					cv::Point(10,30), CV_FONT_NORMAL,
					1, helper::CV_GREEN, 2 );
		}

#if KEG_DEBUG
		double dur = PM.toc();
		cout<<"process duration="<<dur<<endl;
#endif

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
		if (doCapFrame && !videoFromWebcam) {//if file, then save each frame
			keg::CapKeyFrame(keyframes,
				tracker.curT, Mat(), camR, camT, rms, ncc
#if KEG_DEBUG
				, dur
#endif
			);
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
		" [AprilTag Family ID=0]" //4
		" [keyframe saving path]"//5
		<<endl;
	cout<< "Supported TagFamily ID List:\n";
	for(int i=0; i<(int)TagFamilyFactory::TAGTOTAL; ++i) {
		cout<<"\t"<<TagFamilyFactory::SUPPORT_NAME[i]<<" id="<<i<<endl;
	}
	cout<<"Example ImageSource url:\n";
	cout<<"photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/*\n";
	cout<<"camera://0\n";
	cout<<"video:///home/simbaforrest/Videos/Webcam/keg_april.ogv"<<endl;
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

	//// TagDetector
	int familyid = 0; //default tag16h5
	if(argc>4) familyid = atoi(argv[4]);
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

	loglni("[main] load template image from: "<<argv[3]);
	std::ifstream tin(argv[3]);
	string line;
	vector<string> tlist;
	while(helper::readValidLine(tin, line)) {
		tlist.push_back(line);
	}
	processor.tracker.loadTemplateList(*(processor.detector), tlist);

	processor.doCapFrame = argc>5;

	//// Main Loop
	is->run(processor,-1);

	//// Finish
	if(processor.doCapFrame) {
		loglni("[main] saving keyframes to: "<<argv[5]);
		keg::SaveKeyFrames(processor.keyframes, argv[5]);
	}

	loglni("[main] DONE...exit!");
	return 0;
}
