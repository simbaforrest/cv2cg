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
#include <string>
#include <fstream>
#include <sstream>

#include "OpenCVHelper.h"
#include "Log.h"

#include "apriltag/apriltag.hpp"
#include "apriltag/TagFamilyFactory.hpp"

using namespace std;
using namespace cv;
using april::tag::INT64;
using april::tag::TagFamily;
using april::tag::TagFamilyFactory;
using april::tag::TagDetector;
using april::tag::TagDetection;

bool videoFromWebcam;
VideoCapture cap;
Mat frame;
int imgW, imgH;

double videoFrameCnt = -1;
int framecnt = 0;

cv::Ptr<TagFamily> tagFamily;

void ProcessVideo() {
	TagDetector detector(tagFamily);

	string winname = "frame";
	namedWindow(winname);

	helper::PerformanceMeasurer PM;

	for(; videoFromWebcam || framecnt<videoFrameCnt; ++framecnt) {

		if(!videoFromWebcam) {
			cout<<"[process] framecnt="<<framecnt<<endl;
		}

		cap >> frame;
		if(frame.empty()) {
			cout<<"[process] no valid frame, exit!!!"<<endl;
			break;
		}

		//////////////////////////////
		//    process frame
		//////////////////////////////
		vector<TagDetection> detections;
		double opticalCenter[2] = { frame.cols/2.0, frame.rows/2.0 };
		PM.tic();
		detector.process(frame, opticalCenter, detections);
		loglni("[TagDetector] process time = "<<PM.toc()<<" sec.");

		logi(">>> find id: ");
		for(int id=0; id<(int)detections.size(); ++id) {
			TagDetection &dd = detections[id];
			if(dd.hammingDistance>0) continue; //very strict!

			logi("#"<<dd.id<<"|"<<dd.hammingDistance<<" ");
			for(int i=0; i<4; ++i) {
				int j = (i+1)%4;
				Point r1(dd.p[i][0],dd.p[i][1]);
				Point r2(dd.p[j][0],dd.p[j][1]);
				line( frame, r1, r2, helper::CV_RED, 2 );
			}
			cv::putText( frame, helper::num2str(dd.id), cv::Point(dd.cxy[0],dd.cxy[1]), CV_FONT_NORMAL, 1, helper::CV_BLUE, 2 );
		}
		logi(endl);

#if TAG_DEBUG_PERFORMANCE
		static int barH = 30;
		static int textH = 12;
		static vector<cv::Scalar> pclut = helper::pseudocolor(16);
		//draw performance bar
		double total = 0;
		for(int i=0; i<9; ++i) {
			total += detector.steptime[i];
		}
		int lastx=0;
		int lasty=barH+textH;
		for(int i=0; i<9; ++i) {
			double thisx = (detector.steptime[i]/total)*imgW+lastx;
			cv::rectangle(frame, cv::Point(lastx,0), cv::Point(thisx,barH), pclut[i], CV_FILLED);
			lastx = thisx;
			cv::putText(frame, cv::format("step %d: %05.3f ms",i+1,detector.steptime[i]),
				cv::Point(5,lasty-2), CV_FONT_NORMAL, 0.5, pclut[i], 1 );
			lasty += textH;
		}
		cv::putText(frame, cv::format("fps=%4.3lf",1000.0/total), cv::Point(imgW/2,barH), CV_FONT_NORMAL, 1, helper::CV_BLUE, 1);
		loglnd("-------------------------------");
#endif

		imshow(winname, frame);
		char key = (char)waitKey(8);
		switch (key) {
		case 'd':
			detector.segDecimate = !detector.segDecimate;
			loglni("[ProcessVideo] detector.segDecimate="<<detector.segDecimate); break;
		case 27:
		case 'q':
			return;
		}
	}
}

bool InitVideoCapture(int argc, char ** argv)
{
	if(argc==1) {//default
		cap.open(0);
		videoFromWebcam=true;
	} else {
		int len = strlen(argv[1]);
		bool fromDevice=true;
		for(int i=0; i<len; ++i) {
			if( !isdigit(argv[1][i]) ) {
				fromDevice = false;
				break;
			}
		}
		if(fromDevice) {
			int idx = atoi(argv[1]);
			cout<<"[InitVideoCapture] open from device "<<idx<<endl;
			cap.open(idx);
			videoFromWebcam = true;
		} else {
			cout<<"[InitVideoCapture] open from file "<<argv[1]<<endl;
			cap.open(argv[1]);
			videoFromWebcam = false;

			videoFrameCnt=cap.get(CV_CAP_PROP_FRAME_COUNT)-2;
			cout<<"[InitVideoCapture] number of frames: "
				<<videoFrameCnt<<endl;
		}
	}
	return cap.isOpened();
}

void usage( int argc, char **argv ) {
	cout<< "[usage] " <<argv[0]<<" <device number|video file> [TagFamily ID]"<<endl;
	cout<< "Supported TagFamily ID List:\n";
	for(int i=0; i<(int)TagFamilyFactory::TAGTOTAL; ++i) {
		cout<<"\t"<<TagFamilyFactory::SUPPORT_NAME[i]<<" id="<<i<<endl;
	}
	cout<<"default ID: 0"<<endl;
}

#if TAG_DEBUG_PERFORMANCE
Log::Level Log::level = Log::LOG_DEBUG;
#else
Log::Level Log::level = Log::LOG_INFO;
#endif

int main( int argc, char **argv )
{
	if(argc<2) {
		usage(argc,argv);
		return -1;
	}

	if( !InitVideoCapture(argc,argv) ) {
		cout << "[main] Could not initialize capturing...\n";
		return -1;
	}
	cout<<"[main] Video Captured."<<endl;

	if(cap.set(CV_CAP_PROP_FRAME_WIDTH, 640))
		cout<<"[main] video width=640"<<endl;
	if(cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480))
		cout<<"[main] video height=480"<<endl;

	cap >> frame;
	if( frame.empty() ) {
		cout<<"[main] No valid video!"<<endl;
		return -1;
	}
	imgW = frame.cols;
	imgH = frame.rows;

	//// create tagFamily
	int tagid = 0; //default tag16h5
	if(argc>2) tagid = atoi(argv[2]);
	tagFamily = TagFamilyFactory::create(tagid);
	if(tagFamily.empty()) {
		loglne("[main] create TagFamily fail!");
		return -1;
	}

	ProcessVideo();

	cout<<"[main] DONE...exit!"<<endl;
	return 0;
}
