/*
 *  Copyright (c) 2010  Chen Feng (cforrest (at) umich.edu)
 *    and the University of Michigan
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

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

		for(int id=0; id<(int)detections.size(); ++id) {
			TagDetection &dd = detections[id];
			for(int i=0; i<4; ++i) {
				int j = (i+1)%4;
				Point r1(dd.p[i][0],dd.p[i][1]);
				Point r2(dd.p[j][0],dd.p[j][1]);
				line( frame, r1, r2, helper::CV_RED, 2 );
			}
			cv::putText( frame, helper::num2str(dd.id), cv::Point(dd.cxy[0],dd.cxy[1]), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, helper::CV_BLUE, 2 );
		}

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

Log::Level Log::level = Log::INFO;

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
