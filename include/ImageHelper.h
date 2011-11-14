#pragma once
/*
 *  Copyright (c) 2010  Chen Feng (cforrest (at) umich.edu)
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

/* ImageHelper.h
   Image Processing related helper functions*/

//standard include
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <time.h>
//opencv include
#include "OpenCVHelper.h"
#include "PerformanceHelper.h"

namespace ImageHelper
{
#define CV_RED		Scalar(255,0,0)
#define CV_GREEN	Scalar(0,255,0)
#define CV_BLUE		Scalar(0,0,255)
#define CV_RG		Scalar(255,255,0)
#define CV_RB		Scalar(255,0,255)
#define CV_GB		Scalar(0,255,255)
#define CV_WHITE	Scalar(255,255,255)
#define CV_BLACK	Scalar(0,0,0)
#define CV_GRAY		Scalar(128,128,128)

// generate pseudocolor look up table
// maxcolors: required max number of colors
inline std::vector<cv::Scalar> pseudocolor(int maxcolors)
{
	//find proper number of bit_per_channle
	//maxcolors = std::min(1<<30,maxcolors);
	int bit_per_channle, bits_total, color_num;
	for(bit_per_channle=1; bit_per_channle<11; ++bit_per_channle) {
		bits_total = 3*bit_per_channle;
		color_num = 1 << bits_total;
		if(color_num>=maxcolors || bit_per_channle==10) {
			--bit_per_channle;
			bits_total = 3*bit_per_channle;
			color_num = 1 << bits_total;
			break;
		}
	}

	std::vector<cv::Scalar> lut(color_num);

	for(int c = 0; c < color_num; c++) {
		int r = 0;
		int g = 0;
		int b = 0;
		for(int k = 0; k < bits_total; ) {
			b = (b << 1) + ((c >> k++) & 1);
			g = (g << 1) + ((c >> k++) & 1);
			r = (r << 1) + ((c >> k++) & 1);
		}
		r = r << (8 - bit_per_channle);
		g = g << (8 - bit_per_channle);
		b = b << (8 - bit_per_channle);

		lut[c][0]=r;
		lut[c][1]=g;
		lut[c][2]=b;
	}

	return lut;
}

using std::string;
using std::cout;
using std::endl;
using std::vector;

struct ImageSource {
public:
	virtual bool done() = 0;
	virtual void get(cv::Mat& dst) = 0;
	virtual string classname() = 0;

	virtual void reportInfo() {}

	inline void pause(bool val=true) { isPause=val; }
	inline void loop(bool val=true) { isLoop=val; }

	ImageSource() { isLoop=true; isPause=true; }
	virtual ~ImageSource() {}
protected:
	bool isLoop;
	bool isPause;
};

//image source from video files
struct ImageSource_Video : public ImageSource {
private:
	cv::VideoCapture cap;
	cv::Mat current;

	int nFrames; //# of frames
	int curF; //0-based index
public:
	ImageSource_Video(string content) {
		cout<<"[ImageSource_Video] open video from: "<<content<<endl;
		if( !cap.open(content) ) {
			cout<<"[ImageSource_Video error] failed to open!"<<endl;
			exit(-1);
		}
		nFrames = (int) cap.get(CV_CAP_PROP_FRAME_COUNT);
		curF = 0;
		if( !cap.read(current) ) {
			cout<<"[ImageSource_Video] capture error @ frame="
				<<curF<<", exit!"<<endl;
			exit(-1);
		}
	}

	inline string classname() {
		return "ImageSource_Video";
	}

	inline bool done() { return curF>=nFrames; }

	inline void get(cv::Mat& dst) {
		current.copyTo(dst);
		//next
		if(isPause) return;
		++curF;
		if(curF>=nFrames) {
			if(!isLoop) return;
			curF = 0;
			cap.set(CV_CAP_PROP_POS_FRAMES, 0);
		}
		if(!cap.read(current)) {
			cout<<"[ImageSource_Video] capture error @ frame="
				<<curF<<", remove the rest."<<endl;
			nFrames = curF;
			curF = 0;
			cap.set(CV_CAP_PROP_POS_FRAMES, 0);
			cap.read(current);
		}
	}

	~ImageSource_Video() {
		cap.release();
		cout<<"[ImageSource_Video] video closed."<<endl;
	}

	inline void reportInfo() {
		cout<<"[ImageSource_Video] INFO:"<<endl;
		cout<<">>> #frame = "<<nFrames<<endl;
		cout<<">>> current frame @ "<<curF<<endl;
		cout<<">>> frame size = "
			<<cap.get(CV_CAP_PROP_FRAME_WIDTH)<<"x"
			<<cap.get(CV_CAP_PROP_FRAME_HEIGHT)<<endl;
	}
};

//image source from camera
struct ImageSource_Camera : public ImageSource {
private:
	cv::VideoCapture cap;
	cv::Mat current;
	bool isDone;
public:
	ImageSource_Camera(string content) {
		cout<<"[ImageSource_Camera] open from device: "<<content<<endl;
		if( !cap.open(atoi(content.c_str())) ) {
			cout<<"[ImageSource_Camera error] failed to open!"<<endl;
			exit(-1);
		}

		//try to force camera to be 640x480
		cap.set(CV_CAP_PROP_FRAME_WIDTH,  640);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

		isDone = false;
		isPause = false;

		next();
	}

	inline string classname() {
		return "ImageSource_Camera";
	}

	inline void setDone(bool val=true) { isDone = val; }

	inline bool done() { return isDone; }

	inline void next() {
		if( !cap.read(current) ) {
			cout<<"[ImageSource_Camera] capture error, exit!"<<endl;
			exit(-1);
		}
	}

	inline void get(cv::Mat& dst) {
		current.copyTo(dst);
		if(isPause) return;
		next();
	}

	~ImageSource_Camera() {
		cap.release();
		cout<<"[ImageSource_Camera] camera closed."<<endl;
	}

	inline void reportInfo() {
		cout<<"[ImageSource_Camera] INFO:"<<endl;
		cout<<">>> frame size = "
			<<cap.get(CV_CAP_PROP_FRAME_WIDTH)<<"x"
			<<cap.get(CV_CAP_PROP_FRAME_HEIGHT)<<endl;
	}
};

struct ImageSource_Photo : public ImageSource {
private:
	vector<string> imnames;
	cv::Mat current;

	int curF; //0-based index
public:
	ImageSource_Photo(string content) {
		cout<<"[ImageSource_Photo] open images from: "<<content<<endl;
		string cmd = string("ls -1 ")+content;
#ifdef _WIN32
		FILE* fptr = _popen(cmd.c_str(), "r");
#else
		FILE* fptr = popen(cmd.c_str(), "r");
#endif
		char buf[1024]={0};
		while( fgets(buf,1024,fptr)!=0 ) {
			int len = strlen(buf);
			if(len>1)
			imnames.push_back(string(buf,buf+len-1));
			memset(buf,0,sizeof(char)*1024);
			cout<<imnames.back()<<endl;
		}
#ifdef _WIN32
		_pclose(fptr);
#else
		pclose(fptr);
#endif
		curF = -1;
		next();
	}

	inline string classname() {
		return "ImageSource_Photo";
	}

	inline bool done() { return curF>=(int)imnames.size(); }

	inline void next() {
		++curF;
		do {
			if(curF>=(int)imnames.size()) {
				if(!isLoop) return;
				curF = 0;
			}
			current = cv::imread(imnames[curF]);
			if(!current.empty()) return;
			cout<<"[ImageSource_Photo] imread error @ photo\n"
				<<imnames[curF]<<endl;
			imnames.erase(imnames.begin()+curF);
		} while(current.empty());
	}

	inline void get(cv::Mat& dst) {
		current.copyTo(dst);
		if(isPause) return;
		next();
	}

	~ImageSource_Photo() {
		cout<<"[ImageSource_Photo] photo album closed."<<endl;
	}

	inline void reportInfo() {
		cout<<"[ImageSource_Photo] INFO:"<<endl;
		cout<<">>> #image = "<<imnames.size()<<endl;
		cout<<">>> current image @ "<<curF<<endl;
	}
};

//url format example:
//	video:///home/simbaforrest/Videos/Webcam/keg_april.ogv
//	camera://0
//	photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/img00000.jpg
//	photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/img*.jpg
inline ImageSource* createImageSource(string url) {
	size_t pos = url.find("://");
	if(pos == string::npos) {
		cout<<"[ImageSource error] invalid image source url!"<<endl;
		return NULL;
	}
	string header(url, 0, pos);
	string content(url, pos+3, url.length()-pos-3);
	if(header=="video") {
		return new ImageSource_Video(content);
	} else if(header=="camera") {
		return new ImageSource_Camera(content);
	} else if(header=="photo") {
		return new ImageSource_Photo(content);
	} else {
		cout<<"[ImageSource error] unknown url header: "<<header<<endl;
	}
	return NULL;
}

/*//////////////////////////////////////////////////////////////////////
//To properly use the following functions, implement these two functions:
namespace ImageHelper {
void ProcessOneFrame(cv::Mat& frame) {
}

void KeyResponse(char key) {
}
}
*/

inline void ProcessOneFrame(cv::Mat& frame);
inline void KeyResponse(char key);

inline void LoopInImageSource(cv::Ptr<ImageSource> is, double idealfps = 20, bool verbose=false) {
	static bool pa = false;
	static bool lo = false;
	cv::namedWindow("frame");

	PerformanceHelper::PerformanceMeasurer PM;
	PM.scale = 1000;
	double lastdur = 0;
	double idealdur = 1000.0/idealfps;
	while(!is->done()) {
		PM.tic();
		cv::Mat frame;
		is->get(frame);

		ProcessOneFrame(frame);
		cv::imshow("frame",frame);

		lastdur = PM.toc();
		if(verbose) cout<<"[loop] dur = "<<lastdur<<endl;
		double waitdur = idealdur-lastdur;
		char key = cv::waitKey(waitdur>0?waitdur:8);
		KeyResponse(key);
		switch(key) {
		case 'v':
			verbose = !verbose; break;
		case 'p':
			is->pause(pa); if(verbose) cout<<"pause="<<pa<<endl;
			pa = !pa;
			break;
		case 'l':
			is->loop(lo); if(verbose) cout<<"loop="<<lo<<endl;
			lo = !lo;
			break;
		case '=':
			++idealfps;
			idealdur = 1000/idealfps;
			break;
		case '-':
			--idealfps; if(idealfps<=0) idealfps=1;
			idealdur = 1000/idealfps;
			break;
		case 27:
		case 'q':
			return;
		}
	}
}

}//ImageHelper
