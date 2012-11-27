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

using cv::Scalar;
using cv::Mat;
using std::vector;
using cv::Point2f;
using cv::Point;

/**
draw a homography (a quad) given the four corners to be mapped by Homography

@param[in,out] image target image to be drawed on
@param[in] Homo homography matrix, <3x3>
@param[in] crns 4 corners in world coordinate system
*/
inline void drawHomography(Mat& image, const Mat& Homo, double const crns[4][2]) {
	static cv::Scalar homocolors[] = {
		CV_BLACK,
		CV_GREEN,
		CV_BLUE,
		CV_RED  };
	const cv::Mat_<double>& mH = Homo;
	vector<Point2f> corners(4);
	for(int i = 0; i < 4; i++ ) {
		double ptx = crns[i][0], pty=crns[i][1];
		double w = 1./(mH(2,0)*ptx + mH(2,1)*pty + mH(2,2));
		corners[i] =
		    Point2f((float)((mH(0,0)*ptx + mH(0,1)*pty + mH(0,2))*w),
		            (float)((mH(1,0)*ptx + mH(1,1)*pty + mH(1,2))*w));
	}
	for(int i = 0; i < 4; ++i) {
		const Point& r1 = corners[i%4];
		const Point& r2 = corners[(i+1)%4];
		line( image, r1, r2, homocolors[i], 2 );
	}
	line(image, corners[0], corners[2], CV_GB, 2);
	line(image, corners[1], corners[3], CV_GB, 2);
}

/**
draw a 3D pyramid on image, projection matrix \f$P=K[R,T]\f$

@param[in,out] image target image to draw
@param[in] K calibration matrix, <3x3>
@param[in] R rotation matrix, <3x3>
@param[in] T translation vector, <3x1>
@param[in] crns 8 corners of the pyramid, 0~3 bottom, 4~7 top, in world coordinate system
*/
inline void drawPyramid(Mat& image,
		double const K[9], double const R[9], double const T[3],
		double const crns[8][3] ) {
	static cv::Scalar linecolors[] = {
		CV_BLACK,
		CV_GREEN,
		CV_BLUE,
		CV_RED  };
	static int lineidx[12][2] = {
		{0,1},{1,2},{2,3},{3,0},
		{4,5},{5,6},{6,7},{7,4},
		{0,4},{1,5},{2,6},{3,7} };
	double P[12];
	CameraHelper::compose(K,R,T,P,false);
	double p[8][2];
	for(int i=0; i<8; ++i) {
		CameraHelper::project(P,crns[i],p[i]);
	}
	for(int i=0; i<3; ++i) {
		for(int j=i*4; j<4+i*4; ++j) {
			int s=lineidx[j][0], e=lineidx[j][1];
			Point r1(p[s][0],p[s][1]);
			Point r2(p[e][0],p[e][1]);
			line( image, r1, r2, linecolors[j%4], 2 );
		}
	}
}

/**
generate pseudocolor look up table, algorithm from Szeliski's book

@param maxcolors required max number of colors
@return pseudocolor look up table
*/
//inline std::vector<cv::Scalar> pseudocolor(int maxcolors)
//{
//	//find proper number of bit_per_channle
//	//maxcolors = std::min(1<<30,maxcolors);
//	int bit_per_channle, bits_total, color_num;
//	for(bit_per_channle=1; bit_per_channle<11; ++bit_per_channle) {
//		bits_total = 3*bit_per_channle;
//		color_num = 1 << bits_total;
//		if(color_num>=maxcolors || bit_per_channle==10) {
//			--bit_per_channle;
//			bits_total = 3*bit_per_channle;
//			color_num = 1 << bits_total;
//			break;
//		}
//	}
//
//	std::vector<cv::Scalar> lut(color_num);
//
//	for(int c = 0; c < color_num; c++) {
//		int r = 0;
//		int g = 0;
//		int b = 0;
//		for(int k = 0; k < bits_total; ) {
//			b = (b << 1) + ((c >> k++) & 1);
//			g = (g << 1) + ((c >> k++) & 1);
//			r = (r << 1) + ((c >> k++) & 1);
//		}
//		r = r << (8 - bit_per_channle);
//		g = g << (8 - bit_per_channle);
//		b = b << (8 - bit_per_channle);
//
//		lut[c][0]=r;
//		lut[c][1]=g;
//		lut[c][2]=b;
//	}
//
//	return lut;
//}

/**
generate pseudocolor look up table, using opencv's own random version

@param ncolors required number of colors
@return pseudocolor look up table
*/
//inline std::vector<Scalar> pseudocolor(int ncolors) {
//	using namespace cv;
//	std::vector<Scalar> ret;
//	theRNG() = (uint64)time(0);
//	generateColors( ret, ncolors );
//	return ret;
//}

using std::string;
using std::cout;
using std::endl;
using std::vector;

/**
\class ImageSource ImageHelper.h "ImageHelper.h"
\brief Interface for easier access to different image sources such as videos,
       set of images or camera
*/
struct ImageSource {
public:
	/**
	whether finished
	
	@return true if finished
	*/
	virtual bool done() = 0;

	/**
	fill dst with current image
	
	@param dst mat to be filled with current image
	*/
	virtual void get(cv::Mat& dst) = 0;

	virtual string classname() = 0;

	/**
	report information of the image source to console
	*/
	virtual void reportInfo() {}

	/**
	\class ImageSource::Processor
	\brief Callback for process one frame of image and handle key input
	*/
	struct Processor {
		virtual void operator()(cv::Mat& frame) = 0; //main process
		virtual void handle(char key) = 0; //handle user input
	};

	/**
	run processor on image source
	
	@param processor given processor to process one frame of image
	@param idealfps ideal fps for ImageSource_Photo/ImageSource_Video
	@param verbose whether to output to console
	@param pa init pause or not
	@param lo init loop or not
	*/
	virtual void run(Processor& processor,
		double idealfps = 20, bool verbose=false,
		bool pa=false, bool lo=false) {
		pause(pa);
		loop(lo);
		cv::namedWindow("frame");
		cvMoveWindow("frame",10,10);
		bool openFromWebcam = (this->classname() == "ImageSource_Camera");
		PerformanceHelper::PerformanceMeasurer PM;
		PM.scale = 1000;
		double lastdur = 0;
		double idealdur = 1000.0/idealfps;
		bool step=false;
		while(!this->done()) {
			PM.tic();
			cv::Mat frame;
			this->get(frame);

			processor(frame);
			cv::imshow("frame",frame);

			lastdur = PM.toc();
			if(verbose) cout<<"[run] process duration = "<<lastdur<<endl;
			if(step) { step=false; pause(true); }
			double waitdur = openFromWebcam?8:std::max(idealdur-lastdur, 8.0);
			char key = cv::waitKey(waitdur);
			processor.handle(key);
			switch(key) {
			case 'v':
				verbose = !verbose; break;
			case 's':
				if(!step) {
					step=true;
					pause(false);
				}
				break;
			case 'p':
				pause(!isPause); if(verbose) cout<<"pause="<<isPause<<endl;
				break;
			case 'l':
				loop(!isLoop); if(verbose) cout<<"loop="<<isLoop<<endl;
				break;
			case '=':
				++idealfps;
				idealdur = 1000/idealfps;
				break;
			case '-':
				--idealfps; if(idealfps<=0) idealfps=1;
				idealdur = 1000/idealfps;
				break;
			case 'h':
				cout<<"v: verbose\n"
					"s: step one frame forward\n"
					"p: pause\n"
					"l: loop\n"
					"=: increase fps\n"
					"-: decrease fps\n"
					"h: display help information"<<endl;
				break;
			case 27:
			case 'q':
				return;
			}
		}
	}

	inline void pause(bool val=true) { isPause=val; }
	inline void loop(bool val=true) { isLoop=val; }

	ImageSource() { isLoop=true; isPause=true; }
	virtual ~ImageSource() {}
protected:
	bool isLoop;
	bool isPause;
};

/**
\class ImageSource_Video
\brief image source from video files
*/
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

/**
\class ImageSource_Camera
\brief image source from camera
*/
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
	}

	inline string classname() {
		return "ImageSource_Camera";
	}

	inline void setDone(bool val=true) { isDone = val; }

	inline bool done() { return isDone; }

	inline void get(cv::Mat& dst) {
		if(isPause) {
			if( current.empty() && cap.read(dst) ) {
				dst.copyTo(current);
			} else 
				current.copyTo(dst);
			return;
		} else if ( !cap.read(dst) ) {
			cout<<"[ImageSource_Camera] capture error, exit!"<<endl;
			exit(-1);
		}
		current.release();
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

/**
\class ImageSource_Photo
\brief image source from set of images
*/
struct ImageSource_Photo : public ImageSource {
protected:
	vector<string> imnames;
	cv::Mat current;

	int curF; //0-based index
public:
	ImageSource_Photo(string content, bool skip=false) {
		if(skip) return;
		cout<<"[ImageSource_Photo] open images from: "<<content<<endl;
#ifdef _WIN32
		string cmd = string("dir /B ")+content;
		string dirstr = DirHelper::getFileDir(content);
		FILE* fptr = _popen(cmd.c_str(), "r");
#else
		string cmd = string("ls -v1 ")+content;
		FILE* fptr = popen(cmd.c_str(), "r");
#endif
		char buf[1024]={0};
		while( fgets(buf,1024,fptr)!=0 ) {
			int len = strlen(buf);
			if(len>1) {
#ifdef _WIN32
				imnames.push_back(dirstr+string(buf,buf+len-1));
#else
				imnames.push_back(string(buf,buf+len-1));
#endif
				memset(buf,0,sizeof(char)*1024);
				cout<<imnames.back()<<endl;
			}
		}
#ifdef _WIN32
		_pclose(fptr);
#else
		pclose(fptr);
#endif
		this->loop(true);
		if(imnames.size()<10) {
			this->pause(true);
		} else {
			this->pause(false);
		}
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

/**
\class ImageSource_List
\brief image source from set of images
*/
struct ImageSource_List : public ImageSource_Photo {
public:
	ImageSource_List(string content) : ImageSource_Photo("", true) {
		cout<<"[ImageSource_List] open images from: "<<content<<endl;
		std::ifstream is(content.c_str());
		if(!is.is_open()) {
			cout<<"[ImageSource_List error] can't open file: "<<content<<endl;
			exit(-1);
		}
		std::string line;
		while( IOHelper::readValidLine(is, line) ) {
			imnames.push_back(line);
			cout<<imnames.back()<<endl;
		}
		is.close();

		this->loop(true);
		if(imnames.size()<10) {
			this->pause(true);
		} else {
			this->pause(false);
		}
		curF = -1;
		next();
	}

	inline string classname() {
		return "ImageSource_List";
	}
};

/**
helper function for create an ImageSource from url
url format example:
video:///home/simbaforrest/Videos/Webcam/keg_april.ogv
camera://0
photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/img00000.jpg
photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/img*.jpg
list:///home/simbaforrest/Videos/Webcam/seq_UMshort/list.txt

@param url url for image source
@return created ImageSource
*/
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
	} else if(header=="list") {
		return new ImageSource_List(content);
	} else {
		cout<<"[ImageSource error] unknown url header: "<<header<<endl;
	}
	return NULL;
}

}//ImageHelper
