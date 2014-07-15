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

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "StringHelper.h"
#include "DirHelper.h"
#include "IOHelper.h"
#include "LogHelper.h"

#include "OpenCVHeaders.h"
#include "PerformanceHelper.h"

#ifdef USE_FLYCAP
#ifdef _WIN32
	#pragma warning (disable:4819)
#endif
#include "flycap2opencv.hpp"
#endif//USE_FLYCAP

namespace ImageHelper
{
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

	virtual std::string classname() = 0;

	/**
	test is this the TestClass
	*/
	template<typename TestClass>
	bool isClass() {
		return 0!=dynamic_cast<TestClass*>(this);
	}

	/**
	return the directory of the source, WITHOUT any separator '/' or '\\'
	*/
	inline std::string getSourceDir() const {
		return sourceDir;
	}

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
		cv::namedWindow("frame",cv::WINDOW_NORMAL);
		cvMoveWindow("frame",10,10);
		bool openFromWebcam = (this->classname() == "ImageSource_Camera"
#ifdef USE_FLYCAP
			|| this->classname() == "ImageSource_PGR"
#endif
			);
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
			if(verbose) logli("[ImageSource::run] process duration = "<<lastdur<<" ms");
			if(step) { step=false; pause(true); }
			double waitdur = openFromWebcam?8:(std::max)(idealdur-lastdur, 8.0);
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
				pause(!isPause); if(verbose) logli("pause="<<isPause);
				break;
			case 'l':
				loop(!isLoop); if(verbose) logli("loop="<<isLoop);
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
				logli("v: verbose\n"
					"s: step one frame forward\n"
					"p: pause\n"
					"l: loop\n"
					"=: increase fps\n"
					"-: decrease fps\n"
					"h: display help information\n"
					"q,Esc: quit");
				break;
			case 27:
			case 'q':
				return;
			}
		}
	}

	static ImageSource* create(std::string url);

	inline void pause(bool val=true) { isPause=val; }
	inline bool getPause() const { return isPause; }
	inline void loop(bool val=true) { isLoop=val; }
	inline bool getLoop() const { return isLoop; }

	ImageSource() : sourceDir(".") { isLoop=true; isPause=true; }
	virtual ~ImageSource() {}
protected:
	bool isLoop;
	bool isPause;
	std::string sourceDir; //directory of the source
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
	ImageSource_Video(std::string content) {
		logli("[ImageSource_Video] open video from: "<<content);
		if( !cap.open(content) ) {
			logle("[ImageSource_Video error] failed to open!");
			exit(-1);
		}
		nFrames = (int) cap.get(CV_CAP_PROP_FRAME_COUNT);
		curF = 0;
		if( !cap.read(current) ) {
			logle("[ImageSource_Video] capture error @ frame="
				<<curF<<", exit!");
			exit(-1);
		}
		this->sourceDir = DirHelper::getFileDir(content);
	}

	inline std::string classname() {
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
			logli("[ImageSource_Video] capture error @ frame="
				<<curF<<", remove the rest.");
			nFrames = curF;
			curF = 0;
			cap.set(CV_CAP_PROP_POS_FRAMES, 0);
			cap.read(current);
		}
	}

	~ImageSource_Video() {
		cap.release();
		logli("[ImageSource_Video] video closed.");
	}

	inline void reportInfo() {
		logli("[ImageSource_Video] INFO:");
		logli(">>> #frame = "<<nFrames);
		logli(">>> current frame @ "<<curF);
		logli(">>> frame size = "
			<<cap.get(CV_CAP_PROP_FRAME_WIDTH)<<"x"
			<<cap.get(CV_CAP_PROP_FRAME_HEIGHT));
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
	ImageSource_Camera(std::string content) {
		std::vector<std::string> contentParts=StringHelper::split(content, '?');
		logli("[ImageSource_Camera] open from device: "<<contentParts[0]);
		if( !cap.open(atoi(contentParts[0].c_str())) ) {
			logle("[ImageSource_Camera error] failed to open!");
			exit(-1);
		}

		//process parameters
		int ideal_width=640, ideal_height=480;
		int framerate=30;
		for(int i=1; i<(int)contentParts.size(); ++i) {
			std::vector<std::string> par=StringHelper::split(contentParts[i], '=');
			if(par.size()!=2) {
				logli("[ImageSource_Camera] ignore "<<contentParts[i]);
				continue;
			}
			if(par[0].compare("w")==0 || par[0].compare("W")==0) {
				ideal_width=atoi(par[1].c_str());
			} else if(par[0].compare("h")==0 || par[0].compare("H")==0) {
				ideal_height=atoi(par[1].c_str());
			} else if(par[0].compare("f")==0 || par[0].compare("F")==0) {
				framerate=atoi(par[1].c_str());
			}
		}

		bool success=cap.set(CV_CAP_PROP_FRAME_WIDTH,  ideal_width);
		success|=cap.set(CV_CAP_PROP_FRAME_HEIGHT, ideal_height);
		if(!success) {//try to force camera to be 640x480
			cap.set(CV_CAP_PROP_FRAME_WIDTH,  640);
			cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
		}
		cap.set(CV_CAP_PROP_FPS, framerate);

		isDone = false;
		isPause = false;
	}

	inline std::string classname() {
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
			logle("[ImageSource_Camera] capture error, exit!");
			exit(-1);
		}
		current.release();
	}

	~ImageSource_Camera() {
		cap.release();
		logli("[ImageSource_Camera] camera closed.");
	}

	inline void reportInfo() {
		logli("[ImageSource_Camera] INFO:");
		logli(">>> frame size = "
			<<cap.get(CV_CAP_PROP_FRAME_WIDTH)<<"x"
			<<cap.get(CV_CAP_PROP_FRAME_HEIGHT));
		logli(">>> frame rate = "
			<<cap.get(CV_CAP_PROP_FPS));
	}
};

/**
\class ImageSource_Photo
\brief image source from set of images
*/
struct ImageSource_Photo : public ImageSource {
protected:
	std::vector<std::string> imnames;
	cv::Mat current;

	int curF; //0-based index
	ImageSource_Photo() {} //used for child class to skip base constructor
public:
	ImageSource_Photo(std::string content) {
		logli("[ImageSource_Photo] open images from: "<<content);
#ifdef _WIN32
		std::string cmd = std::string("dir /B ")+content;
		std::string dirstr = DirHelper::getFileDir(content);
		FILE* fptr = _popen(cmd.c_str(), "r");

		this->sourceDir = dirstr;
#else
		std::string cmd = std::string("ls -v1 ")+content;
		FILE* fptr = popen(cmd.c_str(), "r");
#endif

		if(fptr==0) {
			logle("[ImageSource_Photo] exit since invalid cmd:"<<cmd);
			exit(-1);
		}

		char buf[1024]={0};
		while( fgets(buf,1024,fptr)!=0 ) {
			int len = strlen(buf);
			if(len>1) {
#ifdef _WIN32
				imnames.push_back(dirstr+std::string(buf,buf+len-1));
#else
				imnames.push_back(std::string(buf,buf+len-1));
				static bool foundDir=false;
				if(!foundDir) {
					this->sourceDir = DirHelper::getFileDir(imnames.back());
					foundDir=true;
				}
#endif
				memset(buf,0,sizeof(char)*1024);
				logli(imnames.back());
			}
		}
#ifdef _WIN32
		_pclose(fptr);
#else
		pclose(fptr);
#endif
		if(imnames.size()<=0) {
			curF = (int)imnames.size(); //set done since no image available
			logle("[ImageSource_Photo] warning: no image available!");
			return;
		}

		this->loop(true);
		if(imnames.size()<10) {
			this->pause(true);
		} else {
			this->pause(false);
		}
		curF = -1;
		next();
		if(current.empty()) {
			curF = (int)imnames.size(); //set done since no image available
			logle("[ImageSource_Photo] warning: no image available!");
		}
	}

	inline std::string classname() {
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
			logle("[ImageSource_Photo] imread error @ photo\n"
				<<imnames[curF]);
			imnames.erase(imnames.begin()+curF);
		} while(current.empty() && imnames.size()>0);
	}

	inline void get(cv::Mat& dst) {
		current.copyTo(dst);
		if(isPause) return;
		next();
	}

	~ImageSource_Photo() {
		logli("[ImageSource_Photo] photo album closed.");
	}

	inline void reportInfo() {
		logli("[ImageSource_Photo] INFO:");
		logli(">>> #image = "<<imnames.size());
		logli(">>> current image @ "<<curF);
	}
};

/**
\class ImageSource_List
\brief image source from set of images
*/
struct ImageSource_List : public ImageSource_Photo {
public:
	ImageSource_List(std::string content) : ImageSource_Photo() {
		logli("[ImageSource_List] open images from: "<<content);
		std::ifstream is(content.c_str());
		if(!is.is_open()) {
			logle("[ImageSource_List error] can't open file: "<<content);
			exit(-1);
		}
		std::string line;
		while( IOHelper::readValidLine(is, line) ) {
			imnames.push_back(line);
			logli(imnames.back());
		}
		is.close();

		if(imnames.size()<=0) {
			curF = (int)imnames.size(); //set done since no image available
			logle("[ImageSource_List] warning: no image available!");
			return;
		}

		this->loop(true);
		if(imnames.size()<10) {
			this->pause(true);
		} else {
			this->pause(false);
		}
		curF = -1;
		next();
		if(current.empty()) {
			curF = (int)imnames.size(); //set done since no image available
			logle("[ImageSource_List] warning: no image available!");
		}

		this->sourceDir = DirHelper::getFileDir(content);
	}

	inline std::string classname() {
		return "ImageSource_List";
	}
};

#ifdef USE_FLYCAP
/**
\class ImageSource_PGR
\brief image source from point greycamera
*/
struct ImageSource_PGR : public ImageSource {
private:
	FlyCap2OpenCV cap;
	cv::Mat current;
	bool isDone;
public:
	ImageSource_PGR(std::string content) {
		std::vector<std::string> contentParts=StringHelper::split(content, '?');
		logli("[ImageSource_PGR] open from device: "<<contentParts[0]);
		if( !cap.init(atoi(contentParts[0].c_str())) ) {
			logle("[ImageSource_PGR error] failed to open!");
			exit(-1);
		}

		//process parameters
		int vi=(int)FlyCapture2::VIDEOMODE_640x480Y8;
		int fi=(int)FlyCapture2::FRAMERATE_30;
		int colormode=2; //0:mono, 1:rgb, 2:bgr
		for(int i=1; i<(int)contentParts.size(); ++i) {
			std::vector<std::string> par=StringHelper::split(contentParts[i], '=');
			if(par.size()!=2) {
				logli("[ImageSource_PGR] ignore "<<contentParts[i]);
				continue;
			}
			if(par[0].compare("v")==0 || par[0].compare("V")==0) {
				vi=atoi(par[1].c_str());
			} else if(par[0].compare("f")==0 || par[0].compare("F")==0) {
				fi=atoi(par[1].c_str());
			} else if(par[0].compare("c")==0 || par[0].compare("C")==0) {
				colormode=atoi(par[1].c_str());
			}
		}

		//try to force camera to be 640x480
		cap.error=cap.cam.SetVideoModeAndFrameRate((FlyCapture2::VideoMode)vi,
					(FlyCapture2::FrameRate)fi);
		if(cap.error!=FlyCapture2::PGRERROR_OK) {
			logle("[ImageSource_PGR error] can not set camera mode to vi="
				<<vi<<" and fi="<<fi);
			exit(-1);
		}

		cap.initBuffer(colormode);

		isDone = false;
		isPause = false;
	}

	inline std::string classname() {
		return "ImageSource_PGR";
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
			logle("[ImageSource_PGR] capture error, exit!");
			exit(-1);
		}
		current.release();
	}

	~ImageSource_PGR() {
		logli("[ImageSource_PGR] camera closed.");
	}

	inline void reportInfo() {
		FlyCapture2::CameraInfo info;
		cap.error=cap.cam.GetCameraInfo(&info);
		if(cap.error!=FlyCapture2::PGRERROR_OK) {
			cap.error.PrintErrorTrace();
			return;
		} else {
			logli(">>>serialNumber: "<<info.serialNumber);
			//switch(info.interfaceType) {
			//	case FlyCapture2::INTERFACE_USB2:
			//		logli(">>>interfaceType: USB2"); break;
			//}
			logli(">>>isColorCamera: "<<(info.isColorCamera?"Yes":"No"));
			logli(">>>modelName: "<<info.modelName);
			logli(">>>vendorName: "<<info.vendorName);
			logli(">>>sensorInfo: "<<info.sensorInfo);
			logli(">>>sensorResolution: "<<info.sensorResolution);
		}
	}
};
#endif//USE_FLYCAP

inline ImageSource* ImageSource::create(std::string url) {
	size_t pos = url.find("://");
	if(pos == std::string::npos) {
		logle("[ImageSource error] invalid image source url!");
		return NULL;
	}
	std::string header(url, 0, pos);
	std::string content(url, pos+3, url.length()-pos-3);
	if(header=="video") {
		return new ImageSource_Video(content);
	} else if(header=="camera") {
		return new ImageSource_Camera(content);
	} else if(header=="photo") {
		return new ImageSource_Photo(content);
	} else if(header=="list") {
		return new ImageSource_List(content);
#ifdef USE_FLYCAP
	} else if(header=="pgr") {
		return new ImageSource_PGR(content);
#endif//USE_FLYCAP
	} else {
		logle("[ImageSource error] unknown url header: "<<header);
	}
	return NULL;
}

/**
helper function for create an ImageSource from url
url format example:
video:///home/simbaforrest/Videos/Webcam/keg_april.ogv
camera://0?w=640?h=480?f=60
photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/img00000.jpg
photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/img*.jpg
list:///home/simbaforrest/Videos/Webcam/seq_UMshort/list.txt
pgr://0?v=5?f=4 (if USE_FLYCAP)

@param url url for image source
@return created ImageSource
*/
inline ImageSource* createImageSource(std::string url) {
	return ImageSource::create(url);
}

}//ImageHelper
