#pragma once
#include "opencv2/opencv.hpp"
#include "FlyCapture2.h"

const std::string FlyCap2OpenCV_videoModeName[]={
	"160x120YUV444",
	"320x240YUV422",
	"640x480YUV411",
	"640x480YUV422",
	"640x480RGB",
	"640x480Y8",
	"640x480Y16",
	"800x600YUV422",
	"800x600RGB",
	"800x600Y8",
	"800x600Y16",
	"1024x768YUV422",
	"1024x768RGB",
	"1024x768Y8",
	"1024x768Y16",
	"1280x960YUV422",
	"1280x960RGB",
	"1280x960Y8",
	"1280x960Y16",
	"1600x1200YUV422",
	"1600x1200RGB",
	"1600x1200Y8",
	"1600x1200Y16",
	"FORMAT7"
};

const std::string FlyCap2OpenCV_frameRateValue[]={
	"1.875Hz",
	"3.75Hz",
	"7.5Hz",
	"15Hz",
	"30Hz",
	"60Hz",
	"120Hz",
	"240Hz",
	"FORMAT7"
};

struct FlyCap2OpenCV {
	FlyCapture2::Error error;
	FlyCapture2::PGRGuid guid;
	FlyCapture2::BusManager busMgr;
	FlyCapture2::Camera cam;
	FlyCapture2::Image rawImage;
	FlyCapture2::Image convertedImg;
	FlyCapture2::PixelFormat outPixelFmt;
	IplImage* frame;

	FlyCap2OpenCV(): outPixelFmt(FlyCapture2::PIXEL_FORMAT_MONO8), frame(0) {
		unsigned int nCameras=0;
		error=busMgr.GetNumOfCameras(&nCameras);
		if(error!=FlyCapture2::PGRERROR_OK) {
			error.PrintErrorTrace();
			exit(-1);
		}
		std::cout<<"[FlyCap2OpenCV] # of pointgrey cameras: "<<nCameras<<std::endl;

		std::cout<<"[FlyCap2OpenCV] camera index <-> camera serial number:"<<std::endl;
		for(unsigned int i=0; i<nCameras; ++i) {
			unsigned int serialNumber;
			error=busMgr.GetCameraSerialNumberFromIndex(i, &serialNumber);
			if(error!=FlyCapture2::PGRERROR_OK) {
				error.PrintErrorTrace();
				exit(-1);
			}
			std::cout<<"\t"<<i<<"\t<->\t"<<serialNumber<<std::endl;
		}
	}
	~FlyCap2OpenCV() { deinit(); }

	//need to be called fisrt
	//FIXME: what if the serial number is very small such as 0, it could be recognized as a index
	inline bool init(const unsigned int idxOrSerialNumber=0) {
		error=busMgr.GetCameraFromIndex(idxOrSerialNumber, &guid); //first try as idx
		if(error!=FlyCapture2::PGRERROR_OK) {
			error=busMgr.GetCameraFromSerialNumber(idxOrSerialNumber, &guid); //as serial number
			if(error!=FlyCapture2::PGRERROR_OK) {
				error.PrintErrorTrace();
				return false;
			} else {
				std::cout<<"[FlyCap2OpenCV::init] got camera from serial number "
					<<idxOrSerialNumber<<std::endl;
			}
		} else {
			std::cout<<"[FlyCap2OpenCV::init] got camera from index "
				<<idxOrSerialNumber<<std::endl;
		}

		error=cam.Connect(&guid);
		if(error!=FlyCapture2::PGRERROR_OK) {
			error.PrintErrorTrace();
			return false;
		}

		//query available videomode & framerate
		std::cout<<"[FlyCap2OpenCV::init] available videomode&framerate:"<<std::endl;
		for(int vi=0; vi<(int)FlyCapture2::NUM_VIDEOMODES; ++vi) {
			for(int fi=0; fi<(int)FlyCapture2::NUM_FRAMERATES; ++fi) {
				bool isSupported=false;
				cam.GetVideoModeAndFrameRateInfo((FlyCapture2::VideoMode)vi,
					(FlyCapture2::FrameRate)fi, &isSupported);
				if(isSupported) {
					std::cout<<"?v="<<vi<<"?f="<<fi<<":\t"
						<<FlyCap2OpenCV_videoModeName[vi]<<" & "<<FlyCap2OpenCV_frameRateValue[fi]<<std::endl;
				}
			}
		}

		return true;
	}

	enum ColorMode {
		COLORMODE_MONO=0,
		COLORMODE_RGB8,
		COLORMODE_BGR8,
		NUM_COLORMODE
	};

	//need to be called before start to capture and
	//after custom settings to cam object
	inline bool initBuffer(const int colormode) {
		switch(colormode) {
			case COLORMODE_RGB8: outPixelFmt=FlyCapture2::PIXEL_FORMAT_RGB8; break;
			case COLORMODE_BGR8: outPixelFmt=FlyCapture2::PIXEL_FORMAT_BGR; break;
			case COLORMODE_MONO: default: outPixelFmt=FlyCapture2::PIXEL_FORMAT_MONO8; break;
		}

		error=cam.StartCapture();
		if(error!=FlyCapture2::PGRERROR_OK) {
			error.PrintErrorTrace();
			return false;
		}

		cam.RetrieveBuffer(&rawImage);
		if(frame!=0) {
			cvReleaseImage(&frame);
			frame=0;
		}
		int depth=8, nchannels=colormode==COLORMODE_MONO?1:3;
		frame=cvCreateImage(cvSize(rawImage.GetCols(),rawImage.GetRows()), depth, nchannels);

		FlyCapture2::VideoMode vm;
		FlyCapture2::FrameRate fr;
		cam.GetVideoModeAndFrameRate(&vm, &fr);
		std::cout<<"[FlyCap2OpenCV::initBuffer] current video mode="<<std::endl;
		if(vm<FlyCapture2::NUM_VIDEOMODES) {
			std::cout<<"\t"<<FlyCap2OpenCV_videoModeName[(int)vm]<<std::endl;
		} else {
			std::cout<<"\tnot recognized!"<<std::endl;
		}

		std::cout<<"[FlyCap2OpenCV::initBuffer] current frame rate="<<std::endl;
		if(fr<FlyCapture2::NUM_FRAMERATES) {
			std::cout<<"\t"<<FlyCap2OpenCV_frameRateValue[(int)fr]<<std::endl;
		} else {
			std::cout<<"\tnot recognized!"<<std::endl;
		}
		return true;
	}

	//read an image into dst from a point grey camera
	inline bool read(cv::Mat& dst) {
		if(!cam.IsConnected()) {
			std::cout<<"[FlyCap2OpenCV::read] FlyCapture2::Camera is not connected,"
				" please call FlyCap2OpenCV::init first!"<<std::endl;
			return false;
		}
		if(frame==0) {
			std::cout<<"[FlyCap2OpenCV::read] frame==0, please call"
				" FlyCap2OpenCV::initBuffer first!"<<std::endl;
			return false;
		}

		cam.RetrieveBuffer(&rawImage);
		error=rawImage.Convert(outPixelFmt, &convertedImg);
		if(error!=FlyCapture2::PGRERROR_OK) {
			error.PrintErrorTrace();
			return false;
		}
		memcpy(frame->imageData, convertedImg.GetData(), convertedImg.GetDataSize());
		dst = cv::Mat(frame);
		return !dst.empty();
	}

	//no need to call explicitly, will be automatically called in deconstructor
	inline bool deinit() {
		if(frame!=0) {
			cvReleaseImage(&frame);
			frame=0;
		}
		error=cam.StopCapture();
		if(error!=FlyCapture2::PGRERROR_OK) {
			error.PrintErrorTrace();
		}
		if(cam.IsConnected()) {
			error=cam.Disconnect();
			if(error!=FlyCapture2::PGRERROR_OK) {
				error.PrintErrorTrace();
				return false;
			}
		}
		std::cout<<"[FlyCap2OpenCV::deinit] done!"<<std::endl;
		return true;
	}
};