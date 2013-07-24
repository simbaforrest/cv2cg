#pragma once
#include "flycap2opencv.hpp"
#include "ImageHelper.h"
#include "UtilHelper.h"

namespace ImageHelper {
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
	ImageSource_PGR(string content) {
		std::vector<std::string> contentParts=UtilHelper::split(content, '&');
		cout<<"[ImageSource_PGR] open from device: "<<contentParts[0]<<endl;
		if( !cap.init(atoi(contentParts[0].c_str())) ) {
			cout<<"[ImageSource_PGR error] failed to open!"<<endl;
			exit(-1);
		}

		//try to force camera to be 640x480
		cap.error=cap.cam.SetVideoModeAndFrameRate(
			FlyCapture2::VIDEOMODE_640x480Y8,
			FlyCapture2::FRAMERATE_60);
		if(cap.error!=FlyCapture2::PGRERROR_OK) {
			cap.error=cap.cam.SetVideoModeAndFrameRate(
				FlyCapture2::VIDEOMODE_640x480Y8,
				FlyCapture2::FRAMERATE_30);
			if(cap.error!=FlyCapture2::PGRERROR_OK) {
				cout<<"[ImageSource_PGR error] can not set camera mode to"
					"VIDEOMODE_640x480Y8 and FRAMERATE_60 or FRAMERATE_30!"<<endl;
				exit(-1);
			}
		}

		//cap.error=cap.cam.SetVideoModeAndFrameRate(
		//	FlyCapture2::VIDEOMODE_1280x960Y8,
		//	FlyCapture2::FRAMERATE_15);
		//if(cap.error!=FlyCapture2::PGRERROR_OK) {
		//	cap.error=cap.cam.SetVideoModeAndFrameRate(
		//		FlyCapture2::VIDEOMODE_1280x960Y8,
		//		FlyCapture2::FRAMERATE_7_5);
		//	if(cap.error!=FlyCapture2::PGRERROR_OK) {
		//		cout<<"[ImageSource_PGR error] can not set camera mode to"
		//			"VIDEOMODE_1280x960Y8 and FRAMERATE_15 or FRAMERATE_7_5!"<<endl;
		//		exit(-1);
		//	}
		//}

		cap.initBuffer(false);

		isDone = false;
		isPause = false;
	}

	inline string classname() {
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
			cout<<"[ImageSource_PGR] capture error, exit!"<<endl;
			exit(-1);
		}
		current.release();
	}

	~ImageSource_PGR() {
		cout<<"[ImageSource_PGR] camera closed."<<endl;
	}

	inline void reportInfo() {}
};

/**
helper function for create an ImageSource from url
url format example:
video:///home/simbaforrest/Videos/Webcam/keg_april.ogv
camera://0
photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/img00000.jpg
photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/img*.jpg
list:///home/simbaforrest/Videos/Webcam/seq_UMshort/list.txt
pgr://0

@param url url for image source
@return created ImageSource
*/
inline ImageSource* createImageSource2(string url) {
	size_t pos = url.find("://");
	if(pos == string::npos) {
		cout<<"[ImageSource error] invalid image source url!"<<endl;
		return NULL;
	}
	string header(url, 0, pos);
	string content(url, pos+3, url.length()-pos-3);
	if(header=="pgr") {
		return new ImageSource_PGR(content);
	}
	return ImageHelper::createImageSource(url);
}

}//ImageHelper