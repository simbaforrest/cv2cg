#pragma once

#include <vector>

#include "OpenCVHeaders.h"

namespace keg {

//struct for template to be tracked by KEG
struct TemplateData {
	int id; //correspond to which template
	std::vector<cv::KeyPoint> keys;
	std::vector<cv::Point2f> X; //template points, get from recognizer/detector
	std::vector<cv::Point2f> crns; //corners of template image
	cv::Mat img; //template image, CV_8UC1
	cv::Mat iHI; //inverse of init Homography, X_tag = iHI * X_keg
};

}//end of keg