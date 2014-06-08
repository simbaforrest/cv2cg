// code modified from ROS:
//	http://www.ros.org/doc/api/posest/html/classHomoESM.html
#pragma once

#include <unsupported/Eigen/MatrixFunctions>
namespace Eigen { using namespace Eigen; }

#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
//opencv include
#include "OpenCVHelper.h"

#include "lie_algebra.hpp"

namespace esm {

#ifndef ESM_DEBUG
	#define ESM_DEBUG 1
#endif

using namespace std;
using namespace cv;

class Tracker : public Interface
{
public:
	Tracker() { delatRMSLimit=0.5; maxIter = 5; }

	inline bool init(const Mat& refimg) {
		setTemplateImage(refimg); return true;
	}

	inline bool operator()(Mat& curimg, Mat& H, double& zncc, double& rms) {
		(*this)(curimg, maxIter, H, rms, zncc);
		return true;
	}

	inline void setTermCrit(int maxIter=5, double mprec=2) {
		this->maxIter = maxIter>0?maxIter:0;
		this->mprec = mprec;
		setDeltaRMSLimit(0.01/mprec);
	}

	//set this value smaller will give more iterations, but also more accurate
	inline void setDeltaRMSLimit(double val) { delatRMSLimit=val; }

	inline void setTemplateImage(const Mat &image)
	{
		image.copyTo(templateImage);
		if( templateImage.type() != CV_8UC1 ) {
			cvtColor(templateImage, templateImage, CV_RGB2GRAY);
		}

		Mat templateImageDouble;
		templateImage.convertTo(templateImageDouble, CV_64F);
		templateImageRowDouble = templateImageDouble.reshape(0, 1);

		Mat templateDx, templateDy;
		computeGradient(templateImageDouble, templateDx, templateDy);
		templateDxRow = templateDx.reshape(0, 1);
		templateDyRow = templateDy.reshape(0, 1);

		xx.create(1, image.rows * image.cols, CV_64FC1);
		yy.create(1, image.rows * image.cols, CV_64FC1);
		for (int i = 0; i < image.rows * image.cols; i++) {
			xx.at<double> (0, i) = i % image.cols;
			yy.at<double> (0, i) = i / image.cols;
		}
#if ESM_DEBUG
		namedWindow("error");
		cvMoveWindow("error",660,10);
		namedWindow("warp");
		cvMoveWindow("warp",660,270);
//		namedWindow("templateImage");
//		cv::imshow("templateImage", templateImage);
#endif
	}

	inline void operator()(Mat &curimg, int nIters, Mat &H,
	           double &rms, double&zncc,
	           cv::Ptr<LieAlgebra> lieAlgebra = new LieAlgebraHomography()) const
	{
		if( curimg.type() != CV_8UC1 ) {
			cvtColor(curimg, curimg, CV_RGB2GRAY);
		}

		double oldrms=1000000;
		cout<<"[ESM] delta RMS";
		for (int iter = 0; iter <= nIters; iter++) {
			Mat warpedTemplateDouble;
			//extract template from current frame(i.e. curimg)
			//ATTENTION: that H maps warpedTemplateDouble -> curimg, so use flag WARP_INVERSE_MAP
			//DEPRECATED //ATTENTION: use flag BORDER_TRANSPARENT to handle the case that template image is
			//partially outside the view, this part won't affect zncc and RMS computation
			//DEPRECATED //templateImage.copyTo(warpedTemplateDouble);
			warpPerspective(curimg,
				warpedTemplateDouble, H,
				templateImage.size(), INTER_LINEAR|WARP_INVERSE_MAP);
			warpedTemplateDouble.convertTo(warpedTemplateDouble,CV_64F);

			Mat warpedTemplateRowDouble = warpedTemplateDouble.reshape(0, 1);
			Mat errorRow = warpedTemplateRowDouble - templateImageRowDouble;
#if ESM_DEBUG
			cv::imshow("error", errorRow.reshape(0,templateImage.rows)/255.0);
			cv::imshow("warp", warpedTemplateDouble/255.0);
#endif
			Mat mask = warpedTemplateDouble!=0;
			rms=helper::rms<double>(errorRow, &mask);
			double deltarms = oldrms-rms;
			oldrms=rms;
			cout<<"->"<<deltarms;

			if (iter == nIters || deltarms<delatRMSLimit) {
				zncc=helper::zncc<double>(warpedTemplateRowDouble,
						templateImageRowDouble,&mask);
				cout<<"|RMS="<<rms<<"|ZNCC="<<zncc<<endl;
				break;
			}

			Mat warpedTestImageDx, warpedTestImageDy;
			computeGradient(warpedTemplateDouble, warpedTestImageDx, warpedTestImageDy);

			Mat Jt;
			computeJacobian(templateDxRow + warpedTestImageDx.reshape(0, 1),
				            templateDyRow + warpedTestImageDy.reshape(0, 1),
				            Jt, lieAlgebra);

			Mat JtJ;
			mulTransposed(Jt, JtJ, false);
			//TODO: Is JtJ always non-singular?
			Mat d = -2 * JtJ.inv(DECOMP_CHOLESKY) * Jt * errorRow.t();
			//Mat d = -2 * JtJ.inv(DECOMP_SVD) * Jt * errorRow.t();

			Mat delta = lieAlgebra->algebra2group(d);
			H = H * delta;
		}
	}

private:
	double delatRMSLimit;
	Mat templateImage, templateImageRowDouble;
	Mat templateDxRow, templateDyRow;
	Mat xx, yy;

private:
	static inline void computeGradient(const Mat &image, Mat &dx, Mat &dy)
	{
		Sobel(image, dx, CV_64FC1, 1, 0, 3, 0.125);
		Sobel(image, dy, CV_64FC1, 0, 1, 3, 0.125);
//		Sobel(image, dx, CV_64FC1, 1, 0, 1, 0.5);
//		Sobel(image, dy, CV_64FC1, 0, 1, 1, 0.5);
	}

	inline void computeJacobian(const Mat &dx, const Mat &dy,
			Mat &Jt, cv::Ptr<LieAlgebra> lieAlgebra) const
	{
//		assert( dx.rows == 1 );
//		assert( dy.rows == 1 );
		Mat Ix = dx;
		Mat Iy = dy;

		const int dim = 3;
		Mat JiJw_t(dim * dim, Ix.cols, CV_64FC1);
		JiJw_t.row(0) = Ix.mul(xx);
		JiJw_t.row(1) = Ix.mul(yy);

		Mat tmpMat = JiJw_t.row(2);
		Ix.copyTo(tmpMat);

		JiJw_t.row(3) = Iy.mul(xx);
		JiJw_t.row(4) = Iy.mul(yy);

		tmpMat = JiJw_t.row(5);
		Iy.copyTo(tmpMat);

		JiJw_t.row(6) = -(xx.mul(JiJw_t.row(0)) + yy.mul(JiJw_t.row(3)));
		JiJw_t.row(7) = -(xx.mul(JiJw_t.row(1)) + yy.mul(JiJw_t.row(4)));
		JiJw_t.row(8) = -(xx.mul(JiJw_t.row(2)) + yy.mul(JiJw_t.row(5)));

		lieAlgebra->dot(JiJw_t, Jt);
	}
};//end of class Tracker

}//end of namespace esm

typedef esm::Tracker ESMTracker;
