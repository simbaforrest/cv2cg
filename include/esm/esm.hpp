// code modified from ROS:
//	http://www.ros.org/doc/api/posest/html/classHomoESM.html
#pragma once

#include <unsupported/Eigen/MatrixFunctions>
namespace Eigen { using namespace Eigen; }

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "lie_algebra.hpp"

namespace esm {

using namespace std;
using namespace cv;

struct HomoState {
	Mat H;
	double error;//rms error
};

class Refiner
{
public:
	Refiner() { delatRMSLimit=1; }

	//set this value smaller will give more iterations, but also more accurate
	inline void setDeltaRMSLimit(double val) { delatRMSLimit=val; }

	inline void setTemplateImage(const Mat &image)
	{
		templateImage = image;
		if( templateImage.type() != CV_8UC1 ) {
			cvtColor(templateImage, templateImage, CV_RGB2GRAY);
		}

		Mat templateImageDouble;
		templateImage.convertTo(templateImageDouble, CV_64FC1);
		templateImageRowDouble = templateImageDouble.reshape(0, 1);

		Mat templateDx, templateDy;
		computeGradient(templateImage, templateDx, templateDy);
		templateDxRow = templateDx.reshape(0, 1);
		templateDyRow = templateDy.reshape(0, 1);

		xx.create(1, image.rows * image.cols, CV_64FC1);
		yy.create(1, image.rows * image.cols, CV_64FC1);
		for (int i = 0; i < image.rows * image.cols; i++) {
			xx.at<double> (0, i) = i % image.cols;
			yy.at<double> (0, i) = i / image.cols;
		}

		rect2vertices(
			Rect(0, 0, templateImage.cols, templateImage.rows),
			templateVertices );
	}

	inline void track(Mat &testImage, int nIters, Mat &H,
	           double &rmsError, double&ncc,
	           cv::Ptr<LieAlgebra> lieAlgebra = new LieAlgebraHomography(),
	           std::vector<HomoState> *computations = 0) const
	{
		if( testImage.type() != CV_8UC1 ) {
			cvtColor(testImage, testImage, CV_RGB2GRAY);
		}

		if (computations) {
			assert( computations != 0 );
			computations->clear();
		}

		double oldrms=1000000;
		cout<<"[ESM] delta RMS";
		for (int iter = 0; iter <= nIters; iter++) {
			Mat warpedTemplateDouble;
			templateImage.copyTo(warpedTemplateDouble);
			//extract template from current frame(i.e. testImage)
			//ATTENTION: that H maps warpedTemplateDouble -> testImage, so use flag WARP_INVERSE_MAP
			//ATTENTION: use flag BORDER_TRANSPARENT to handle the case that template image is
			//partially outside the view, this part won't affect NCC and RMS computation
			warpPerspective(testImage,
				warpedTemplateDouble, H,
				templateImage.size(), INTER_LINEAR|WARP_INVERSE_MAP, BORDER_TRANSPARENT);
			warpedTemplateDouble.convertTo(warpedTemplateDouble,CV_64F);

			Mat warpedTemplateRowDouble = warpedTemplateDouble.reshape(0, 1);
			Mat errorRow = warpedTemplateRowDouble - templateImageRowDouble;

			rmsError=computeRMSError(errorRow);
			double deltarms = oldrms-rmsError;
			oldrms=rmsError;
			cout<<"->"<<deltarms;

			if (computations) {
				HomoState state;
				H.copyTo(state.H);
				state.error = rmsError;
				computations->push_back(state);
			}

			if (iter == nIters || deltarms<delatRMSLimit) {
				ncc=computeNCC(warpedTemplateRowDouble,templateImageRowDouble);
				cout<<"|RMS="<<rmsError<<"|NCC="<<ncc<<endl;
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

			Mat delta = lieAlgebra->algebra2group(d);
			H = H * delta;
		}
	}

	inline void visualizeTracking(const Mat &H, Mat &visualization) const
	{
		assert(!visualization.empty());

		vector<Point2f> vertices;
		projectVertices(H, vertices);

		Scalar color = Scalar(0, 255, 0);
		int thickness = 2;
		for (size_t i = 0; i < vertices.size(); i++) {
			line( visualization, 
				vertices[i], vertices[(i + 1) % vertices.size()],
				color, thickness );
		}
	}

private:
	double delatRMSLimit;
	Mat templateImage, templateImageRowDouble;
	Mat templateDxRow, templateDyRow;
	Mat xx, yy;

	std::vector<cv::Point2f> templateVertices;

private:
	static inline void rect2vertices(const Rect &rect, vector<Point2f> &vertices)
	{
		vertices.clear();
		vertices.push_back(Point2f(rect.x, rect.y));
		vertices.push_back(Point2f(rect.x + rect.width, rect.y));
		vertices.push_back(Point2f(rect.x + rect.width, rect.y + rect.height));
		vertices.push_back(Point2f(rect.x, rect.y + rect.height));
	}


	static inline void computeGradient(const Mat &image, Mat &dx, Mat &dy)
	{
		Sobel(image, dx, CV_64FC1, 1, 0);
		Sobel(image, dy, CV_64FC1, 0, 1);

		//normalize to get derivative
		dx /= 8.;
		dy /= 8.;
	}

	static inline double computeRMSError(const Mat &error)
	{
		return norm(error) / sqrt((double)error.size().area());
	}

	//calc normalized cross-correlation (NCC)
	//between the warped image and the template image
	static inline double computeNCC(const Mat& w, const Mat& t)
	{
		Scalar mw, mt, dw, dt;
		meanStdDev(w, mw, dw);
		meanStdDev(t, mt, dt);
		Mat vecw = (w - mw.val[0])/dw.val[0];
		Mat vect = (t - mt.val[0])/dt.val[0];
		return vecw.dot(vect)/vecw.total();
	}

	inline void computeJacobian(const Mat &dx, const Mat &dy,
			Mat &Jt, cv::Ptr<LieAlgebra> lieAlgebra) const
	{
		assert( dx.rows == 1 );
		assert( dy.rows == 1 );
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

	inline void projectVertices(const Mat &H, std::vector<cv::Point2f> &vertices) const
	{
		vertices.clear();
		Mat transformedVertices;
		transform(Mat(templateVertices), transformedVertices, H);
		convertPointsHomogeneous(transformedVertices, vertices);
	}
};//end of class Refiner

}//end of namespace esm
