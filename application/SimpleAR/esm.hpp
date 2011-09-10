#ifndef ESM_HPP_
#define ESM_HPP_

#include <unsupported/Eigen/MatrixFunctions>
namespace Eigen
{
using namespace Eigen;
}

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "lie_algebra.hpp"

struct HomoESMState {
	cv::Mat H;
	double error;
};

class HomoESM
{
public:
	HomoESM() {
		delatRMSLimit=1;
	}
	//set this value smaller will give more iterations, but also more accurate
	inline void setDeltaRMSLimit(double val) {
		delatRMSLimit=val;
	}
	void setTemplateImage(const cv::Mat &image);
	void track(cv::Mat &testImage, int nIters, cv::Mat &H,
	           double &rmsError, double&ncc,
	           cv::Ptr<LieAlgebra> lieAlgebra = new LieAlgebraHomography(),
	           bool saveComputations = false,
	           std::vector<HomoESMState> *computations = 0) const;
	void visualizeTracking(const cv::Mat &H, cv::Mat &visualization) const;

private:
	double delatRMSLimit;
	cv::Mat templateImage, templateImageRowDouble;
	cv::Mat templateDxRow, templateDyRow;
	cv::Mat xx, yy;

	std::vector<cv::Point2f> templateVertices;

	static void computeGradient(const cv::Mat &image, cv::Mat &dx, cv::Mat &dy);
//	static double computeRMSError(const cv::Mat &error) const;
//	static double computeNCC(const cv::Mat& w, const cv::Mat& t) const;
	void computeJacobian(const cv::Mat &dx, const cv::Mat &dy, cv::Mat &J, cv::Ptr<LieAlgebra> lieAlgebra) const;
	void projectVertices(const cv::Mat &H, std::vector<cv::Point2f> &vertices) const;
};

#endif /* ESM_HPP_ */
