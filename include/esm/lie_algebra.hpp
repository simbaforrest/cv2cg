// code modified from ROS:
//	http://www.ros.org/doc/api/posest/html/classHomoESM.html
#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>


namespace esm {

using namespace std;
using namespace cv;

class LieAlgebra
{
public:
	//computes dot-product of each column in src with each basis vector
	inline void dot(const cv::Mat &src, cv::Mat &dst) const
	{
		dst.create(basis.size(), src.cols, CV_64FC1);
		dst.setTo(0);

		for (size_t i = 0; i < basis.size(); i++) {
			basis[i].dot(src, dst.row(i));
		}
	}

	//transform coordinates in the Lie algebra basis to a matrix in the Lie group
	inline cv::Mat algebra2group(const cv::Mat &lieAlgebraCoords) const
	{
		Mat coords = lieAlgebraCoords.reshape(0, 1);
		assert( coords.cols == basis.size() );
		assert( coords.type() == CV_64FC1 );

		const int dim = 3;
		Mat lieAlgebraMat(dim, dim, CV_64F, Scalar(0));
		for (size_t i = 0; i < basis.size(); i++) {
			lieAlgebraMat += basis[i].vector2mat(coords.at<double> (0, i));
		}

		Mat lieGroupMat = matrixExp(lieAlgebraMat);
		return lieGroupMat;
	}

	static inline cv::Mat matrixExp(const cv::Mat &mat)
	{
		assert( mat.rows == 3 );
		assert( mat.cols == 3 );
		assert( mat.type() == CV_64FC1 );

		Eigen::Matrix<double, 3, 3> eigenMat;
		cv2eigen(mat, eigenMat);
		Eigen::Matrix<double, 3, 3> matExp;
		matExp = eigenMat.exp();

		Mat result;
		eigen2cv(matExp, result);
		return result;
	}

	virtual ~LieAlgebra() {}
protected:
	LieAlgebra() {}

	class LieAlgebraBasisVector
	{
	public:
		inline void addPositive(int idx) { positives.push_back(idx); }
		inline void addNegative(int idx) { negatives.push_back(idx); }

		//computes dot-product of each column in src with basis vector
		inline void dot(const cv::Mat &src, cv::Mat dst) const
		{
			assert( dst.rows == 1 && dst.cols == src.cols );
			assert( dst.type() == src.type() );

			for (size_t i = 0; i < positives.size(); i++) {
				dst += src.row(positives[i]);
			}
			for (size_t i = 0; i < negatives.size(); i++) {
				dst -= src.row(negatives[i]);
			}
		}

		inline cv::Mat vector2mat(double coordinate) const
		{
			const int dim = 3;
			Mat result(dim, dim, CV_64FC1, Scalar(0));

			for (size_t i = 0; i < positives.size(); i++) {
				int row = positives[i] / dim;
				int col = positives[i] % dim;
				result.at<double> (row, col) = coordinate;
			}

			for (size_t i = 0; i < negatives.size(); i++) {
				int row = negatives[i] / dim;
				int col = negatives[i] % dim;
				result.at<double> (row, col) = -coordinate;
			}

			return result;
		}
	private:
		std::vector<int> positives;
		std::vector<int> negatives;
	};

protected:
  std::vector<LieAlgebraBasisVector> basis;
};

class LieAlgebraHomography : public LieAlgebra
{
public:
	LieAlgebraHomography()
	{
		basis.resize(8);
		basis[0].addPositive(2);
		basis[1].addPositive(5);
		basis[2].addPositive(1);
		basis[3].addPositive(3);

		basis[4].addPositive(0);
		basis[4].addNegative(4);
		basis[5].addNegative(4);
		basis[5].addPositive(8);

		basis[6].addPositive(6);
		basis[7].addPositive(7);
	}
};

class LieAlgebraRotation3d : public LieAlgebra
{
public:
	LieAlgebraRotation3d()
	{
		basis.resize(3);
		basis[0].addPositive(1);
		basis[0].addNegative(3);
		basis[1].addPositive(2);
		basis[1].addNegative(6);
		basis[2].addPositive(5);
		basis[2].addNegative(7);
	}
};

class LieAlgebraTranslation : public LieAlgebra
{
public:
	LieAlgebraTranslation()
	{
		basis.resize(2);
		basis[0].addPositive(2);
		basis[1].addPositive(5);
	}
};

class LieAlgebraEuclidean : public LieAlgebraTranslation
{
public:
	LieAlgebraEuclidean()
	{
		LieAlgebraBasisVector vector;
		vector.addPositive(1);
		vector.addNegative(3);

		basis.push_back(vector);
	}
};

class LieAlgebraSimilarity : public LieAlgebraEuclidean
{
public:
	LieAlgebraSimilarity()
	{
		LieAlgebraBasisVector vector;
		vector.addPositive(0);
		vector.addPositive(4);

		basis.push_back(vector);
	}
};

class LieAlgebraAffine : public LieAlgebra
{
public:
	LieAlgebraAffine()
	{
		basis.resize(6);
		basis[0].addPositive(0);
		basis[1].addPositive(1);
		basis[2].addPositive(3);
		basis[3].addPositive(4);
		basis[4].addPositive(2);
		basis[5].addPositive(5);
	}
};

}//end of namespace esm
