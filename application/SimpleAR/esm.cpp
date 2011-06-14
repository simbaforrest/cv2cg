#include "esm.hpp"
#include <iostream>
using namespace cv;

inline void rect2vertices(const Rect &rect, vector<Point2f> &vertices)
{
  vertices.clear();
  vertices.push_back(Point2f(rect.x, rect.y));
  vertices.push_back(Point2f(rect.x + rect.width, rect.y));
  vertices.push_back(Point2f(rect.x + rect.width, rect.y + rect.height));
  vertices.push_back(Point2f(rect.x, rect.y + rect.height));
}

void HomoESM::setTemplateImage(const Mat &image)
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
  for (int i = 0; i < image.rows * image.cols; i++)
  {
    xx.at<double> (0, i) = i % image.cols;
    yy.at<double> (0, i) = i / image.cols;
  }

  rect2vertices(Rect(0, 0, templateImage.cols, templateImage.rows), templateVertices);
}

double HomoESM::computeRMSError(const Mat &error)
{
  return norm(error) / sqrt((double)error.size().area());
}

void HomoESM::track(Mat &testImage, int nIters, Mat &H, double &rmsError, Ptr<LieAlgebra> lieAlgebra, bool saveComputations, vector<
    HomoESMState> *computations) const
{
  if( testImage.type() != CV_8UC1 ) {
	  cvtColor(testImage, testImage, CV_RGB2GRAY);
  }
  
  if (saveComputations)
  {
    assert( computations != 0 );
    computations->clear();
  }

  double nelem = 0.00072*templateImage.rows*templateImage.cols;
  for (int iter = 0; iter < nIters; iter++)
  {
    Mat warpedTemplateDouble;
    warpPerspective(testImage, warpedTemplateDouble, H, templateImage.size(), INTER_LINEAR|WARP_INVERSE_MAP); //
    warpedTemplateDouble.convertTo(warpedTemplateDouble,CV_64F);

    Mat errorRow = warpedTemplateDouble.reshape(0, 1) - templateImageRowDouble;

	double rms=computeRMSError(errorRow);

    if (saveComputations)
    {
      HomoESMState state;
      H.copyTo(state.H);
      state.error = rms;
      computations->push_back(state);
    }

    if (iter == nIters - 1 || rms<nelem)
    {
      rmsError = rms;
      break;
    }

    Mat warpedTestImageDx, warpedTestImageDy;
    computeGradient(warpedTemplateDouble, warpedTestImageDx, warpedTestImageDy);
    
    Mat Jt;
    computeJacobian(templateDxRow + warpedTestImageDx.reshape(0, 1), templateDyRow + warpedTestImageDy.reshape(0, 1),
                    Jt, lieAlgebra);
    
    Mat JtJ;
    mulTransposed(Jt, JtJ, false);
    //TODO: Is JtJ always non-singular?
    Mat d = -2 * JtJ.inv(DECOMP_CHOLESKY) * Jt * errorRow.t();

    Mat delta = lieAlgebra->algebra2group(d);
    H = H * delta;
  }
}

void HomoESM::projectVertices(const cv::Mat &H, std::vector<cv::Point2f> &vertices) const
{
  vertices.clear();
  Mat transformedVertices;
  transform(Mat(templateVertices), transformedVertices, H);
  convertPointsHomogeneous(transformedVertices, vertices);
}

void HomoESM::visualizeTracking(const Mat &H, Mat &visualization) const
{
  assert(!visualization.empty());
  
  vector<Point2f> vertices;
  projectVertices(H, vertices);

  Scalar color = Scalar(0, 255, 0);
  int thickness = 2;
  for (size_t i = 0; i < vertices.size(); i++)
  {
    line(visualization, vertices[i], vertices[(i + 1) % vertices.size()], color, thickness);
  }
}

void HomoESM::computeGradient(const Mat &image, Mat &dx, Mat &dy)
{
  Sobel(image, dx, CV_64FC1, 1, 0);
  Sobel(image, dy, CV_64FC1, 0, 1);

  //normalize to get derivative
  dx /= 8.;
  dy /= 8.;
}

void HomoESM::computeJacobian(const Mat &dx, const Mat &dy, Mat &Jt, Ptr<LieAlgebra> lieAlgebra) const
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

LieAlgebraHomography::LieAlgebraHomography()
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

LieAlgebraRotation3d::LieAlgebraRotation3d()
{
  basis.resize(3);
  basis[0].addPositive(1);
  basis[0].addNegative(3);
  basis[1].addPositive(2);
  basis[1].addNegative(6);
  basis[2].addPositive(5);
  basis[2].addNegative(7);
}

LieAlgebraTranslation::LieAlgebraTranslation()
{
  basis.resize(2);
  basis[0].addPositive(2);
  basis[1].addPositive(5);
}

LieAlgebraEuclidean::LieAlgebraEuclidean()
{
  LieAlgebraBasisVector vector;
  vector.addPositive(1);
  vector.addNegative(3);

  basis.push_back(vector);
}

LieAlgebraSimilarity::LieAlgebraSimilarity()
{
  LieAlgebraBasisVector vector;
  vector.addPositive(0);
  vector.addPositive(4);

  basis.push_back(vector);
}

LieAlgebraAffine::LieAlgebraAffine()
{
  basis.resize(6);
  basis[0].addPositive(0);
  basis[1].addPositive(1);
  basis[2].addPositive(3);
  basis[3].addPositive(4);
  basis[4].addPositive(2);
  basis[5].addPositive(5);
}

void LieAlgebra::LieAlgebraBasisVector::dot(const Mat &src, Mat dst) const
{
  assert( dst.rows == 1 && dst.cols == src.cols );
  assert( dst.type() == src.type() );

  for (size_t i = 0; i < positives.size(); i++)
    dst += src.row(positives[i]);
  for (size_t i = 0; i < negatives.size(); i++)
    dst -= src.row(negatives[i]);
}

Mat LieAlgebra::LieAlgebraBasisVector::vector2mat(double coordinate) const
{
  const int dim = 3;
  Mat result(dim, dim, CV_64FC1, Scalar(0));

  for (size_t i = 0; i < positives.size(); i++)
  {
    int row = positives[i] / dim;
    int col = positives[i] % dim;
    result.at<double> (row, col) = coordinate;
  }

  for (size_t i = 0; i < negatives.size(); i++)
  {
    int row = negatives[i] / dim;
    int col = negatives[i] % dim;
    result.at<double> (row, col) = -coordinate;
  }

  return result;
}

void LieAlgebra::dot(const Mat &src, Mat &dst) const
{
  dst.create(basis.size(), src.cols, CV_64FC1);
  dst.setTo(0);

  for (size_t i = 0; i < basis.size(); i++)
  {
    basis[i].dot(src, dst.row(i));
  }
}

Mat LieAlgebra::algebra2group(const Mat &lieAlgebraCoords) const
{
  Mat coords = lieAlgebraCoords.reshape(0, 1);
  assert( coords.cols == basis.size() );
  assert( coords.type() == CV_64FC1 );

  const int dim = 3;
  Mat lieAlgebraMat(dim, dim, CV_64F, Scalar(0));
  for (size_t i = 0; i < basis.size(); i++)
  {
    lieAlgebraMat += basis[i].vector2mat(coords.at<double> (0, i));
  }

  Mat lieGroupMat = matrixExp(lieAlgebraMat);
  return lieGroupMat;
}

Mat LieAlgebra::matrixExp(const Mat &mat)
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
