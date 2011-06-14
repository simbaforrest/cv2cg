#ifndef LIE_ALGEBRA_HPP_
#define LIE_ALGEBRA_HPP_

class LieAlgebra
{
public:
  //computes dot-product of each column in src with each basis vector
  void dot(const cv::Mat &src, cv::Mat &dst) const;

  //transform coordinates in the Lie algebra basis to a matrix in the Lie group
  cv::Mat algebra2group(const cv::Mat &lieAlgebraCoords) const;

  virtual ~LieAlgebra()
  {
  }
protected:
  LieAlgebra()
  {
  }

  class LieAlgebraBasisVector
  {
  public:
    void addPositive(int idx)
    {
      positives.push_back(idx);
    }
    void addNegative(int idx)
    {
      negatives.push_back(idx);
    }

    //computes dot-product of each column in src with basis vector
    void dot(const cv::Mat &src, cv::Mat dst) const;
    cv::Mat vector2mat(double coordinate) const;
  private:
    std::vector<int> positives;
    std::vector<int> negatives;
  };

  static cv::Mat matrixExp(const cv::Mat &mat);

  std::vector<LieAlgebraBasisVector> basis;
};

class LieAlgebraHomography : public LieAlgebra
{
public:
  LieAlgebraHomography();
};

class LieAlgebraRotation3d : public LieAlgebra
{
public:
  LieAlgebraRotation3d();
};

class LieAlgebraTranslation : public LieAlgebra
{
public:
  LieAlgebraTranslation();
};

class LieAlgebraEuclidean : public LieAlgebraTranslation
{
public:
  LieAlgebraEuclidean();
};

class LieAlgebraSimilarity : public LieAlgebraEuclidean
{
public:
  LieAlgebraSimilarity();
};

class LieAlgebraAffine : public LieAlgebra
{
public:
  LieAlgebraAffine();
};

#endif /* LIE_ALGEBRA_HPP_ */
