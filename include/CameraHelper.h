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

/* CameraHelper.h
   Multiple View Geometry related helper functions */

#include "OpenCVHeaders.h"

#include "CvMatHelper.h"

namespace CameraHelper
{

#define IDX(m,n,N) ((m)*(N)+(n)) //N cols

// s[u,v,1]' = P * [x,y,z,1]'
// P<3x4>, projection matrix
inline void project(double const *P,
                    double x, double y, double z,
                    double &u, double &v)
{
	double w;
	u = P[IDX(0,0,4)] * x + P[IDX(0,1,4)] * y
	    + P[IDX(0,2,4)] * z + P[IDX(0,3,4)] * 1;
	v = P[IDX(1,0,4)] * x + P[IDX(1,1,4)] * y
	    + P[IDX(1,2,4)] * z + P[IDX(1,3,4)] * 1;
	w = P[IDX(2,0,4)] * x + P[IDX(2,1,4)] * y
	    + P[IDX(2,2,4)] * z + P[IDX(2,3,4)] * 1;
	if(fabs(w)<1e-10) {
		u = v = 0;    // ideal point
	} else {
		u/=w;
		v/=w;
	}
}

#undef IDX

inline void project(double const *P,
                    double const *X, double *p)
{
	project(P, X[0], X[1], X[2], p[0], p[1]);
}

// K<3x3> * [R<3x3> , T<3x1>] = P<3x4>
// K<3x3> * R<3x3> [I<3x3> , -C<3x1>] = P<3x4>
// T<3x1> = -R<3x3> * C<3x1>
inline void decompose(double const *P,
                      double *K, double *R, double *T, double *C)
{
	cv::Mat mP(3,4,CV_64FC1,const_cast<double*>(P));
	cv::Mat mK(3,3,CV_64FC1,K);
	cv::Mat mR(3,3,CV_64FC1,R);
	double tmp[4];
	cv::Mat mtmp(4,1,CV_64FC1,tmp);
	//translation vector is camera Center in world coordinate system
	//so it is C rather than T in opencv manual
	decomposeProjectionMatrix(mP,mK,mR,mtmp);
	C[0]=tmp[0]/tmp[3];
	C[1]=tmp[1]/tmp[3];
	C[2]=tmp[2]/tmp[3];
	CvMatHelper::mul(3,3,3,1,R,C,T); //T = -RC, C = -R'T
	T[0]*=-1;
	T[1]*=-1;
	T[2]*=-1;
}

inline void compose(double const *K,
                    double const *R, double const *T,
                    double *P, bool TisC=false)
{
	double PP[12]= {0};
	if(!TisC) { //K[R,T]
		PP[0]=R[0], PP[1]=R[1], PP[2]= R[2], PP[3]= T[0];
		PP[4]=R[3], PP[5]=R[4], PP[6]= R[5], PP[7]= T[1];
		PP[8]=R[6], PP[9]=R[7], PP[10]=R[8], PP[11]=T[2];
		CvMatHelper::mul(3,3,3,4,K,PP,P);
	} else { //KR[I,-C], remember now T is C
		P[0]=1, P[1]=0, P[2] =0, P[3] =-T[0];
		P[4]=0, P[5]=1, P[6] =0, P[7] =-T[1];
		P[8]=0, P[9]=0, P[10]=1, P[11]=-T[2];
		CvMatHelper::mul(3,3,3,4,R,P,PP);
		CvMatHelper::mul(3,3,3,4,K,PP,P);
	}
}

inline void RotationMatrix_PH_CV(double *R)
{
	for(int i=1; i<3; ++i)
		for(int j=0; j<3; ++j) {
			R[i*3+j] *= -1;
		}
}

inline bool triangulate(const double x1, const double y1,
                        const double x2, const double y2,
                        const double P1[12], const double P2[12], double X[3])
{
	double A[12] = {
		P1[0]-x1 *P1[8], P1[1]-x1 *P1[9], P1[2]-x1 *P1[10],
		P1[4]-y1 *P1[8], P1[5]-y1 *P1[9], P1[6]-y1 *P1[10],
		P2[0]-x2 *P2[8], P2[1]-x2 *P2[9], P2[2]-x2 *P2[10],
		P2[4]-y2 *P2[8], P2[5]-y2 *P2[9], P2[6]-y2 *P2[10]
	};
	double b[4] = {
		x1 *P1[11]-P1[3],
		y1 *P1[11]-P1[7],
		x2 *P2[11]-P2[3],
		y2 *P2[11]-P2[7]
	};

	return CvMatHelper::solve(4,3,A,b,X);
}

//compute plane pose (R and T) from K and Homography
//H = s^(-1) * K * [r1,r2,T]
//this method assumes the world origin [0,0,0] lies
//in front of the camera, i.e. T[3]>0 is ensured
inline void RTfromKH(const double K[9], const double H[9],
                     double R[9], double T[3], bool doPolarDecomp=false)
{
	double invK[9];
	double A[9];
	CvMatHelper::inv(3,K,invK);
	CvMatHelper::mul(3,3,3,3,invK,H,A);
	//as suggested by AprilTag, use geometric average to scale
	double s1 = sqrt(A[0]*A[0]+A[3]*A[3]+A[6]*A[6]);
	double s2 = sqrt(A[1]*A[1]+A[4]*A[4]+A[7]*A[7]);
	double s = (A[8]>=0?1.0:-1.0)/sqrt(s1*s2); //ensure T[3]>0
	if(fabs(A[8])<1e-8) {
		std::cout<<"[RTfromKH warn] T[3]~0, please check!"<<std::endl;
	}
	CvMatHelper::scale(3,3,A,s,A);
	//TODO, should we normalize r1 and r2 respectively?
	//  what's the difference between this and polar decomposition then?
	double r1[3]= {A[0],A[3],A[6]};
	double r2[3]= {A[1],A[4],A[7]};
	double r3[3]= {0};
	CvMatHelper::cross(r1,r2,r3);
	R[0]=r1[0], R[1]=r2[0], R[2]=r3[0];
	R[3]=r1[1], R[4]=r2[1], R[5]=r3[1];
	R[6]=r1[2], R[7]=r2[2], R[8]=r3[2];
	T[0]=A[2],  T[1]=A[5],  T[2]=A[8]; //translation

//simbaforrest: add this will cause affect rendering effect, since lost info
	//as suggested by AprilTag, do polar decomposition so R is orthogonal
	//R = (UV')(VSV')
	if(doPolarDecomp) {
		double U[9],S[9],VT[9];
		CvMatHelper::svd(3,3,R,U,S,VT);
		CvMatHelper::mul(3,3,3,3,U,VT,R);
	}
}

/**
condition X such that the center is at 0 and std is 1

@param X <dimxn>: inhomogeneous points
@param T <(dim+1)x(dim+1)>: Condition matrix
@param hX <(dim+1)xn>: conditioned X of homogeneous form
*/
template<typename DType>
inline void conditioner(const cv::Mat& X, cv::Mat& T, cv::Mat& hX) {
	const int dim = X.rows;
	const int type = cv::DataType<DType>::type;
	T = cv::Mat::eye(dim+1, dim+1, type);
	hX.create(dim+1, X.cols, type);

	for(int i=0; i<dim; ++i) {
		cv::Mat m, s;
		cv::meanStdDev(X.row(i), m, s);
		const DType& mean=m.at<DType>(0);
		DType sigma=s.at<DType>(0);
		double sigma_inv = sigma==0?1:(1.0f/sigma);
		T.at<DType>(i,i) = sigma_inv;
		T.at<DType>(i,dim) = -mean*sigma_inv;
	}
	cv::Mat Xi=cv::Mat::ones(dim+1,1,type);
	for(int i=0; i<hX.cols; ++i) {
		X.col(i).copyTo(Xi.rowRange(0,dim));
		hX.col(i) = T*Xi;
	}
}

/**
find projection matrix P such that U=HXw ([U_i;1]~P*[X_i;1] \forall i\in[1,n])

@param U <2xn>: measured 2d image points
@param Xw <3xn>: world 3d points
@param P <3x4>: projection matrix, P=K[Rwc,twc]
@param K <3x3>: calibration matrix
@param Rwc <3x3>: rotation matrix (from world to camera)
@param twc <3x1>: translation matrix (from world to camera, i.e. world's origin in camera frame)
*/
template<typename DType>
inline void dlt3(const cv::Mat& U, const cv::Mat& Xw, cv::Mat& P)
{
	const int n = Xw.cols;
	assert(n>=6 && U.cols==n && U.rows==2 && Xw.rows==3);
	const int type = cv::DataType<DType>::type;
	P.create(3,4,type);

	cv::Mat T, hXw, C, hU;
	conditioner<DType>(Xw, T, hXw);
	conditioner<DType>(U, C, hU);

	cv::Mat AtA=cv::Mat::zeros(12, 12, type);
	for(int i=0; i<n; ++i) {//for each correspondence
		cv::Mat xwxwt=hXw.col(i)*hXw.col(i).t();
		DType u=hU.at<DType>(0,i);
		DType v=hU.at<DType>(1,i);
		AtA(cv::Rect(0,0,4,4)) += xwxwt;
		AtA(cv::Rect(4,4,4,4)) += xwxwt;
		AtA(cv::Rect(8,0,4,4)) += -u*xwxwt;
		AtA(cv::Rect(8,4,4,4)) += -v*xwxwt;
		AtA(cv::Rect(8,8,4,4)) += (u*u+v*v)*xwxwt;
	}
	//make symmetrical
	AtA(cv::Rect(8,0,4,4)).copyTo(AtA(cv::Rect(0,8,4,4)));
	AtA(cv::Rect(8,4,4,4)).copyTo(AtA(cv::Rect(4,8,4,4)));

	//A=USV' => A'A=VS^2V', so A and A'A share the same V for SVD
	cv::SVD svd(AtA);
	P = svd.vt.row(11).reshape(1, 3);

	P = C.inv()*P*T;
}

/**
decompose projection matrix P into K*[Rwc,twc]

@param P <3x4>: projection matrix, P=K[Rwc,twc]
@param K <3x3>: calibration matrix, [a,0,c;0,b,d;0,0,1]
@param Rwc <3x3>: rotation matrix (from world to camera)
@param twc <3x1>: translation matrix (from world to camera, i.e. world's origin in camera frame)
*/
template<typename DType>
inline void decomposeP10(const cv::Mat& P, cv::Mat& K, cv::Mat& Rwc, cv::Mat& twc) {
	const int type = cv::DataType<DType>::type;
	K=cv::Mat::eye(3,3,type);
	Rwc.create(3,3,type);
	twc.create(3,1,type);

	cv::Mat nP = P/cv::norm(P(cv::Rect(0,2,3,1)))*(
		cv::determinant(P(cv::Rect(0,0,3,3)))>=0?1:-1);
	cv::Mat r789 = nP(cv::Rect(0,2,3,1));
	DType t3 = nP.at<DType>(2,3);
	DType c = r789.dot(nP(cv::Rect(0,0,3,1)));
	DType d = r789.dot(nP(cv::Rect(0,1,3,1)));
	DType a = cv::norm(nP(cv::Rect(0,0,3,1))-c*r789);
	DType b = cv::norm(nP(cv::Rect(0,1,3,1))-d*r789);
	K.at<DType>(0,0)=a; K.at<DType>(0,2)=c;
	K.at<DType>(1,1)=b; K.at<DType>(1,2)=d;
	cv::Mat Rraw(3,3,type);
	Rraw.row(0) = (nP(cv::Rect(0,0,3,1))-c*r789)/a;
	Rraw.row(1) = (nP(cv::Rect(0,1,3,1))-d*r789)/b;
	Rraw.row(2) = r789;
	//polar decomposition to get a valid rotation matrix
	cv::Mat u,s,vt;
	cv::SVD::compute(Rraw, s, u, vt);
	Rwc = u*vt;
	twc.at<DType>(0)=(P.at<DType>(0,3)-c*t3)/a;
	twc.at<DType>(1)=(P.at<DType>(1,3)-d*t3)/b;
	twc.at<DType>(2)=t3;
}

/**
calibrate camera using 2d-3d/2d-2d point correspondences(if 2d-3d,inited by dlt3), optimized by LM

@param imagePtsArr <nxnix2>: measured 2d image points
@param worldPtsArr <nxnix3>: measured 3d world points
@param isRig3d <1x1>: true if the rig is 3d and false otherwise
@param imageSize <2x1>: width and height of image
@param K <3x3>: calibration matrix
@param distCoeffs <5x1>: distortion coefficients, [k1, k2, p1, p2, k3]
@param rvecs <nx3x1>: rotation vectors (from world to camera)
@param tvecs <nx3x1>: translation vectors (from world to camera, i.e. world's origin in camera frame)
@param totalAvgErr <1x1>: total rms
@param CovK <9x9>: covariance matrix for all intrinsics with order [fx,fy,cx,cy,k1,k2,p1,p2,k3]
@param Covrs, Covts <nx3x3>: covariance matrix for each extrinsics
*/
inline void intrinsicCalibration(const std::vector<std::vector<cv::Point2f> >& imagePtsArr,
						  const std::vector<std::vector<cv::Point3f> >& worldPtsArr,
						  bool isRig3d,
						  const cv::Size& imageSize,
						  cv::Mat& K, cv::Mat& distCoeffs,
						  std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
						  double& totalAvgErr,
						  cv::Mat* pCovK=0,
						  std::vector<cv::Mat>* pCovrs=0, std::vector<cv::Mat>* pCovts=0)
{
	assert(imagePtsArr.size()==worldPtsArr.size());
	const int nImgs = imagePtsArr.size();

	int flag=0;
	if(isRig3d) {
		if(K.empty()) {//1. init by dlt3 from the first view
			cv::Mat P, Rwc, twc;
			cv::Mat Ut, Xwt;
			cv::Mat(imagePtsArr[0]).reshape(1).convertTo(Ut, cv::DataType<double>::type);
			cv::Mat(worldPtsArr[0]).reshape(1).convertTo(Xwt, cv::DataType<double>::type);
			dlt3<double>(Ut.t(), Xwt.t(), P);
			decomposeP10<double>(P, K, Rwc, twc);
		}
		flag=CV_CALIB_USE_INTRINSIC_GUESS;
	}
	const int nDistCoeffs = 5;
	switch(distCoeffs.total()) {
		case 0: flag|=CV_CALIB_FIX_K1|CV_CALIB_FIX_K2|CV_CALIB_FIX_K3|CV_CALIB_ZERO_TANGENT_DIST; break;
		case 1: flag|=CV_CALIB_FIX_K2|CV_CALIB_FIX_K3|CV_CALIB_ZERO_TANGENT_DIST; break;
		case 2: flag|=CV_CALIB_FIX_K3|CV_CALIB_ZERO_TANGENT_DIST; break;
		case 3:
		case 4: flag|=CV_CALIB_FIX_K3; break;
	};
	distCoeffs = cv::Mat::zeros(nDistCoeffs,1,CV_64FC1);

	//2. optimize by LM
	totalAvgErr = cv::calibrateCamera(worldPtsArr, imagePtsArr, imageSize,
		K, distCoeffs, rvecs, tvecs, flag);

	//3. estimate uncertainty
	if(pCovK!=0 || pCovrs!=0 || pCovts!=0) {
		const int nKParam = 4;
		const int nIntrinsicParams = nKParam+nDistCoeffs;
		const int nParams = nIntrinsicParams+6*nImgs;
		cv::Mat JtJ(nParams,nParams,CV_64FC1,0.0f);
		for(int i=0; i<nImgs; ++i) {
			std::vector<cv::Point2f> imgPts;
			cv::Mat J;//Jacobian for this image, J=[Je,Ji], Je is the extrinsic part, Ji intrinsic
			cv::projectPoints(worldPtsArr[i], rvecs[i], tvecs[i], K, distCoeffs, imgPts, J);
			cv::Mat Je=J.colRange(0,6); //Je=[Jr,Jt]
			cv::Mat Ji=J.colRange(6,6+nIntrinsicParams); //Ji=[Jfx,Jfy,Jcx,Jcy,Jk1,Jk2,Jp1,Jp2,Jk3]
			JtJ(cv::Rect(0,0,nIntrinsicParams,nIntrinsicParams))+=Ji.t()*Ji;
			JtJ(cv::Rect(nIntrinsicParams+i*6,nIntrinsicParams+i*6,6,6))+=Je.t()*Je;
			JtJ(cv::Rect(nIntrinsicParams+i*6,0,6,nIntrinsicParams))+=Ji.t()*Je;
		}
		cv::completeSymm(JtJ, false);
		//std::clog<<"JtJ="<<JtJ<<";"<<std::endl;
		const double sigma_m=7e-5*imageSize.width;//std of measurement
		cv::Mat cov=(sigma_m*sigma_m)*JtJ.inv(); //covariance of all parameters estimated is inv(Jt*inv(cov_measurement)*J)
		if(pCovK) {
			cov(cv::Rect(0,0,nIntrinsicParams,nIntrinsicParams)).copyTo((*pCovK));
		}
		if(pCovrs) {
			pCovrs->resize(nImgs);
			for(int i=0; i<nImgs; ++i) {
				cov(cv::Rect(nIntrinsicParams+i*6, nIntrinsicParams+i*6, 3, 3)).copyTo(pCovrs->at(i));
			}
		}
		if(pCovts) {
			pCovts->resize(nImgs);
			for(int i=0; i<nImgs; ++i) {
				cov(cv::Rect(nIntrinsicParams+i*6+3, nIntrinsicParams+i*6+3, 3, 3)).copyTo(pCovts->at(i));
			}
		}
	}
}


}//CameraHelper
