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

/* CV2CG.h */

//standard include
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <time.h>

#include "Log.h"
#include "OpenCVHelper.h"

//Statement:
//All notation such as K,R,C,T,P are in standard computer vision manner
//details in <<Multiple View Geometry>> by Richard Hartley and A. Zisserman

//some explanations of formulas in this file could be found on my undergraduate
//thesis (in Chinese):
//http://www-personal.umich.edu/~cforrest/upload/ChenFeng.UnderGrad.Thesis.ch.pdf

namespace CV2CG
{
//get CV matrices from an osg::Camera
//need to provided image size imgWximgH since K depends on them
inline void cg2cv(const osg::Camera &camera,
                  double imgW, double imgH,
                  double K[3][3], double C[3], double R[3][3])
{
	double l,r,b,t,n,f;
	camera.getProjectionMatrixAsFrustum(l,r,b,t,n,f);

	//K
	//[ alphaX    0     u0 ]
	//[   0     alphaY  v0 ]
	//[   0       0      1 ]
	helper::zeros(3,3,K[0]);
	K[0][0] = n*imgW/(r-l); //alphaX
	K[1][1] = n*imgH/(t-b); //alphaY
	K[0][2] = -imgW*(r+l)/((r-l)*2)+imgW/2;//u0
	K[1][2] = imgH*(t+b)/((t-b)*2)+imgH/2;//v0
	K[2][2] = 1;

	osg::Vec3 center,eye,up;
	camera.getViewMatrixAsLookAt(eye, center, up);
	C[0]=eye.x();
	C[1]=eye.y();
	C[2]=eye.z();

	osg::Matrix vmat = camera.getViewMatrix();
	R[0][0] =  vmat(0,0);
	R[0][1] =  vmat(1,0);
	R[0][2] =  vmat(2,0);
	R[1][0] = -vmat(0,1);
	R[1][1] = -vmat(1,1);
	R[1][2] = -vmat(2,1);
	R[2][0] = -vmat(0,2);
	R[2][1] = -vmat(1,2);
	R[2][2] = -vmat(2,2);
}

//set up projection matrix of osg::Camera from CV calibration matrix
inline void cv2cg(const double K[3][3], const double n, const double f,
                  const double imgW, const double imgH, osg::Camera &camera)
{
	//intrinsic
	double ax=K[0][0]/K[2][2], ay=K[1][1]/K[2][2],
	       u0=K[0][2]/K[2][2], v0=K[1][2]/K[2][2]; //insure that K[2][2]==1
	double l,r,t,b;

	r = ( imgW - u0 ) * n / ax;
	l = r - n*imgW/ax;
	t = v0 * n / ay;
	b = t - n*imgH/ay;

	camera.setProjectionMatrixAsFrustum(l,r,b,t,n,f);
}

//set up modelview matrix of osg::Camera from CV calibration matrix
//!!! note the difference of T and C here!!!
inline void cv2cg(const double T[3], const double R[3][3], osg::Camera &camera,
                  bool TisC=false)
{
	if(TisC) {
		double tT[3];
		helper::mul(3,3,3,1,R[0],T,tT);
		tT[0]*=-1;
		tT[1]*=-1;
		tT[2]*=-1;
		//extrinsic
		osg::Matrix vmat(
		    R[0][0], -R[1][0], -R[2][0], 0,
		    R[0][1], -R[1][1], -R[2][1], 0,
		    R[0][2], -R[1][2], -R[2][2], 0,
		    tT[0], -tT[1], -tT[2], 1            //ATTENTION! should set as (T0,-T1,-T2)
		);
		camera.setViewMatrix(vmat);
	} else {
		//extrinsic
		osg::Matrix vmat(
		    R[0][0], -R[1][0], -R[2][0], 0,
		    R[0][1], -R[1][1], -R[2][1], 0,
		    R[0][2], -R[1][2], -R[2][2], 0,
		    T[0], -T[1], -T[2], 1            //ATTENTION! should set as (T0,-T1,-T2)
		);
		camera.setViewMatrix(vmat);
	}
}


//setup an osg::Camera from CV matrices
inline void cv2cg(const double K[3][3], const double C[3],
                  const double R[3][3], const double n, const double f,
                  const double imgW, const double imgH, osg::Camera &camera)
{
	//intrinsic
	double ax=K[0][0]/K[2][2], ay=K[1][1]/K[2][2],
	       u0=K[0][2]/K[2][2], v0=K[1][2]/K[2][2]; //insure that K[2][2]==1
	double l,r,t,b;

	r = ( imgW - u0 ) * n / ax;
	l = r - n*imgW/ax;
	t = v0 * n / ay;
	b = t - n*imgH/ay;

	camera.setProjectionMatrixAsFrustum(l,r,b,t,n,f);

	//extrinsic
	double T[3];
	helper::mul(3,3,3,1,R[0],C,T);
	T[0]*=-1;
	T[1]*=-1;
	T[2]*=-1;
	osg::Matrix vmat(
	    R[0][0], -R[1][0], -R[2][0], 0,
	    R[0][1], -R[1][1], -R[2][1], 0,
	    R[0][2], -R[1][2], -R[2][2], 0,
	    T[0], -T[1], -T[2], 1            //ATTENTION! should set as (T0,-T1,-T2)
	);
	camera.setViewMatrix(vmat);
}

//this actually use a osg camera to get cv photo coordinate
inline void CG_Project(
    const osg::Camera &camera, //camera
    double x, double y, double z, //world 3d point
    double &u, double &v //image 2d point
)
{
	osg::Vec3 wp(x,y,z);
	osg::Vec3 ep = wp * camera.getViewMatrix();
	double l,r,b,t,n,f;
	camera.getProjectionMatrixAsFrustum(l,r,b,t,n,f);
	const osg::Image *img = dynamic_cast<const osg::Image *>(
	                            camera.getUserData());
	double imgW=500, imgH=500;
	if(!img) {
		const osg::Viewport *vp = camera.getViewport();
		if(vp) {
			imgW = vp->width();
		}
		imgH = vp->height();
	} else {
		imgW=img->s(), imgH=img->t();
	}
	//const osg::Viewport* vp = camera.getViewport();
	//double x0 = vp?vp->x():0, y0 = vp?vp->y():0,
	//	w = vp?:vp->width():1, h = vp?vp->height():1;
	double xe=ep.x(), ye=ep.y(), ze=ep.z();
	//u = -n*w*xe/((r-l)*ze)-w*(r+l)/((r-l)*2)+x0+w/2;
	//v = -n*h*ye/((t-b)*ze)-h*(t+b)/((t-b)*2)+y0+h/2;
	u = -n*imgW*xe/((r-l)*ze)-imgW*(r+l)/((r-l)*2)+imgW/2;
	v = -n*imgH*ye/((t-b)*ze)-imgH*(t+b)/((t-b)*2)+imgH/2;
	// be careful, this change the photo coordinate system to computer vision
	v = imgH-v;
}

inline std::ostream &operator<<(std::ostream &os, const osg::Matrix &mat)
{
	os << std::setiosflags(std::ios::scientific);
	for(int i=0; i<4; ++i) {
		for(int j=0; j<4; ++j) {
			os << mat(i,j) << "\t";
		}
		os << std::endl;
	}
	return os;
}

inline std::ostream &operator<<(std::ostream &os, const osg::Camera &camera)
{
	os << std::setiosflags(std::ios::scientific);
	osg::Matrix vmat = camera.getViewMatrix();
	osg::Matrix pmat = camera.getProjectionMatrix();
	os << "ViewMatrix = " << std::endl;
	os << vmat ;
	os << "ProjectionMatrix = " << std::endl;
	os << pmat ;
	double l,r,b,t,n,f;
	camera.getProjectionMatrixAsFrustum(l,r,b,t,n,f);
	os <<
	   "l=" << l << " r=" << r <<
	   " b=" << b << " t=" << t <<
	   " n=" << n << " f=" << f <<std::endl;
	const osg::Viewport &viewport = *(camera.getViewport());
	os << "x=" << viewport.x() << " y=" << viewport.y()
	   << " w=" << viewport.width() << " h=" << viewport.height() << std::endl;
	return os;
}

inline void CG_Report(const osg::Camera &camera,
                      std::ostream &out=std::cout)
{
	out<<camera<<std::endl;
	out<<"#Test CG Projection:"<<std::endl;
	double u,v;
	CG_Project(camera, 0, 0, 0, u, v);
	out<<"#[0,0,0]->["<<u<<","<<v<<"]"<<std::endl;
}

inline void CV_Report(const osg::Camera &camera,
                      std::ostream &out=std::cout)
{
	const osg::Image *img = dynamic_cast<const osg::Image *>(
	                            camera.getUserData());
	double imgW=500, imgH=500;
	if(!img) {
		out << "#Camera do not have valid Image as UserData!"
		    " use viewport size instead!"<<std::endl;
		const osg::Viewport *vp = camera.getViewport();
		if(!vp) {
			out << "#No valid viewport, use default image size!"<<std::endl;
		} else {
			imgW = vp->width();
		}
		imgH = vp->height();
		out << "imgW=" << imgW << " imgH=" <<imgH<<std::endl;
	} else {
		imgW=img->s(), imgH=img->t();
		out << "imgW=" << imgW << " imgH=" <<imgH<<std::endl;
	}

	double K[3][3],C[3],T[3],R[3][3],P[3][4];
	cg2cv(camera, imgW, imgH, K,C,R);
	helper::compose(K[0],R[0],C,P[0],true);
	helper::mul(3,3,3,1,R[0],C,T);
	T[0]*=-1;
	T[1]*=-1;
	T[2]*=-1;

	out<<"K=\n"<<helper::PrintMat<>(3,3,K[0]);
	out<<"R=\n"<<helper::PrintMat<>(3,3,R[0]);
	out<<"C=\n"<<helper::PrintMat<>(3,1,C);
	out<<"T=\n"<<helper::PrintMat<>(3,1,T);
	out<<"P=\n"<<helper::PrintMat<>(3,4,P[0]);
	double nu,nv;
	helper::project(P[0],0,0,0,nu,nv);
	out<<"#Test CV Projection:"<<std::endl;
	out<<"#[0,0,0]->["<<nu<<","<<nv<<"]"<<std::endl;
}

inline void CV_CG_Report(const osg::Camera &camera,
                         std::ostream &out=std::cout)
{
	CV_Report(camera, out);
	CG_Report(camera, out);
}

inline void CG_Project_Report(const osg::Camera &camera,
                              const osg::Vec3Array *v3a, std::ostream &out=std::cout)
{
	out << std::setiosflags(std::ios::scientific);
	for(unsigned int i=0; i<v3a->size(); ++i) {
		const osg::Vec3 &v3 = v3a->at(i);
		double pu,pv;
		CG_Project(camera,v3.x(),v3.y(),v3.z(),pu,pv);
		out << v3.x() << " " << v3.y() << " " << v3.z() << " "
		    << pu << " " << pv << std::endl;
	}
}

}//end of namespace CV2CG
