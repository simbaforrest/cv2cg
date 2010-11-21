#pragma once
/* 
 *  Copyright (c) 2010  Chen Feng (cforrest (at) umich.edu)
 *    and the University of Michigan
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

/* CV_CG.h */

#include <iomanip>
#include "CameraAlgebra.h"

//Statement:
//All notation such as K,R,C,T,P are in standard computer vision system
//details in <<Multiple View Geometry>> by Richard Hartley and A. Zisserman

//this actually use a osg camera to get cv photo coordinate
inline void CG_Project(const osg::Camera& camera, 
											 double x, double y, double z, double& u, double& v)
{
	osg::Vec3 wp(x,y,z);
	osg::Vec3 ep = wp * camera.getViewMatrix();
	double l,r,b,t,n,f;
	camera.getProjectionMatrixAsFrustum(l,r,b,t,n,f);
	const osg::Viewport* vp = camera.getViewport();
	double x0 = vp->x(), y0 = vp->y(), w = vp->width(), h = vp->height();
	double xe=ep.x(), ye=ep.y(), ze=ep.z();
	u = -n*w*xe/((r-l)*ze)-w*(r+l)/((r-l)*2)+x0+w/2;
	v = -n*h*ye/((t-b)*ze)-h*(t+b)/((t-b)*2)+y0+h/2;
	v = h-v; // be careful, this change the photo coordinate system to computer vision
}

inline void Camera_Decomposition(const osg::Camera& camera,
	double K[3][3], double C[3], double R[3][3])
{
	double l,r,b,t,n,f;
	camera.getProjectionMatrixAsFrustum(l,r,b,t,n,f);
	const osg::Viewport* vp = camera.getViewport();
	double x0 = vp->x(), y0 = vp->y(), w = vp->width(), h = vp->height();

	//K
	//[ alphaX    0     u0 ]
	//[   0     alphaY  v0 ]
	//[   0       0      1 ]
	MatrixManip::Zeros(3,3,K[0]);
	K[0][0] = n*w/(r-l); //alphaX
	K[1][1] = n*h/(t-b); //alphaY
	K[0][2] = -w*(r+l)/((r-l)*2)+x0+w/2;//u0
	K[1][2] = h*(t+b)/((t-b)*2)-y0+h/2;//v0
	K[2][2] = 1;

	osg::Vec3 center,eye,up;
	camera.getViewMatrixAsLookAt(eye, center, up);
	C[0]=eye.x(); C[1]=eye.y(); C[2]=eye.z();

	osg::Matrix vmat = camera.getViewMatrix();
	R[0][0] = vmat(0,0); R[0][1] = vmat(1,0); R[0][2] = vmat(2,0);
	R[1][0] = -vmat(0,1); R[1][1] = -vmat(1,1); R[1][2] = -vmat(2,1);
	R[2][0] = -vmat(0,2); R[2][1] = -vmat(1,2); R[2][2] = -vmat(2,2);
}

inline void Camera_Composition(const double K[3][3], const double C[3],
	const double R[3][3], const double n, const double f,
	const osg::Viewport* vp, osg::Camera& camera)
{
	//intrinsic
	double ax=K[0][0], ay=K[1][1], u0=K[0][2], v0=K[1][2];
	double l,r,t,b;

	double x0,y0,w,h;
	if(vp) {
		x0 = vp->x(), y0 = vp->y(), w = vp->width(), h = vp->height();
	} else {
		const osg::Viewport* vpi = camera.getViewport();
		if(!vpi) {
			osg::notify(osg::NOTICE)<<
				"No Viewport available! Camera_Composition Failed!"
				<<std::endl;
			return;
		}
		x0 = vpi->x(), y0 = vpi->y(), w = vpi->width(), h = vpi->height();
	}

	r = ( n*w*0.5 - n*(u0-x0-w*0.5) ) / ax;
	l = r - n*w/ax;
	t = ( n*h*0.5 - n*(-v0-y0+h*0.5) ) / ay;
	b = t - n*h/ay;

	camera.setProjectionMatrixAsFrustum(l,r,b,t,n,f);
	osg::ref_ptr<osg::Viewport> vpn = new osg::Viewport(x0,y0,w,h);
	camera.setViewport(vpn);

	//extrinsic
	double T[3];
	MatrixManip::Product331(R[0],C,T);
	T[0]*=-1;T[1]*=-1;T[2]*=-1;
	osg::Matrix vmat(
		R[0][0], -R[1][0], -R[2][0], 0,
		R[0][1], -R[1][1], -R[2][1], 0,
		R[0][2], -R[1][2], -R[2][2], 0,
		T[0], -T[1], -T[2], 1            //ATTENTION! should set as (T0,-T1,-T2)
		);
	camera.setViewMatrix(vmat);
}

inline std::ostream& operator<<(std::ostream& os, const osg::Matrix& mat)
{
	os << setiosflags(ios::scientific);
	for(int i=0; i<4; ++i) {
		for(int j=0; j<4; ++j)
			os << mat(i,j) << "\t";
		os << std::endl;
	}
	return os;
}

inline std::ostream& operator<<(std::ostream& os, const osg::Camera& camera)
{
	os << setiosflags(ios::scientific);
	osg::Matrix vmat = camera.getViewMatrix();
	osg::Matrix pmat = camera.getProjectionMatrix();
	os << "View Matrix = " << std::endl;
	os << vmat ;
	os << "Projection Matrix = " << std::endl;
	os << pmat ;
	double l,r,b,t,n,f;
	camera.getProjectionMatrixAsFrustum(l,r,b,t,n,f);
	os << 
		"l=" << l << " r=" << r <<
		" b=" << b << " t=" << t << 
		" n=" << n << " f=" << f <<std::endl;
	const osg::Viewport& viewport = *(camera.getViewport());
	os << "x=" << viewport.x() << " y=" << viewport.y()
		<< " w=" << viewport.width() << " h=" << viewport.height() << std::endl;
	return os;
}

inline void CG_Report(const osg::Camera& camera, 
	std::ostream& out=std::cout)
{
	out<<"-------------Graphics--------------"<<std::endl;
	out<<camera<<std::endl;
	out<<"Test CG Projection:"<<std::endl;
	double u,v;
	CG_Project(camera, 0, 0, 0, u, v);
	out<<"[0,0,0]->["<<u<<","<<v<<"]"<<std::endl;
}

inline void CV_Report(const osg::Camera& camera, 
	std::ostream& out=std::cout)
{
	out<<"-------------Vision----------------"<<std::endl;
	double K[3][3],C[3],T[3],R[3][3],P[3][4];
	Camera_Decomposition(camera,K,C,R);
	CameraAlgebra::Compose(K[0],C,R[0],P[0]);
	MatrixManip::Print(3,4,P[0],"P");
	MatrixManip::Product331(R[0], C, T);
	T[0]*=-1;T[1]*=-1;T[2]*=-1;

	out<<"K(alphaX alphaY u0 v0)="<<std::endl,
	out<<K[0][0]<<"\n"<<K[1][1]<<"\n"<<K[0][2]<<"\n"<<K[1][2]<<std::endl;
	MatrixManip::Print(
		out<<"R="<<std::endl,
		R[0],3,3);
	MatrixManip::Print(
		out<<"C="<<std::endl,
		C,3,1);
	MatrixManip::Print(
		out<<"T="<<std::endl,
		T,3,1);
	MatrixManip::Print(
		out<<"P="<<std::endl,
		P[0],3,4)<<std::endl;
	double nu,nv;
	CameraAlgebra::Project(P[0], 0,0,0,nu,nv);
	out<<"Test CV Projection:"<<std::endl;
	out<<"[0,0,0]->["<<nu<<","<<nv<<"]"<<std::endl;
}

inline void CV_CG_Report(const osg::Camera& camera, 
	std::ostream& out=std::cout)
{
	CG_Report(camera, out);
	CV_Report(camera, out);
}

inline void CG_Project_Report(const osg::Camera& camera, 
	const osg::Vec3Array* v3a, std::ostream& out=std::cout)
{
	out << setiosflags(ios::scientific);
	for(unsigned int i=0; i<v3a->size(); ++i) {
		const osg::Vec3& v3 = v3a->at(i);
		double pu,pv;
		CG_Project(camera,v3.x(),v3.y(),v3.z(),pu,pv);
		out << v3.x() << " " << v3.y() << " " << v3.z() << " "
			<< pu << " " << pv << std::endl;
	}
}
