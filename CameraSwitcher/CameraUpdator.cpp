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

/* CameraUpdator.cpp */

#include "CameraUpdator.h"

///////////////////////////////////////////////////
///   API
template<class Type>
inline Type Interpolate(const Type&s, const Type& e, double t)
{
#if 1
	return s*(1-t)+e*t; //linear
#else
	t -= 1;
	t *= t;
	return e+(s-e)*t; //parabola
#endif
}

//t belongs to [0,1]
void Interpolate_Camera(const osg::Camera& s, const osg::Camera& e,
												const double t, osg::Camera& camera)
{
	//view matrix
	const osg::Matrix& vms = s.getViewMatrix();
	const osg::Matrix& vme = e.getViewMatrix();
	osg::Quat qs = vms.getRotate();
	osg::Quat qe = vme.getRotate();
	osg::Vec3 transS = vms.getTrans();
	osg::Vec3 transE = vme.getTrans();

	osg::Quat q;
	osg::Vec3 trans;
	q.slerp(t, qs, qe);
	trans = Interpolate(transS,transE,t);
	osg::Matrix vmat;
	vmat.setRotate(q);
	vmat.setTrans(trans);
	camera.setViewMatrix(vmat);

	//proj matrix
	double ls,rs,bs,ts,ns,fs;
	s.getProjectionMatrixAsFrustum(ls,rs,bs,ts,ns,fs);
	double le,re,be,te,ne,fe;
	e.getProjectionMatrixAsFrustum(le,re,be,te,ne,fe);
	double lt,rt,bt,tt,nt,ft;
	lt = Interpolate(ls,le,t);
	rt = Interpolate(rs,re,t);
	bt = Interpolate(bs,be,t);
	tt = Interpolate(ts,te,t);
	nt = Interpolate(ns,ne,t);
	ft = Interpolate(fs,fe,t);
	camera.setProjectionMatrixAsFrustum(lt,rt,bt,tt,nt,ft);
}

///////////////////////////////////////////////////
///    CameraUpdator

CameraUpdator::CameraUpdator(osgViewer::View* v) {
	viewer = v;
	manip = new osgGA::TrackballManipulator;
	viewer->setCameraManipulator(manip);
	MaxCnt = 100;
	updating = false;
	cnt=0;
	s=e=0;
	lastE=0;
}

//map counter n{0,MaxCnt} to time t{0,1}
double CameraUpdator::Elapsed_time(int n)
{
#define CUBICALLY_ELAPSE

#ifdef LINEARLY_ELAPSE
	// linearly elapsed time
	// same speed
	static const double deno = 1.0/MaxCnt;
	return deno * n;
#elif defined PARABOLICALLY_ELAPSE
	// parabolically elapsed time
	// first slow, last fast
	static const double MaxDeno = 1.0/(MaxCnt*MaxCnt);
	return MaxDeno * n * n;
#elif defined CUBICALLY_ELAPSE
	// cubically elapsed time
	// first slow, then fast, last slow
	static const double a = 3.0/MaxCnt/MaxCnt;
	static const double b = -2.0/MaxCnt/MaxCnt/MaxCnt;
	double n2 = n*n;
	return a*n2+b*n2*n;
#endif
}


void CameraUpdator::Update(osg::Camera* cam)
{
	if(!cam) return;
	double t = Elapsed_time(cnt);
	std::cout<<"Interpolating... counter="<<cnt<<" time="<<t<<"\r";
	Interpolate_Camera(*s,*e, t,*cam);
	++cnt;
}

void CameraUpdator::Finish_Update()
{
	std::cout<<".............................Finish Update"<<std::endl;
	Stop_Update();
	osg::Vec3 eye,center,up;
	e->getViewMatrixAsLookAt(eye,center,up);
	manip->setHomePosition(eye, center, up);
	viewer->setCameraManipulator(manip);
	viewer->getCamera()->setUserData( e->getUserData() );//

	lastE = e;
	s = 0;
	e = 0;
}

void CameraUpdator::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	osg::Camera* cam = dynamic_cast<osg::Camera*>(node);
	if(cam && updating) {
		if( !(s.valid() && e.valid()) ) {
			viewer->setCameraManipulator(manip);
			Stop_Update();
		} else {
			if(cnt>MaxCnt || e==lastE)
				Finish_Update();
			else
				Update(cam);
		}
	}
	traverse(node, nv);
}