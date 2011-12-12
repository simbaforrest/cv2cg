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

/* SwitchCameraHandler.h */

#include <osg/ref_ptr>
#include <osg/Group>
#include <osgViewer/Viewer>
#include <osg/NodeVisitor>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/NodeCallback>
#include <osgGA/GUIEventHandler>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>

#include <iostream>

namespace SwitchCameraHelper {

const unsigned int PICKED_MASK=10;

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
inline void Interpolate_Camera(
	const osg::Camera& s,
	const osg::Camera& e,
	const double t,
	osg::Camera& camera)
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

class CameraUpdator : public osg::NodeCallback
{
private:
	int MaxCnt;
	osgViewer::View* viewer;
	osg::ref_ptr<osgGA::TrackballManipulator> manip;
	osg::ref_ptr<osg::Camera> s;
	osg::ref_ptr<osg::Camera> e;
	osg::ref_ptr<osg::Camera> lastE;
	bool updating;
	int cnt;
public:
	CameraUpdator(osgViewer::View* v) {
		if(!v) {
			std::cout<<"[CameraUpdator] invalid "
				"osgViewer::View, exit(-1)..."<<std::endl;
			exit(-1);
		}
		viewer = v;
		manip = new osgGA::TrackballManipulator;
		viewer->setCameraManipulator(manip);
		MaxCnt = 100;
		updating = false;
		cnt=0;
		s=e=0;
		lastE=0;
	}

	inline bool isUpdating() { return updating; }

	inline bool startUpdate()
	{
		cnt = 0;
		if( s.valid() && e.valid() ) {
//			std::cout<<"Begin Update........................\r";
			updating = true;
			return true;
		}
		return false;
	}
	inline void stopUpdate()
	{	updating = false; }

	inline void setStartCamera(osg::Camera* s_)
	{ s = s_; }
	inline osg::ref_ptr<osg::Camera> getStartCamera()
	{ return s; }

	inline void setDestCamera(osg::Camera* e_)
	{ e = e_; }
	inline osg::ref_ptr<osg::Camera> getDestCamera()
	{ return e; }

	inline osg::ref_ptr<osg::Camera> getLastDestCamera()
	{ return lastE; }

	inline void lockDestCamera()
	{	if(e.valid()) e->setNodeMask(PICKED_MASK); }
	inline void unlockLastDestCamera()
	{	if(lastE.valid()) lastE->setNodeMask(1); }


	//map counter n{0,MaxCnt} to time t{0,1}
	inline double elapsedTime(int n) {
#if 0
		// linearly elapsed time
		// same speed
		static const double deno = 1.0/MaxCnt;
		return deno * n;
#elif 0
		// parabolically elapsed time
		// first slow, last fast
		static const double MaxDeno = 1.0/(MaxCnt*MaxCnt);
		return MaxDeno * n * n;
#elif 1
		// cubically elapsed time
		// first slow, then fast, last slow
		static const double a = 3.0/MaxCnt/MaxCnt;
		static const double b = -2.0/MaxCnt/MaxCnt/MaxCnt;
		double n2 = n*n;
		return a*n2+b*n2*n;
#endif
	}

	inline void update(osg::Camera* cam) {
		if(!cam) return;
		double t = elapsedTime(cnt);
//		std::cout<<"Interpolating... counter="<<cnt<<" time="<<t<<"\r";
		Interpolate_Camera(*s,*e, t,*cam);
		++cnt;
	}

	inline void finishUpdate() {
//		std::cout<<".............................Finish Update"<<std::endl;
		if(!viewer) return;
		stopUpdate();
		osg::Vec3 eye,center,up;
		e->getViewMatrixAsLookAt(eye,center,up);
		manip->setHomePosition(eye, center, up);
		viewer->setCameraManipulator(manip);

		lastE = e;
		s = 0;
		e = 0;
	}

	inline void operator()(osg::Node* node, osg::NodeVisitor* nv) {
		osg::Camera* cam = dynamic_cast<osg::Camera*>(node);
		if(cam && updating) {
			if( !(s.valid() && e.valid()) ) {
				viewer->setCameraManipulator(manip);
				stopUpdate();
			} else {
				if(cnt>MaxCnt || e==lastE)
					finishUpdate();
				else
					update(cam);
			}
		}
		traverse(node, nv);
	}
};//enf of class CameraUpdator

class CameraFrustumCollector : public osg::NodeVisitor {
	std::vector<osg::Camera*>* storage;
public:
	CameraFrustumCollector(std::vector<osg::Camera*>* storage) :
		osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {
		this->storage = storage;
		storage->clear();
	}

	inline void apply(osg::MatrixTransform& mt) {
		if(!storage) return;
		if(mt.getName().find("frustum")==std::string::npos) return;
		osg::Camera* cam = dynamic_cast<osg::Camera*>(mt.getUserData());
		if(!cam) return;
		storage->push_back(cam);
		traverse(mt);
	}
};//end of CameraFrustumCollector

class SwitchCameraHandler : public osgGA::GUIEventHandler
{
private:
	CameraUpdator* _updator;

	double _dist;
	osg::Vec3 _center;
	bool _lock;

	bool cameraScanned;
	std::vector<osg::Camera*> camList;
	int nextCamID;
public:
	SwitchCameraHandler(CameraUpdator* updator):_updator(updator),_lock(false)
	{ cameraScanned = false; nextCamID = 0; }

	inline osg::ref_ptr<osg::Camera> 
		pick_camera(const osgGA::GUIEventAdapter& ea,
		osgViewer::View* viewer)
	{
		osg::Camera* ret=0;

		double x = ea.getXnormalized();
		double y = ea.getYnormalized();

		osgUtil::PolytopeIntersector* picker =
			new osgUtil::PolytopeIntersector(
			osgUtil::Intersector::PROJECTION,
			x-0.01, y-0.01, x+0.01, y+0.01 );

		osgUtil::IntersectionVisitor iv( picker );
		viewer->getCamera()->accept( iv );

		if (picker->containsIntersections())
		{
			osgUtil::PolytopeIntersector::Intersections& intersections =
				picker->getIntersections();
			osgUtil::PolytopeIntersector::Intersections::iterator itr =
				intersections.begin();
			for(; itr!=intersections.end(); ++itr) {
				const osgUtil::PolytopeIntersector::Intersection& intersection =
					(*itr);//picker->getFirstIntersection();

				const osg::NodePath& nodePath =
					intersection.nodePath;
				unsigned int idx = nodePath.size();
				osg::MatrixTransform* sel=NULL;
				while (idx--)	{
					// find the last one in nodepath of intersection
					sel =	dynamic_cast<osg::MatrixTransform*>( nodePath[ idx ] );
					if (sel == NULL)
						continue;

					osg::Camera* storedCam = dynamic_cast<osg::Camera*>(sel->getUserData());
					if(storedCam==NULL)
						continue;
					//if(_updator->getLastDestCamera()==storedCam)
					//  continue;
					if(storedCam->getNodeMask()==PICKED_MASK)
						continue;
					return ret = storedCam;
				}//end while
			}//end for
		}//end if
		return ret;
	}

	inline bool handle(const osgGA::GUIEventAdapter& ea,
		osgGA::GUIActionAdapter& aa,
		osg::Object*, osg::NodeVisitor*)
	{
		osgViewer::View* viewer =
			dynamic_cast<osgViewer::View*>( &aa );
		if (!viewer || ea.getHandled() || !_updator ) return false;
		if(!cameraScanned) { //find camera list
			cameraScanned=true;
			osg::ref_ptr<CameraFrustumCollector> collector = 
				new CameraFrustumCollector(&camList);
			osg::Node* root = viewer->getSceneData();
			if(root) {
				root->accept(*collector);
			}
		}

		switch(ea.getEventType()) {
		case(osgGA::GUIEventAdapter::KEYDOWN): {
			//hold s and drag : do not change eye position, only change rotation
			if(ea.getKey()=='s') {
				CameraUpdator* updator = dynamic_cast<CameraUpdator*>(
					viewer->getCamera()->getUpdateCallback() );
				osgGA::TrackballManipulator* manip =
					dynamic_cast<osgGA::TrackballManipulator*>(
					viewer->getCameraManipulator() );
				if(_lock || !manip || (updator && updator->isUpdating()) )
	                return true;
				_lock = true;
				_dist = manip->getDistance();
				_center = manip->getCenter();
				manip->setDistance(0);
				osg::Vec3 eye,cen,up;
				viewer->getCamera()->getViewMatrixAsLookAt(eye, cen, up);
				manip->setCenter(eye);
				return true;
			}

			double l,r,b,t,n,f;
			viewer->getCamera()->getProjectionMatrixAsFrustum(l,r,b,t,n,f);
			double dw,dh,dd;
			bool bmodified = false;
			dd = 0.01;
			dw = dd*fabs(r-l); dh = dd*fabs(t-b);
			switch(ea.getKey()) {
			case(osgGA::GUIEventAdapter::KEY_Up):
				t+=dh;b+=dh; bmodified = true; break;
			case(osgGA::GUIEventAdapter::KEY_Down):
				b-=dh;t-=dh; bmodified = true; break;
			case(osgGA::GUIEventAdapter::KEY_Left):
				l+=dw;r+=dw; bmodified = true; break;
			case(osgGA::GUIEventAdapter::KEY_Right):
				l-=dw;r-=dw; bmodified = true; break;
			case(osgGA::GUIEventAdapter::KEY_Page_Up):
				t+=dh;b-=dh;l-=dw;r+=dw; bmodified = true; break;
			case(osgGA::GUIEventAdapter::KEY_Page_Down):
				t-=dh;b+=dh;l+=dw;r-=dw; bmodified = true; break;
			}
			viewer->getCamera()->setProjectionMatrixAsFrustum(l,r,b,t,n,f);
			if(bmodified) return true;
			break;}//end of KEYDOWN
		case(osgGA::GUIEventAdapter::KEYUP): {
			switch(ea.getKey()) {
			case 's': {
				CameraUpdator* updator = dynamic_cast<CameraUpdator*>(
					viewer->getCamera()->getUpdateCallback() );
				osgGA::TrackballManipulator* manip =
					dynamic_cast<osgGA::TrackballManipulator*>(
					viewer->getCameraManipulator() );
				if(!manip || (updator && updator->isUpdating()) ) return true;
				_lock = false;
				manip->setDistance(_dist);
				manip->setCenter(_center);
				viewer->home();
				return true;}
			case 'r':
				CV2CG::CV_CG_Report(*(viewer->getCamera()));
				return true;
			case 'c':
#ifndef WIN32//unix
				system("clear");
#else//win32
				system("cls");
#endif
				return true;
			case 'h':
				printHelp();
				return false;
			case '.':
			case ',': {
				//now updator still have valid s/e cam, no need to pick
				if(_updator->getStartCamera().valid() &&
					_updator->getDestCamera().valid())
					return true;

				if((int)camList.size()<=1) return true; //only one camera

				int mv = ea.getKey()=='.'?1:-1;
				nextCamID = (nextCamID+mv+camList.size())%camList.size();
				//std::cout<<"Change To Next Camera..."<<std::endl;
				_updator->setStartCamera(new osg::Camera(*viewer->getCamera()));
				osg::Camera* cam = camList[nextCamID];
				_updator->setDestCamera( cam );

				if(!_updator->getDestCamera().valid()) {
					//std::cout<<"No valid destination camera!"<<std::endl;
					_updator->setStartCamera(0);
					_updator->setDestCamera(0);
					nextCamID = 0; // not found, so next cam is now the first one
					return true;
				}

				viewer->setCameraManipulator(0);
				_updator->lockDestCamera();//prevent pick again
				_updator->unlockLastDestCamera();//release last pick;
				_updator->startUpdate();
				return true;}
			}
			break;}//end of KEYUP
		case(osgGA::GUIEventAdapter::DOUBLECLICK): {//TODO why no response here?
			loglni("here");
			//now updator still have valid s/e cam, no need to pick
			if(_updator->getStartCamera().valid() &&
				_updator->getDestCamera().valid())
				return true;

			LogD("Double clicked...\n");
			_updator->setStartCamera(new osg::Camera(*viewer->getCamera()));
			_updator->setDestCamera( pick_camera(ea,viewer) );

			if(!_updator->getDestCamera().valid()) {
				LogD("No valid destination camera!\n");
				_updator->setStartCamera(0);
				_updator->setDestCamera(0);
				return true;
			}

			viewer->setCameraManipulator(0);
			_updator->lockDestCamera();//prevent pick again
			_updator->unlockLastDestCamera();//release last pick;
			_updator->startUpdate();
			return true;}//end of DOUBLECLICK
		}//end of switch
		return false;
	}//end of handle

	static void printHelp()
	{
		LogI("\nUsage(SwitchCameraHandler):\n"
			"\tDOUBLE CLICK : double click on cameras(cam0,cam1,...),"
				 "can smoothly change your viewpoints to that camera.\n"
			"\tHold s and DRAG : do not change eye position, only rotate camera.\n"
			"\tPress UP,DOWN,LEFT,RIGHT : change principle point\n"
			"\tPress PageUp,PageDown : scale image\n"
			"\tPress r : print to console information of current camera\n"
			"\tPress c : clear concole\n"
			"\tPress h : print help information to console\n"
			"\tPress , or . : switch between cameras\n");
	}
};//end of class SwitchCameraHandler

}//end of namespace SwitchCameraHelper
