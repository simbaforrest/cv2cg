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

/* CameraSwitcher.cpp */

#include "CameraSwitcher.h"

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

#include "SwitcherAPI.h"

void printHelp()
{
	system("cls");
#ifdef USE_IN_CHINA
	printf("\n\t欢迎使用相机浏览器——CameraSwitcher！\n");
	printf("使用方法：\n");
	printf("\t鼠标双击：使用鼠标双击相机（cam0,cam1,...）,即可平滑的切换到该相机所代表的视角.\n");
	printf("\t按住s键不放拖动鼠标：视点位置不变，只改变相机姿态\n");
	printf("\t按方向键 上下左右：改变像主点位置\n");
	printf("\t按PageUp，PageDown：放大缩小\n");
	printf("\t按p键：在控制台打印当前相机的位置与姿态信息\n");
	printf("\t按c键：控制台清屏\n");
	printf("\t按h键：在控制台打印帮助信息\n");
	printf("\t按=键：按顺序切换相机\n");
#endif
}

const unsigned int PICKED_MASK=10;
int camId = -1;

class CamIDFinder : public osg::NodeVisitor
{
public:
	CamIDFinder() : osg::NodeVisitor(
		osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {
			findCam = 0;
			found = false;
	}
	osg::Camera* findCam;
	std::string idstr;
	bool found;

	virtual void apply(osg::MatrixTransform& mt)
	{
		osg::Camera* cam = dynamic_cast<osg::Camera*>(mt.getUserData());
		if(found || !cam) {
			traverse(mt);
			return;
		}

		std::cout<<"Iterate Cam name = "<<cam->getName()<<std::endl;
		if( cam->getName() == idstr ) {
			std::cout<<"found!"<<std::endl;
			found = true;
			findCam = cam;
		} else {
			traverse(mt);
		}
	}
};

osg::ref_ptr<osg::Camera> get_camera_from_id(int id, osg::Node* root)
{
	std::cout<<"root name="<<root->getName()<<std::endl;
	std::stringstream ss;
	ss << id;
	std::string strnum;
	ss >> strnum;
	std::string idstr = std::string(std::string("cam")+strnum);
	std::cout<<"To Find Cam Id string = "<<idstr<<std::endl;
	osg::ref_ptr<CamIDFinder> finder = new CamIDFinder;
	finder->idstr = idstr;
	root->accept(*finder);
	return finder->findCam;
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
		viewer = v;
		manip = new osgGA::TrackballManipulator;
		viewer->setCameraManipulator(manip);
		MaxCnt = 100;
		updating = false;
		cnt=0;
		s=e=0;
		lastE=0;
	}

	inline bool IsUpdating() { return updating; }

	inline bool Start_Update()
	{
		cnt = 0;
		if( s.valid() && e.valid() ) {
			std::cout<<"Begin Update"<<std::endl;
			updating = true;
			return true;
		}
		return false;
	}
	inline void Stop_Update()
	{
		updating = false;
	}

	inline void Set_Start_Camera(osg::Camera* s_)
	{ s = s_; }
	inline osg::ref_ptr<osg::Camera> Get_Start_Camera()
	{ return s; }

	inline void Set_Destination_Camera(osg::Camera* e_)
	{ e = e_; }
	inline osg::ref_ptr<osg::Camera> Get_Destination_Camera()
	{ return e; }

	inline osg::ref_ptr<osg::Camera> Get_Last_Dest_Camera()
	{ return lastE; }

	inline void Lock_Dest_Camera()
	{
		if(e.valid()) e->setNodeMask(PICKED_MASK);
	}
	inline void Unlock_Last_Dest_Camera()
	{
		if(lastE.valid()) lastE->setNodeMask(1);
	}


#define CUBICALLY_ELAPSE
	//map counter n{0,MaxCnt} to time t{0,1}
	inline double Elapsed_time(int n)
	{
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

	inline void Update(osg::Camera* cam)
	{
		if(!cam) return;
		double t = Elapsed_time(cnt);
		std::cout<<"Interpolating... counter="<<cnt<<" time="<<t<<std::endl;
		Interpolate_Camera(*s,*e, t,*cam);
		++cnt;
	}

	inline void Finish_Update()
	{
		std::cout<<"Finish Update"<<std::endl;
		Stop_Update();
		osg::Vec3 eye,center,up;
		e->getViewMatrixAsLookAt(eye,center,up);
		manip->setHomePosition(eye, center, up);
		viewer->setCameraManipulator(manip);

		lastE = e;
		s = 0;
		e = 0;
	}

	void operator()(osg::Node* node, osg::NodeVisitor* nv)
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
};

class SwitcherHandler : public osgGA::GUIEventHandler
{
private:
	CameraUpdator* _updator;

	double _dist;
	osg::Vec3 _center;
	bool _lock;
public:
	SwitcherHandler(CameraUpdator* updator):_updator(updator),_lock(false) {}

	osg::ref_ptr<osg::Camera> 
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
				osgUtil::PolytopeIntersector::Intersection& intersection = 
					(*itr);//picker->getFirstIntersection();

				osg::NodePath& nodePath =
					intersection.nodePath;
				unsigned int idx = nodePath.size();
				osg::MatrixTransform* sel=NULL;
				while (idx--)	{
					// 查找交集节点路径中的最后一个
					sel =	dynamic_cast<osg::MatrixTransform*>( nodePath[ idx ] );
					if (sel == NULL)
						continue;

					osg::Camera* storedCam = dynamic_cast<osg::Camera*>(sel->getUserData());
					if(storedCam==NULL)
						continue;
					// 				if(_updator->Get_Last_Dest_Camera()==storedCam)
					// 					continue;
					if(storedCam->getNodeMask()==PICKED_MASK)
						continue;
					return ret = storedCam;
				}//end while
			}//end for
		}//end if
		return ret;
	}

	bool handle(const osgGA::GUIEventAdapter& ea,
		osgGA::GUIActionAdapter& aa)
	{
		osgViewer::View* viewer =
			dynamic_cast<osgViewer::View*>( &aa );
		if (!viewer ||
			ea.getHandled() ||
			!_updator ) return false;

		switch(ea.getEventType())
		{
		case(osgGA::GUIEventAdapter::KEYDOWN):
			{
				if(ea.getKey()=='s') {
					CameraUpdator* updator = dynamic_cast<CameraUpdator*>(
						viewer->getCamera()->getUpdateCallback() );
					osgGA::TrackballManipulator* manip = 
						dynamic_cast<osgGA::TrackballManipulator*>(
						viewer->getCameraManipulator() );
					if(_lock || !manip || (updator && updator->IsUpdating()) ) return true;
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
				if(ea.getKey()==osgGA::GUIEventAdapter::KEY_Up) {
					t+=dh;b+=dh;
					bmodified = true;
				} else if(ea.getKey()==osgGA::GUIEventAdapter::KEY_Down) {
					b-=dh;t-=dh;
					bmodified = true;
				} else if(ea.getKey()==osgGA::GUIEventAdapter::KEY_Left) {
					l+=dw;r+=dw;
					bmodified = true;
				} else if(ea.getKey()==osgGA::GUIEventAdapter::KEY_Right) {
					l-=dw;r-=dw;
					bmodified = true;
				} else if(ea.getKey()==osgGA::GUIEventAdapter::KEY_Page_Up) {
					t+=dh;b-=dh;l-=dw;r+=dw;
					bmodified = true;
				} else if(ea.getKey()==osgGA::GUIEventAdapter::KEY_Page_Down) {
					t-=dh;b+=dh;l+=dw;r-=dw;
					bmodified = true;
				}
				viewer->getCamera()->setProjectionMatrixAsFrustum(l,r,b,t,n,f);
				if(bmodified) return true;
				break;
			}
		case(osgGA::GUIEventAdapter::KEYUP):
			{
				if(ea.getKey()=='s') {
					CameraUpdator* updator = dynamic_cast<CameraUpdator*>(
						viewer->getCamera()->getUpdateCallback() );
					osgGA::TrackballManipulator* manip = 
						dynamic_cast<osgGA::TrackballManipulator*>(
						viewer->getCameraManipulator() );
					if(!manip || (updator && updator->IsUpdating()) ) return true;
					_lock = false;
					manip->setDistance(_dist);
					manip->setCenter(_center);
					viewer->home();
					return true;
				}
				if(ea.getKey()=='p') {
					CV_CG_Report(*(viewer->getCamera()));
					return true;
				}
				if(ea.getKey()=='c') {
					system("cls");
					return true;
				}
				if(ea.getKey()=='h') {
					printHelp();
					return true;
				}
				if(ea.getKey()=='=') {
					++camId;
					//now updator still have valid s/e cam, no need to pick
					if(_updator->Get_Start_Camera().valid() &&
						_updator->Get_Destination_Camera().valid())
						return true;

					std::cout<<"Change To Next Camera..."<<std::endl;
					_updator->Set_Start_Camera(new osg::Camera(*viewer->getCamera()));
					osg::Camera* cam = get_camera_from_id(camId, viewer->getSceneData());
					_updator->Set_Destination_Camera( cam );

					if(!_updator->Get_Destination_Camera().valid()) {
						std::cout<<"No valid destination camera!"<<std::endl;
						_updator->Set_Start_Camera(0);
						_updator->Set_Destination_Camera(0);
						camId = -1;
						return true;
					}

					viewer->setCameraManipulator(0);
					_updator->Lock_Dest_Camera();//prevent pick again
					_updator->Unlock_Last_Dest_Camera();//release last pick;
					_updator->Start_Update();
					return true;
				}
				break;
			}
		case(osgGA::GUIEventAdapter::DOUBLECLICK):
			{
				//now updator still have valid s/e cam, no need to pick
				if(_updator->Get_Start_Camera().valid() &&
					_updator->Get_Destination_Camera().valid())
					return true;

				std::cout<<"Double clicked..."<<std::endl;
				_updator->Set_Start_Camera(new osg::Camera(*viewer->getCamera()));
				_updator->Set_Destination_Camera( pick_camera(ea,viewer) );

				if(!_updator->Get_Destination_Camera().valid()) {
					std::cout<<"No valid destination camera!"<<std::endl;
					_updator->Set_Start_Camera(0);
					_updator->Set_Destination_Camera(0);
					return true;
				}

				viewer->setCameraManipulator(0);
				_updator->Lock_Dest_Camera();//prevent pick again
				_updator->Unlock_Last_Dest_Camera();//release last pick;
				_updator->Start_Update();
				return true;
			}
		}//end of switch
		return false;
	}
};

int CameraSwitcher::run()
{
	osg::ref_ptr<osg::Node> scene =
		createSceneFromFile(infilename);
	if(writeModel)
		osgDB::writeNodeFile(*scene,
			infilename+std::string("-model.osg"));

	printHelp();

	osgViewer::Viewer viewer;
	viewer.getCamera()->setClearColor(osg::Vec4(0.1f,0.1f,0.3f,0.0f));
	viewer.setSceneData(scene);
	CameraUpdator* cu = new CameraUpdator(&viewer);
	SwitcherHandler* sh = new SwitcherHandler(cu);
	viewer.getCamera()->setUpdateCallback(cu);
	viewer.addEventHandler(sh);
	viewer.getCamera()->setComputeNearFarMode(
		osg::Camera::DO_NOT_COMPUTE_NEAR_FAR);

	viewer.addEventHandler( new osgGA::StateSetManipulator(
		viewer.getCamera()->getOrCreateStateSet()) );
	viewer.addEventHandler(new osgViewer::StatsHandler);

	return viewer.run();
}
