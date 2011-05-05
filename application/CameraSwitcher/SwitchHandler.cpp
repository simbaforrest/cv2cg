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

/* SwitchHandler.cpp */
#include "SwitchHandler.h"
#include "CameraUpdator.h"

#include "Log.h"
#include "CV2CG.h"

using namespace CV2CG;

//////////////////////////////////////////////////////////
/// API
int nextCamId = 0;

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

		//std::cout<<"Iterate Cam name = "<<cam->getName()<<std::endl;
		if( cam->getName() == idstr ) {
			//std::cout<<"found!"<<std::endl;
			found = true;
			findCam = cam;
		} else {
			traverse(mt);
		}
	}
};

osg::ref_ptr<osg::Camera> get_camera_from_id(int id, osg::Node* root)
{
	//std::cout<<"root name="<<root->getName()<<std::endl;
	std::stringstream ss;
	ss << id;
	std::string strnum;
	ss >> strnum;
	std::string idstr = std::string(std::string("cam")+strnum);
	//std::cout<<"To Find Cam Id string = "<<idstr<<std::endl;
	osg::ref_ptr<CamIDFinder> finder = new CamIDFinder;
	finder->idstr = idstr;
	root->accept(*finder);
	return finder->findCam;
}

//////////////////////////////////////////////////////////
///   SwitchHandler

osg::ref_ptr<osg::Camera>
SwitchHandler::pick_camera(
	const osgGA::GUIEventAdapter& ea,
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
				//if(_updator->Get_Last_Dest_Camera()==storedCam)
				//  continue;
				if(storedCam->getNodeMask()==PICKED_MASK)
					continue;
				return ret = storedCam;
			}//end while
		}//end for
	}//end if
	return ret;
}

bool SwitchHandler::handle(
	const osgGA::GUIEventAdapter& ea,
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
			//hold s and drag : do not change eye position, only change rotation
			if(ea.getKey()=='s') {
				CameraUpdator* updator = dynamic_cast<CameraUpdator*>(
					viewer->getCamera()->getUpdateCallback() );
				osgGA::TrackballManipulator* manip =
					dynamic_cast<osgGA::TrackballManipulator*>(
					viewer->getCameraManipulator() );
				if(_lock || !manip || (updator && updator->IsUpdating()) )
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
#ifndef WIN32//unix
				system("clear");
#else//win32
				system("cls");
#endif
				return true;
			}
			if(ea.getKey()=='h') {
				printHelp();
				return true;
			}
			if(ea.getKey()=='.' || ea.getKey()==',') { //TODO to refine this
				//now updator still have valid s/e cam, no need to pick
				if(_updator->Get_Start_Camera().valid() &&
					_updator->Get_Destination_Camera().valid())
					return true;

				//std::cout<<"Change To Next Camera..."<<std::endl;
				_updator->Set_Start_Camera(new osg::Camera(*viewer->getCamera()));
				osg::Camera* cam = get_camera_from_id(nextCamId, viewer->getSceneData());
				_updator->Set_Destination_Camera( cam );

				if(!_updator->Get_Destination_Camera().valid()) {
					//std::cout<<"No valid destination camera!"<<std::endl;
					_updator->Set_Start_Camera(0);
					_updator->Set_Destination_Camera(0);
					nextCamId = 0; // not found, so next cam is now the first one
					return true;
				}
				ea.getKey()=='.'?++nextCamId:--nextCamId;

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

			LogD("Double clicked...\n");
			_updator->Set_Start_Camera(new osg::Camera(*viewer->getCamera()));
			_updator->Set_Destination_Camera( pick_camera(ea,viewer) );

			if(!_updator->Get_Destination_Camera().valid()) {
				LogD("No valid destination camera!\n");
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

void SwitchHandler::printHelp()
{
	//system("cls");
	LogI("\n\tWelcome to use CameraSwitcher!\n"
		"Usage:\n"
		"\tDOUBLE CLICK : double click on cameras(cam0,cam1,...),"
		     "can smoothly change your viewpoints to that camera.\n"
		"\tHold s and DRAG : do not change eye position, only rotate camera.\n"
		"\tPress UP,DOWN,LEFT,RIGHT : change principle point\n"
		"\tPress PageUp,PageDown : scale image\n"
		"\tPress p : print to console information of current camera\n"
		"\tPress c : clear concole\n"
		"\tPress h : print help information to console\n"
		"\tPress , or . : switch between cameras\n");
}
