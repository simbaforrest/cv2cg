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

/* CameraUpdator.h */

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

const unsigned int PICKED_MASK=10;

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
	CameraUpdator(osgViewer::View* v);

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
	{	updating = false; }

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
	{	if(e.valid()) e->setNodeMask(PICKED_MASK); }
	inline void Unlock_Last_Dest_Camera()
	{	if(lastE.valid()) lastE->setNodeMask(1); }


	//map counter n{0,MaxCnt} to time t{0,1}
	double Elapsed_time(int n);
	void Update(osg::Camera* cam);
	void Finish_Update();

	void operator()(osg::Node* node, osg::NodeVisitor* nv);
};