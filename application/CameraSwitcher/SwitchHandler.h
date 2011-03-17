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

/* SwitchHandler.h */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

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
#include <osg/MatrixTransform>

//forward declare
class CameraUpdator;

class SwitchHandler : public osgGA::GUIEventHandler
{
private:
	CameraUpdator* _updator;

	double _dist;
	osg::Vec3 _center;
	bool _lock;
public:
	SwitchHandler(CameraUpdator* updator):_updator(updator),_lock(false) {}

	osg::ref_ptr<osg::Camera> 
		pick_camera(const osgGA::GUIEventAdapter& ea,
		osgViewer::View* viewer);

	bool handle(const osgGA::GUIEventAdapter& ea,
		osgGA::GUIActionAdapter& aa);

	static void printHelp();
};