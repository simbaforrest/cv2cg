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

#include <iostream>
#include <string>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>

#include "Log.h"
#include "CreateSceneHelper.h"
#include "CaptureSceneHelper.h"
#include "SwitchCameraHelper.h"

Log::Level Log::level = Log::LOG_INFO;

int main( int argc, char **argv )
{
	if(argc<=1) {
		LogI("Usage: \n\tCameraSimulator <InputFileName>"
			" [OutPut Dir]\n");
		LogI("Example: \n\tCameraSimulator draw.txt D:/Out/\n");
		LogI("Example: \n\tCameraSimulator draw.txt\n");
		return -1;
	}

	std::string inname(argv[1]);
#ifdef WIN32
	std::string outdir(".\\");
#else
	std::string outdir("./");
#endif

	if(argc>2) outdir = std::string(argv[2]);
	helper::legalDir(outdir);

	osgViewer::Viewer viewer;
	osg::ref_ptr<osg::Node> md = CreateSceneHelper::create_SCENE(inname,viewer);
	osgDB::writeNodeFile(*md, outdir+"model.osg");
	viewer.setSceneData(md);

	if(!viewer.getCameraManipulator()) {
		osgGA::TrackballManipulator* manip = new osgGA::TrackballManipulator;
		manip->setAllowThrow(false);
		viewer.setCameraManipulator(manip);
	}

	using CaptureSceneHelper::CaptureCallBack;
	using CaptureSceneHelper::CaptureHandler;
	using SwitchCameraHelper::CameraUpdator;
	using SwitchCameraHelper::SwitchCameraHandler;

	osg::ref_ptr<CameraUpdator> updator = 
		new CameraUpdator(&viewer);
	osg::ref_ptr<SwitchCameraHandler> switcher = 
		new SwitchCameraHandler(updator);
	viewer.getCamera()->setUpdateCallback(updator);
	viewer.addEventHandler(switcher);

	osg::ref_ptr<CaptureCallBack> capture;
	osg::ref_ptr<CaptureHandler> handler;
	if(argc>2) { //do not capture without output dir!!!
		capture = new CaptureCallBack(30);
		handler = new CaptureHandler(capture);
		viewer.getCamera()->setPostDrawCallback(capture);
		viewer.addEventHandler(handler);
	}

	viewer.addEventHandler( new osgGA::StateSetManipulator(
		viewer.getCamera()->getOrCreateStateSet()) );
	viewer.addEventHandler(new osgViewer::StatsHandler);

	viewer.setUpViewInWindow(50,50,640,480);
	viewer.getCamera()->setClearColor(osg::Vec4(0,0.1,0.3,1));

	viewer.run();

	if(capture.valid()) capture->saveKeyFrames(outdir);
	return 0;
}
