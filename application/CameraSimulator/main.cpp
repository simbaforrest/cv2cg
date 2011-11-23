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
			" [<OutPutFileName>(Without extention!)]\n");
		LogI("Example: \n\tCameraSimulator draw.txt D:/Out\n");
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
	osg::ref_ptr<osg::Node> md = CreateSceneHelper::create_SCENE(inname);
	osgDB::writeNodeFile(*md, outdir+"_model.osg");
	osgViewer::Viewer viewer;
	viewer.setSceneData(md);

	using CaptureSceneHelper::CaptureCallBack;
	using CaptureSceneHelper::CaptureHandler;
	using SwitchCameraHelper::CameraUpdator;
	using SwitchCameraHelper::SwitchCameraHandler;

	osg::ref_ptr<CaptureCallBack> capture = new CaptureCallBack(30);
	osg::ref_ptr<CaptureHandler> handler = new CaptureHandler(capture);
	osg::ref_ptr<CameraUpdator> updator = new CameraUpdator(&viewer);
	osg::ref_ptr<SwitchCameraHandler> switcher = new SwitchCameraHandler(updator);
	viewer.getCamera()->setPostDrawCallback(capture);
	viewer.getCamera()->setUpdateCallback(updator);
	viewer.addEventHandler(handler);
	viewer.addEventHandler(switcher);

	viewer.addEventHandler( new osgGA::StateSetManipulator(
		viewer.getCamera()->getOrCreateStateSet()) );
	viewer.addEventHandler(new osgViewer::StatsHandler);

	viewer.setUpViewInWindow(50,50,640,480);
	viewer.getCamera()->setClearColor(osg::Vec4(0,0.1,0.3,1));

	osgGA::TrackballManipulator* manip = new osgGA::TrackballManipulator;
	manip->setAllowThrow(false);
	viewer.setCameraManipulator(manip);

	viewer.run();

	if(argc>2) capture->saveKeyFrames(outdir);
	return 0;
}
