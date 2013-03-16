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

int GUIMode(osgViewer::Viewer& viewer, std::string outdir, bool doCapture) {
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
	if(doCapture) {
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

struct Pose {
	double R[3][3], T[3];
};

int AutoMode(osgViewer::Viewer& viewer, std::string posefilename, std::string outdir) {
	using CaptureSceneHelper::CaptureCallBack;
	using CaptureSceneHelper::CaptureHandler;

	osg::ref_ptr<CaptureCallBack> capture;
	capture = new CaptureCallBack(-1);
	viewer.getCamera()->setPostDrawCallback(capture);

	viewer.getCamera()->setClearColor(osg::Vec4(1,1,1,1));
	//viewer.getCamera()->setClearColor(osg::Vec4(0,0.1,0.3,1));

	std::ifstream in(posefilename.c_str());
	if(!in) return -1;

	int nPoses=0;
	in>>nPoses;
	if(nPoses<=0) return 1;

	int imgW=0, imgH=0;
	in>> imgW >> imgH;
	if(imgW<=0 || imgH<=0) return 1;

	double K[3][3];
	for(int i=0; i<3; ++i)
		for(int j=0; j<3; ++j)
			in>>K[i][j];

	std::vector<Pose> pose(nPoses);
	for(int k=0; k<nPoses; ++k) {
		Pose& p = pose[k];
		for(int i=0; i<3; ++i)
			for(int j=0; j<3; ++j)
				in>>p.R[i][j];
		for(int i=0; i<3; ++i) in>>p.T[i];
	}

	viewer.setUpViewInWindow(50,50,imgW,imgH);
	CV2CG::cv2cg(K, 1, 10000, imgW, imgH, *viewer.getCamera());
	int cnt=0;
	//make sure even only 1 frame can be captured
	viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
	viewer.realize();
	while(!viewer.done() && cnt<nPoses) {
		capture->start(true);
		const Pose& p = pose[cnt];
		CV2CG::cv2cg(p.T,p.R,*viewer.getCamera());
		viewer.frame();
		++cnt;
	}
	capture->saveKeyFrames(outdir);
	return 0;
}

int main( int argc, char **argv )
{
	LogHelper::GLogControl::Instance().level = LogHelper::LOG_INFO;
	if(argc<=1) {
		printf("Usage: \n\tCameraSimulator <Drawing Script File Name>"
			" [OutPut Dir]\n");
		printf("Usage: \n\tCameraSimulator <Drawing Script File Name>"
			" <Camera Parameter File Name> <OutPut Dir>\n");
		printf("Example: \n\tCameraSimulator draw.txt D:/Out/\n");
		printf("Example: \n\tCameraSimulator draw.txt\n");
		printf("Example: \n\tCameraSimulator draw.txt pose.txt D:/out/\n");
		return -1;
	}

	std::string inname(argv[1]);
#ifdef WIN32
	std::string outdir(".\\");
#else
	std::string outdir("./");
#endif

	if(argc==3) outdir = std::string(argv[2]);
	if(argc==4) outdir = std::string(argv[3]);
	helper::legalDir(outdir);

	osgViewer::Viewer viewer;
	osg::ref_ptr<osg::Node> md = CreateSceneHelper::create_SCENE(inname,viewer);
	osgDB::writeNodeFile(*md, outdir+"model.osg");
	viewer.setSceneData(md);

	if(argc!=4) return GUIMode(viewer, outdir, argc>2);//do not capture without output dir!!!
	return AutoMode(viewer, std::string(argv[2]), outdir);
}
