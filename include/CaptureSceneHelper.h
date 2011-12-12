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

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osg/Image>
#include <osg/Camera>
#include <osg/RenderInfo>
#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>

#include "Log.h"
#include "OpenCVHelper.h"

namespace CaptureSceneHelper {

struct KeyFrame {
	osg::ref_ptr<osg::Camera> cam; //camera paramters
	osg::ref_ptr<osg::Image> img; //screenshot
};

using helper::PerformanceMeasurer;

struct CaptureCallBack : public osg::Camera::DrawCallback
{
	CaptureCallBack(int idealfps=25) {
		PM.scale = 1000.0;
		fps = idealfps;
		deltams = 1000.0/fps;
		isManual = true;
		doCapture = false;
	}

	mutable bool                        doCapture;
	mutable std::vector<KeyFrame>       keyframes;
	mutable PerformanceMeasurer         PM;
	double deltams;
	int fps;
	bool isManual;

	inline void operator () (osg::RenderInfo& renderInfo) const
	{
		if (!doCapture) return;
		double delta = PM.toc();
		if(delta<deltams) return;
		PM.tic();

		osg::Camera* camera = renderInfo.getCurrentCamera();
		osg::Viewport* viewport = camera ? camera->getViewport() : 0;
		if(!viewport) return;

		KeyFrame kf;
		kf.img = new osg::Image;
		//fill image
		kf.img->readPixels(int(viewport->x()),int(viewport->y()),
			int(viewport->width()),int(viewport->height()),
			GL_RGBA,
			GL_UNSIGNED_BYTE);
		kf.cam = new osg::Camera(*camera);

		keyframes.push_back(kf);

		loglni("[CaptureCallBack] Screenshot #"<<(int)keyframes.size());
		if(isManual) doCapture = false;
	}

	inline void start(bool manual=true) {
		doCapture = true;
		isManual = manual;
		PM.tic();
	}

	inline void end() {
		doCapture = false;
		isManual = true;
	}

	inline bool saveKeyFrames(std::string dir, std::string header="frame") {
		if(keyframes.size()<=0) return false;
		helper::legalDir(dir);
		std::ofstream mainout((dir+header+".main").c_str());
		mainout << "CAMERAFRUSTUM " << keyframes.size() << " 1"<< std::endl;
		mainout << dir << std::endl;
		loglni("[CaptureCallBack] saving keyframes...");
		for(int i=0; i<(int) keyframes.size(); ++i) {
			KeyFrame& kf = keyframes[i];
			std::string strid = helper::num2str(i);
			std::string imgname = header + strid + std::string(".png");
			std::string parname = header + strid + std::string(".par");
			osgDB::writeImageFile(*(kf.img), dir+imgname);
			std::ofstream out((dir+parname).c_str());
			CV2CG::CV_CG_Report(*(kf.cam), out);
			out.close();
			mainout << imgname << std::endl;
			mainout << parname << std::endl;
			loglni(">>> "<<imgname);
			loglni(">>> "<<parname);
		}
		loglni("[CaptureCallBack] all keyframes saved.");
		mainout.close();
		return true;
	}
};

struct CaptureHandler : public osgGA::GUIEventHandler
{
	CaptureHandler(CaptureCallBack* cap, int mkey='p', int akey='v') {
		manCapKey = mkey;
		autoCapKey = akey;
		capture = cap;
	}

	int manCapKey; //manual capture
	int autoCapKey; //auto capture
	osg::ref_ptr<CaptureCallBack> capture;

	inline bool handle(const osgGA::GUIEventAdapter& ea,
		osgGA::GUIActionAdapter& aa,
		osg::Object*, osg::NodeVisitor*)
	{
		if(!capture.valid()) return false;
		if (ea.getHandled()) return false;
		osgViewer::View* viewer = dynamic_cast<osgViewer::View*>(&aa);
		if(!viewer) return false;

		if(ea.getEventType()==osgGA::GUIEventAdapter::KEYUP) {
			if(ea.getKey()==manCapKey) { //manual capture key
				capture->start(true); return true;
			}
			if(ea.getKey()==autoCapKey) { //auto capture key
				if(!capture->doCapture) {
					capture->start(false);
				} else {
					capture->end();
				}
				return true;
			}
			if(ea.getKey()=='h'||ea.getKey()=='H') {
				printHelp(); return false;
			}
		}

		return false;
	}

	inline void printHelp() {
		loglni("\nUsage(CaptureHandler):\n"
			"\tPress "<<(char)manCapKey<<" : take pictures manually\n"
			"\tPress "<<(char)autoCapKey<<" : take pictures automatically\n"
			"\tPress h : print help information to console");
	}
};

}//end of namespace CaptureSceneHelper
