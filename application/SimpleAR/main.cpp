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
#include <fstream>
#include <sstream>

#include "Log.hxx"
#include "Log.h"
#include "OpenCVHelper.h"
#include "OSGHelper.h"
#include "OpenCV2OSG.h"
#include "CV2CG.h"

#include "Tracker.hpp"

using namespace cv;
using namespace std;

bool videoFromWebcam;
VideoCapture cap;
Mat frame,gray,prevGray;
int imgW, imgH;

osg::ref_ptr<helper::ARVideoBackground> arvideo;
osg::ref_ptr<helper::ARSceneRoot> arscene;
osg::ref_ptr<osg::MatrixTransform> manipMat;

LKTracker tracker("/home/simbaforrest/project/Datasets/lena.jpg");

int BkgModifyCnt=0; //global signal for update osg
bool OpenCVneedQuit=false;
bool needToInit = false;
bool needToCapframe = false;

double camR[3][3]={{1,0,0},{0,1,0},{0,0,1}},camT[3]={0,0,0},lastT[3]={0};

int framecnt = 0;

struct TrackThread : public OpenThreads::Thread {
	OpenThreads::Mutex _mutex;
	bool running;

	TrackThread() {running=false;}

	virtual void run() {
		running=true;
		for(; videoFromWebcam || 
				cap.get(CV_CAP_PROP_POS_AVI_RATIO)<=1;++framecnt) {

			_mutex.lock();
			helper::fps(true);
			cap >> frame;
			cvtColor(frame, gray, CV_BGR2GRAY);
			if( needToInit ) {
				cout<<"[TrackThread] initing..."<<endl;
				needToInit=!tracker.init(gray);
				cout<<"[TrackThread] ...inited"<<endl;
			} else if( !tracker.opts.empty() ) {
				if(prevGray.empty()) {
					gray.copyTo(prevGray);
				}
				needToInit=!tracker(prevGray, gray, frame);
				tracker.GetCameraPose(camR,camT);
				double diff[3]={camT[0]-lastT[0],camT[1]-lastT[1],camT[2]-lastT[2]};
				double dist = helper::normL2(3,1,diff);
				if(dist>200 && needToCapframe) {
					//needToCapframe = false;
					std::copy(camT,camT+3,lastT);
					tracker.CapKeyFrame(frame, camR, camT);
				}
			}
			++BkgModifyCnt;
			helper::fps(false);

			_mutex.unlock();
			if(OpenCVneedQuit) {
				break;
			}

			swap(prevGray, gray);
		}

		OpenThreads::Thread::YieldCurrentThread();
		cout<<"[TrackThread] OpenCV quited..."<<endl;
		running=false;
	}
};

struct QuitHandler : public osgGA::GUIEventHandler {
	double sx,sy,sz;
	QuitHandler() {sx=sy=sz=1;}
	virtual bool handle(const osgGA::GUIEventAdapter &ea, 
		osgGA::GUIActionAdapter &aa) {
		if(ea.getEventType()==osgGA::GUIEventAdapter::KEYDOWN) {
			if(ea.getKey()==osgGA::GUIEventAdapter::KEY_Escape) {
				OpenCVneedQuit=true;
				cout<<"[QuitHandler] OSG notify OpenCV to quit..."<<endl;
			} else if(ea.getKey()==' ') {
				needToInit=true;
			} else if(ea.getKey()=='d') {
				tracker.debug=!tracker.debug;
			} else if(ea.getKey()=='.') { //>
				sx+=0.4;sy+=0.4;sz+=0.4;
				manipMat->setMatrix(osg::Matrix::scale(sx,sy,sz));
			} else if(ea.getKey()==',') { //<
				sx-=0.2;sy-=0.2;sz-=0.2;
				manipMat->setMatrix(osg::Matrix::scale(sx,sy,sz));
			} else if(ea.getKey()=='c') {
				needToCapframe = !needToCapframe;
			}
		}
		return false;
	}
};

//update captured image as well as transforms for each frame
struct ARUpdateCallback : public osg::NodeCallback {
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) {
		if(BkgModifyCnt>0) {
			arvideo->frame->dirty();
			CV2CG::cv2cg(camT,camR,*arscene);
			--BkgModifyCnt;
		}
	}
};

bool InitVideoCapture(int argc, char ** argv)
{
	if(argc==1) {//default
		cap.open(0);
		videoFromWebcam=true;
	} else {
		int len = strlen(argv[1]);
		bool fromDevice=true;
		for(int i=0; i<len; ++i) {
			if( !isdigit(argv[1][i]) ) {
				fromDevice = false;
				break;
			}
		}
		if(fromDevice) {
			int idx = atoi(argv[1]);
			cout<<"[InitVideoCapture] open from device "<<idx<<endl;
			cap.open(idx);
			videoFromWebcam = true;
		} else {
			cout<<"[InitVideoCapture] open from file "<<argv[1]<<endl;
			cap.open(argv[1]);
			videoFromWebcam = false;
		}
	}
	return cap.isOpened();
}

int main( int argc, char **argv )
{
	if(argc<2) {
		cout<< "[usage] " <<argv[0]<<" <device number|video file>"
			" [osg scene file]" <<endl;
		return 1;
	}
	if( !InitVideoCapture(argc,argv) ) {
		cout << "[main] Could not initialize capturing...\n";
		return 1;
	}

	if(cap.set(CV_CAP_PROP_FRAME_WIDTH, 640))
		cout<<"[main] video width=640"<<endl;
	if(cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480))
		cout<<"[main] video height=480"<<endl;

	cap >> frame;
	if( frame.empty() ) {
		cout<<"[main] No valid video!"<<endl;
		return 1;
	}
	imgW = frame.cols;
	imgH = frame.rows;

	osgViewer::Viewer viewer;
	osg::ref_ptr<osg::Group> root = new osg::Group;

	string scenefilename = (argc>2?argv[2]:("cow.osg"));
	osg::ref_ptr<osg::Node> cow = osgDB::readNodeFile(scenefilename);
	arscene = new helper::ARSceneRoot;
	helper::FixMat<3,double>::Type matK = helper::FixMat<3,double>::ConvertType(K);
	CV2CG::cv2cg(matK,0.01,500,imgW,imgH,*arscene);
	manipMat = new osg::MatrixTransform(osg::Matrix::identity());
	manipMat->addChild(cow);
	arscene->addChild(manipMat);

	osg::ref_ptr<osg::Image> backgroundImage = new osg::Image;
	helper::cvmat2osgimage(frame,backgroundImage);
	arvideo = new helper::ARVideoBackground(backgroundImage);
	arvideo->setUpdateCallback(new ARUpdateCallback);

	root->addChild(arvideo);
	root->addChild(arscene);

	viewer.setSceneData(root);
	viewer.addEventHandler(new osgViewer::StatsHandler);
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
	viewer.addEventHandler(new QuitHandler);

	//start tracking thread
	OpenThreads::Thread::Init();
	TrackThread thr;
	thr.start();

	viewer.run();

	tracker.SaveKeyFrames("/home/simbaforrest/project/cv2cg/data/");

	cout<<"[main] press any key to quit..."<<endl;
	return getchar();
}
