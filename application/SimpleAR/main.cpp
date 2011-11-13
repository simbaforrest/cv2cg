/************************************************************************\

  Copyright 2011 The University of Michigan.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software
  and its documentation for educational, research and non-profit
  purposes, without fee, and without a written agreement is
  hereby granted, provided that the above copyright notice and
  the following paragraph appear in all copies.

  THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY OF MICHIGAN "AS IS" AND 
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF MICHIGAN
  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
  Authors:

			Chen Feng
            Laboratory for Interactive Visualization in Engineering (LIVE)
			Department of Civil and Environmental Engineering
            2350 Hayward Street, Suite 2340 GG Brown Building
            University of Michigan
            Ann Arbor, MI 48109-2125
			Phone:    (734)764-8495
			EMail:    simbaforrest@gmail.com
			WWW site: http://www.umich.edu/~cforrest
            
			Vineet R. Kamat
            Laboratory for Interactive Visualization in Engineering (LIVE)
			Department of Civil and Environmental Engineering
            2350 Hayward Street, Suite 2340 GG Brown Building
            University of Michigan
            Ann Arbor, MI 48109-2125
            Phone:    (734)764-4325
			EMail:    vkamat@umich.edu
			WWW site: http://pathfinder.engin.umich.edu

\************************************************************************/

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "Log.h"
#include "OpenCVHelper.h"
#include "OSGHelper.h"
#include "OpenCV2OSG.h"
#include "CV2CG.h"

#include "KEGTracker.hpp"

Log::Level Log::level = Log::LOG_INFO;

using namespace cv;
using namespace std;

bool videoFromWebcam;
VideoCapture cap;
Mat frame,gray,prevGray;
int imgW, imgH;

osgViewer::Viewer viewer;
osg::ref_ptr<helper::ARVideoBackground> arvideo;
osg::ref_ptr<helper::ARSceneRoot> arscene;
osg::ref_ptr<osg::MatrixTransform> manipMat;

KEGTracker tracker;

int BkgModifyCnt=0; //global signal for update osg
bool OpenCVneedQuit=false;
bool needToInit = false;
bool needToCapframe = false;
double videoFrameCnt = -1;
double threshKeydistance = 200;

double camR[3][3]={{1,0,0},{0,1,0},{0,0,1}},camT[3]={0,0,0},lastT[3]={0};

int framecnt = 0;

struct TrackThread : public OpenThreads::Thread {
	OpenThreads::Mutex _mutex;

	TrackThread() {}

	virtual void run() {
		helper::PerformanceMeasurer PM;

		PM.tic();
		for(; videoFromWebcam || framecnt<videoFrameCnt; ++framecnt) {

			_mutex.lock();

			if(!videoFromWebcam) {
				loglni("[TrackThread] framecnt="<<framecnt);
			}

			cap >> frame;
			if(frame.empty()) {
				loglni("[TrackThread] no valid frame, exit!!!");
				OpenCVneedQuit = true;
				_mutex.unlock();
				break;
			}

			cvtColor(frame, gray, CV_BGR2GRAY);
			double rms, ncc;
			if( needToInit ) {
				loglni("[TrackThread] INITing...");
				needToInit=!tracker.init(gray,rms,ncc);
				loglni("[TrackThread] ...INITed");
			} else if( !tracker.opts.empty() ) {
				if(prevGray.empty()) {
					gray.copyTo(prevGray);
				}
				needToInit=!tracker(prevGray, gray, frame, rms, ncc);
				tracker.GetCameraPose(camR,camT);
				double diff[3]={camT[0]-lastT[0],camT[1]-lastT[1],camT[2]-lastT[2]};
				double dist = helper::normL2(3,1,diff);
				if(!needToInit && (!videoFromWebcam || //if from file, then save each frame
					(dist>=threshKeydistance && needToCapframe)) ) {
					//needToCapframe = false;
					std::copy(camT,camT+3,lastT);
					tracker.CapKeyFrame(framecnt, frame, camR, camT, rms, ncc);
				}
			}
			++BkgModifyCnt;
			loglni("[TrackThread] fps="<<1.0/PM.toctic());
			if(tracker.debug)
				loglni("[TrackThread] frame end -------------------------");

			_mutex.unlock();
			if(OpenCVneedQuit) {
				break;
			}

			swap(prevGray, gray);
		}

		OpenThreads::Thread::YieldCurrentThread();
		loglni("[TrackThread] OpenCV quited...");
		if(!videoFromWebcam) {
			loglni("[TrackThread] OpenCV notify OSG to quit...");
			viewer.setDone(true);
		}
//		cerr<<"[TrackThread] testCancel="<<testCancel()<<endl;
	}
};

struct QuitHandler : public osgGA::GUIEventHandler {
	double sx,sy,sz;
	double mx,my,mz;
	QuitHandler() {sx=sy=sz=1;mx=my=mz=0;}

	inline void switchARVideo() {
		static bool showvideo=false;
		showvideo=!showvideo;
		arvideo->setNodeMask(showvideo);
	}

	inline void switchARScene() {
		static bool showscene=true;
		showscene=!showscene;
		arscene->setNodeMask(showscene);
	}

	virtual bool handle(const osgGA::GUIEventAdapter &ea, 
		osgGA::GUIActionAdapter &aa) {
		if(ea.getEventType()==osgGA::GUIEventAdapter::KEYDOWN) {
			if(ea.getKey()==osgGA::GUIEventAdapter::KEY_Escape) {
				OpenCVneedQuit=true;
				loglni("[QuitHandler] OSG notify OpenCV to quit...");
			} else if(ea.getKey()==' ') {
				needToInit=true;
			} else if(ea.getKey()=='d') {
				tracker.debug=!tracker.debug;
				if(tracker.debug) loglni("[Debug Mode] ON.");
				else loglni("[Debug Mode] OFF.");
			} else if(ea.getKey()=='.') { //>
				sx+=0.4;sy+=0.4;sz+=0.4;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
			} else if(ea.getKey()==',') { //<
				sx-=0.2;sy-=0.2;sz-=0.2;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
			} else if(ea.getKey()=='c') {
				needToCapframe = !needToCapframe;
				if(needToCapframe) loglni("[Capture Frame] Begin.");
				else loglni("[Capture Frame] End.");
			} else if(ea.getKey()=='h') {
				QuitHandler::usage();
			} else if(ea.getKey()=='1') {
				switchARVideo();
			} else if(ea.getKey()=='2') {
				switchARScene();
			} else if(ea.getKey()==osgGA::GUIEventAdapter::KEY_KP_Up) {
				my+=10;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
			} else if(ea.getKey()==osgGA::GUIEventAdapter::KEY_KP_Down) {
				my-=5;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
			} else if(ea.getKey()==osgGA::GUIEventAdapter::KEY_KP_Left) {
				mx-=5;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
			} else if(ea.getKey()==osgGA::GUIEventAdapter::KEY_KP_Right) {
				mx+=10;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
			} else if(ea.getKey()==osgGA::GUIEventAdapter::KEY_Page_Up) {
				mz+=10;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
			} else if(ea.getKey()==osgGA::GUIEventAdapter::KEY_Page_Down) {
				mz-=5;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
			}
		}
		return false;
	}

	static void usage() {
		cout<<
		"Handler Usage\n"
		"  \'1\': show background video ON/OFF\n"
		"  \'2\': show scene object ON/OFF\n"
		"  \' \': reset/redetect template\n"
		"  \'d\': switch debug mode ON/OFF\n"
		"  \'.\': increase scene scale\n"
		"  \',\': decreate scene scale\n"
		"  \'c\': switch scene capture ON/OFF\n"
		"  \'h\': print HELP information"
		<<endl;
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
			loglni("[InitVideoCapture] open from device "<<idx);
			cap.open(idx);
			videoFromWebcam = true;
		} else {
			loglni("[InitVideoCapture] open from file "<<argv[1]);
			cap.open(argv[1]);
			videoFromWebcam = false;

			videoFrameCnt=cap.get(CV_CAP_PROP_FRAME_COUNT)-2;
			loglni("[InitVideoCapture] number of frames: "<<videoFrameCnt);

			needToInit=true;
		}
	}
	return cap.isOpened();
}

int main( int argc, char **argv )
{
	if(argc<4) {
		cout<< "[usage] " <<argv[0]<<" <device number|video file>"
			" <K matrix file> <template file>"
			" [osg scene file] [keyframe saving path]" <<endl;
		QuitHandler::usage();
		return 1;
	}
	if( !InitVideoCapture(argc,argv) ) {
		loglne("[main] Could not initialize capturing...");
		return 1;
	}
	loglni("[main] Video Captured.");
	QuitHandler::usage();

	if(cap.set(CV_CAP_PROP_FRAME_WIDTH, 640))
		loglni("[main] video width=640");
	if(cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480))
		loglni("[main] video height=480");

	cap >> frame;
	if( frame.empty() ) {
		loglni("[main] No valid video!");
		return 1;
	}
	imgW = frame.cols;
	imgH = frame.rows;

	loglni("[main] loading K matrix from: "<<argv[2]);
	double K[9];
	std::ifstream kfile(argv[2]);
	for(int i=0; i<9; ++i) kfile >> K[i];
	tracker.loadK(K);
	loglni("[main] K matrix loaded:");
	loglni(helper::PrintMat<>(3,3,K));

	loglni("[main] load template image from: "<<argv[3]);
	tracker.loadTemplate(argv[3]);

	osg::ref_ptr<osg::Group> root = new osg::Group;

	string scenefilename = (argc>4?argv[4]:("cow.osg"));
	osg::ref_ptr<osg::Node> cow = osgDB::readNodeFile(scenefilename);
	arscene = new helper::ARSceneRoot;
	helper::FixMat<3,double>::Type matK = helper::FixMat<3,double>::ConvertType(K);
	CV2CG::cv2cg(matK,0.01,500,imgW,imgH,*arscene);
	manipMat = new osg::MatrixTransform(osg::Matrix::identity());
	manipMat->addChild(cow);
	manipMat->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
	arscene->addChild(manipMat);

	osg::ref_ptr<osg::Image> backgroundImage = new osg::Image;
	helper::cvmat2osgimage(frame,backgroundImage);
	arvideo = new helper::ARVideoBackground(backgroundImage);
	root->setUpdateCallback(new ARUpdateCallback);

	root->addChild(arvideo);
	root->addChild(arscene);

	viewer.setSceneData(root);
	viewer.addEventHandler(new osgViewer::StatsHandler);
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
	viewer.addEventHandler(new QuitHandler);

	loglni("[main] press any key to start...");
	getchar();

	//start tracking thread
	OpenThreads::Thread::Init();
	TrackThread* thr = new TrackThread;
	thr->start();

	viewer.run();

	if(argc>5) {
		loglni("[main] saving keyframes to: "<<argv[5]);
		tracker.SaveKeyFrames(argv[5]);
	}

	delete thr;
	loglni("[main] DONE...exit!");
	return 0;
}
