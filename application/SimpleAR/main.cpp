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

#define ESM_ORIGINAL //define this if you have original implementation of ESM
#define ESM_DEBUG 0
#define KEG_DEBUG 0
#include "keg/KEGTracker.hpp"

#include "apriltag/apriltag.hpp"
#include "apriltag/TagFamilyFactory.hpp"

Log::Level Log::level = Log::LOG_INFO;

using namespace cv;
using namespace std;

//////////////apriltag//////////////////
using april::tag::INT64;
using april::tag::TagFamily;
using april::tag::TagFamilyFactory;
using april::tag::TagDetector;
using april::tag::TagDetection;
using helper::ImageSource;

cv::Ptr<TagFamily> tagFamily;
cv::Ptr<TagDetector> detector;
cv::Mat HI;
cv::Mat iHI; //inverse of init Homography, X_tag = iHI * X_keg

//////////////video/////////////////////
cv::Ptr<ImageSource> is;
bool videoFromWebcam;
Mat frame,gray;
int imgW, imgH;

//////////////osg///////////////////////
osgViewer::Viewer viewer;
osg::ref_ptr<helper::ARVideoBackground> arvideo;
osg::ref_ptr<helper::ARSceneRoot> arscene;
osg::ref_ptr<osg::MatrixTransform> manipMat;

//////////////KEGTracker////////////////
KEGTracker tracker;

//////////////MISC//////////////////////
int BkgModifyCnt=0; //global signal for update osg
bool OpenCVneedQuit=false;
bool needToInit = false;
bool needToCapframe = false;
bool opencvDraw = true;
double videoFrameCnt = -1;
double threshKeydistance = 200;

double camR[3][3]={{1,0,0},{0,1,0},{0,0,1}},camT[3]={0,0,0},lastT[3]={0};

int framecnt = 0;

////////////////////////////////////////////////////
bool findAprilTag(Mat &image, int targetID, Mat &ret, bool draw=false, int errorThresh=0) {
	vector<TagDetection> detections;
	double opticalCenter[2] = { image.cols/2.0, image.rows/2.0 };
	detector->process(image, opticalCenter, detections);

	for(int id=0; id<(int)detections.size(); ++id) {
		TagDetection &dd = detections[id];
		if(dd.id!=targetID) continue;
		if(dd.hammingDistance>errorThresh) continue; //very strict!

		if(draw) {
			for(int i=0; i<4; ++i) {
				int j = (i+1)%4;
				Point r1(dd.p[i][0],dd.p[i][1]);
				Point r2(dd.p[j][0],dd.p[j][1]);
				line( image, r1, r2, helper::CV_RED, 2 );
			}
			cv::putText( image, helper::num2str(dd.id), cv::Point(dd.cxy[0],dd.cxy[1]), CV_FONT_NORMAL, 1, helper::CV_BLUE, 2 );
		}

		cv::Mat tmp(3,3,CV_64FC1, (double*)dd.homography[0]);
		double vm[] = {1,0,dd.hxy[0],0,1,dd.hxy[1],0,0,1};
		ret = cv::Mat(3,3,CV_64FC1,vm) * tmp;
		return true;
	}
	return false;
}

////////////////////////////////////////////////////
struct TrackThread : public OpenThreads::Thread {
	OpenThreads::Mutex _mutex;

	TrackThread() {}

	virtual void run() {
		helper::PerformanceMeasurer PM;

		PM.tic();
		while(!is->done()) {
			_mutex.lock();

			is->get(frame);
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
				Mat tmpH;
				if( findAprilTag(frame, 0, tmpH, true) ) {
#if !USE_INTERNAL_DETECTOR
					Mat initH = tmpH * iHI;
					needToInit = !tracker.init(gray, initH, rms, ncc);
#else
					needToInit=!tracker.init(gray,rms,ncc);
#endif
					if(!needToInit) loglni("[TrackThread] ...INITed");
					if(!needToInit) tracker.draw3D(frame);
				}
			} else {
				needToInit=!tracker(gray, rms, ncc, opencvDraw?&frame:0);
				if(!needToInit) tracker.GetCameraPose(camR,camT,false,true);
			}
			++BkgModifyCnt;
			loglni("[TrackThread] fps="<<1.0/PM.toctic());

			_mutex.unlock();
			if(OpenCVneedQuit) {
				break;
			}

			++framecnt;
		}

		OpenThreads::Thread::YieldCurrentThread();
		loglni("[TrackThread] OpenCV quited...");
		if(!videoFromWebcam) {
			loglni("[TrackThread] OpenCV notify OSG to quit...");
			viewer.setDone(true);
		}
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
			switch(ea.getKey()) {
			case osgGA::GUIEventAdapter::KEY_Escape:
				OpenCVneedQuit=true;
				loglni("[QuitHandler] OSG notify OpenCV to quit...");
				break;
			case ' ':
				needToInit=true;
				break;
			case 'd':
				opencvDraw = !opencvDraw;
				break;
			case '.': //>
				sx+=0.4;sy+=0.4;sz+=0.4;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
				break;
			case ',': //<
				sx-=0.2;sy-=0.2;sz-=0.2;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
				break;
			case 'c':
				needToCapframe = !needToCapframe;
				if(needToCapframe) loglni("[Capture Frame] Begin.");
				else loglni("[Capture Frame] End.");
				break;
			case 'h':
				QuitHandler::usage(); break;
			case '1':
				switchARVideo(); break;
			case '2':
				switchARScene(); break;
			case osgGA::GUIEventAdapter::KEY_KP_Up:
				my+=10;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
				break;
			case osgGA::GUIEventAdapter::KEY_KP_Down:
				my-=5;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
				break;
			case osgGA::GUIEventAdapter::KEY_KP_Left:
				mx-=5;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
				break;
			case osgGA::GUIEventAdapter::KEY_KP_Right:
				mx+=10;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
				break;
			case osgGA::GUIEventAdapter::KEY_Page_Up:
				mz+=10;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
				break;
			case osgGA::GUIEventAdapter::KEY_Page_Down:
				mz-=5;
				manipMat->setMatrix(osg::Matrix::translate(mx,my,mz)*osg::Matrix::scale(sx,sy,sz));
				break;
			case 'p': {
				static bool pa=false;
				is->pause(pa); pa = !pa; break; }
			case 'l': {
				static bool lo=false;
				is->loop(lo); lo = !lo; break; }
			}//end of switch
		}
		return true;
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

void usage( int argc, char **argv ) {
	cout<< "[usage] " <<argv[0]<<" <url>"
		" <K matrix file> <template file>"
		" [osg scene file] [AprilTag Family ID]" <<endl;
	cout<< "Supported TagFamily ID List:\n";
	for(int i=0; i<(int)TagFamilyFactory::TAGTOTAL; ++i) {
		cout<<"\t"<<TagFamilyFactory::SUPPORT_NAME[i]<<" id="<<i<<endl;
	}
	cout<<"default ID: 0"<<endl;
	cout<<"Example ImageSource url:\n";
	cout<<"photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/*\n";
	cout<<"camera://0\n";
	cout<<"video:///home/simbaforrest/Videos/Webcam/keg_april.ogv"<<endl;
	QuitHandler::usage();
}

int main( int argc, char **argv )
{
	if(argc<4) {
		usage(argc,argv);
		return 1;
	}
	is = helper::createImageSource(argv[1]);
	if(is.empty() || is->done()) {
		loglne("[main] createImageSource failed or no valid imagesource!");
		return -1;
	}
	is->pause(false);
	is->reportInfo();
	is->get(frame);
	imgW = frame.cols; imgH = frame.rows;
	videoFromWebcam = false;
	if( is->classname() == "ImageSource_Camera" ) {
		videoFromWebcam = true;
	}

	loglni("[main] loading K matrix from: "<<argv[2]);
	double K[9];
	std::ifstream kfile(argv[2]);
	for(int i=0; i<9; ++i) kfile >> K[i];
	tracker.loadK(K);
	loglni("[main] K matrix loaded:");
	loglni(helper::PrintMat<>(3,3,K));

	loglni("[main] load template image from: "<<argv[3]);
	tracker.loadTemplate(argv[3]);

	//////////////// TagDetector /////////////////////////////////////////
	int tagid = 0; //default tag16h5
	if(argc>5) tagid = atoi(argv[5]);
	tagFamily = TagFamilyFactory::create(tagid);
	if(tagFamily.empty()) {
		loglne("[main] create TagFamily fail!");
		return -1;
	}
	detector = new TagDetector(tagFamily);
	if(detector.empty()) {
		loglne("[main] create TagDetector fail!");
		return -1;
	}
	Mat temp = imread(argv[3]);
	if( findAprilTag(temp, 0, HI, true) ) {
		namedWindow("template");
		imshow("template", temp);
		iHI = HI.inv();
	} else {
		loglne("[main error] detector did not find any apriltag on template image!");
		return -1;
	}

	//////////////// OSG ////////////////////////////////////////////////
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

	//start tracking thread
	OpenThreads::Thread::Init();
	TrackThread* thr = new TrackThread;
	thr->start();

	viewer.run();

	delete thr;
	loglni("[main] DONE...exit!");
	return 0;
}
