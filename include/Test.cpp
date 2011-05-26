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

LKTracker tracker("/home/simbaforrest/project/Datasets/lena.jpg");

int BkgModifyCnt=0; //global signal for update osg
bool OpenCVneedQuit=false;
bool needToInit = false;

double camR[3][3]={1,0,0,0,1,0,0,0,1},camC[3]={0,-10,0};

struct TrackThread : public OpenThreads::Thread {
	OpenThreads::Mutex _mutex;

	virtual void run() {
		for(; videoFromWebcam || 
				cap.get(CV_CAP_PROP_POS_AVI_RATIO)<=1;) {

			_mutex.lock();

			cap >> frame;
			cvtColor(frame, gray, CV_BGR2GRAY);
			if( needToInit ) {
				cout<<"initing..."<<endl;
				needToInit=!tracker.init(gray);
				cout<<"...inited"<<endl;
			} else if( !tracker.opts.empty() ) {
				cout<<"tracking..."<<endl;
				if(prevGray.empty()) {
					gray.copyTo(prevGray);
				}
				needToInit=!tracker(prevGray, gray, frame);
				cout<<(needToInit?"...track lost!":"...tracked")<<endl;
				tracker.GetCameraPose(camR,camC);
			}
			++BkgModifyCnt;

			if(OpenCVneedQuit) {
				break;
			}
			_mutex.unlock();

			swap(prevGray, gray);
		}

		OpenThreads::Thread::YieldCurrentThread();
		cout<<"[TrackThread] OpenCV quited..."<<endl;
	}
};

struct QuitHandler : public osgGA::GUIEventHandler {
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
			}
		}
		return false;
	}
};

//update captured image as well as transforms for each frame
struct ARVideoUpdateCallback : public osg::NodeCallback {
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) {
		if(BkgModifyCnt>0) {
			arvideo->frame->dirty();
			CV2CG::cv2cg(camC,camR,*arscene);
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
	arscene->addChild(cow);

	osg::ref_ptr<osg::Image> backgroundImage = new osg::Image;
	helper::cvmat2osgimage(frame,backgroundImage);
	arvideo = new helper::ARVideoBackground(backgroundImage);
	arvideo->setUpdateCallback(new ARVideoUpdateCallback);

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

	cout<<"[main] press any key to quit..."<<endl;
	return getchar();
}
