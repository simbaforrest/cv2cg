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

#include "AllHelpers.h"
#include "apriltag/apriltag.hpp"
#include "apriltag/TagFamilyFactory.hpp"

using namespace std;
using namespace cv;
using april::tag::UINT64;
using april::tag::TagFamily;
using april::tag::TagFamilyFactory;
using april::tag::TagDetector;
using april::tag::TagDetection;
using helper::ImageSource;
using helper::GConfig;

std::vector< cv::Ptr<TagFamily> > gTagFamilies;
cv::Ptr<TagDetector> gDetector;

/**
Calibration rig
To define Calibration rig in config file, simply write in AprilCalib.cfg as follows:
CalibRig={
	mode=3d
#names of each tag
	markerNames=[]
#repeat follow lines N times with i=S...S+N-1, (Xi, Yi, Zi) is the world coordinate
#of the center of the tagi
	tagCenters=[]
}
*/
struct CalibRig {
	bool isTag3d;

	typedef std::map<std::string, cv::Point3d> DataMap;
	DataMap name2Xw;

	CalibRig() : isTag3d(true)
	{}

	operator std::string() const {
		std::stringstream ss;
		ss<<"CalibRig::mode="<<(isTag3d?"3d":"2d")<<std::endl;
		ss<<"CalibRig::name2Xw=["<<std::endl;
		DataMap::const_iterator itr=name2Xw.begin();
		for(; itr!=name2Xw.end(); ++itr) {
			ss<<"\t"<<itr->first<<" "<<itr->second<<std::endl;
		}
		ss<<"]"<<std::endl;
		return ss.str();
	}

	void loadFromConfig() {
		using namespace ConfigHelper;
		ConfigNode::Ptr cfg = GConfig::Instance().getRoot().getChild("CalibRig");
		if (cfg==0) {
			logle("[CalibRig error] no CalibRig in Config!");
			exit(-1);
		}
		ConfigNode& cfn = *cfg;
		
		//get rig mode and start_id
		std::string mode = cfn["mode"].str();
		this->isTag3d=(mode.compare("3d")==0);
		std::vector<std::string> markerNames;
		cfn["markerNames"] >> markerNames;

		const int nTags=(int)markerNames.size();
		if(nTags<8) {
			logle("[CalibRig error] a 3D calibration rig must have more than 8 tags!");
			exit(-1);
		}
		std::vector<double> buf;
		cfn["tagCenters"]>>buf;
		if( nTags*3 != (int)buf.size() ) {
			logle("[CalibRig error] CalibRig:tagCenters invalid in Config!");
			exit(-1);
		}//TODO: check if the points forms a 3d rig really by PCA

		for(int i=0; i<nTags; ++i) {
			const std::string& name=markerNames[i];
			name2Xw[name]=cv::Point3d(buf[i*3+0],buf[i*3+1],buf[i*3+2]);
		}
	}

	bool name2worldPt(const std::string& name, double worldPt[3]) const {
		DataMap::const_iterator itr=name2Xw.find(name);
		if(itr==name2Xw.end()) return false;
		const cv::Point3d& pt=itr->second;
		worldPt[0]=pt.x;
		worldPt[1]=pt.y;
		worldPt[2]=pt.z;
		return true;
	}
} gRig;

struct AprilCalibprocessor : public ImageHelper::ImageSource::Processor {
	double tagTextScale;
	int tagTextThickness;
	bool doLog;
	bool isPhoto;
	bool useEachValidPhoto;
	bool logVisFrame;
	std::string outputDir;

	int nDistCoeffs;
	std::vector<std::vector<cv::Point2f> > imagePtsArr;
	std::vector<std::vector<cv::Point3f> > worldPtsArr;
	cv::Mat K;
	double rmsThresh;

	virtual ~AprilCalibprocessor() {}
	AprilCalibprocessor() : doLog(false), isPhoto(false) {
		ConfigHelper::ConfigNode::Ptr cfg_ptr = GConfig::Instance()->getChild("AprilCalibprocessor");
		assert(cfg_ptr!=0);
		ConfigHelper::ConfigNode& cfg=*cfg_ptr;
		tagTextScale = cfg.get<double>("tagTextScale",1.0f);
		tagTextThickness = cfg.get<int>("tagTextThickness",1);
		useEachValidPhoto = cfg.get<bool>("useEachValidPhoto",false);
		rmsThresh = cfg.get<int>("rmsThresh",2);
		nDistCoeffs = cfg.get<int>("nDistCoeffs",0);
		logVisFrame = cfg.get<bool>("logVisFrame",false);
		if(nDistCoeffs>5) {
			nDistCoeffs=5;
			logli("[AprilCalibprocessor warn] AprilCalib::nDistCoeffs>5, set back to 5!");
		}
		gDetector->segDecimate = cfg.get<bool>("segDecimate",false);
	}
/////// Override
	void operator()(cv::Mat& frame) {
		static helper::PerformanceMeasurer PM;
		vector<TagDetection> detections;
		cv::Mat orgFrame;
		frame.copyTo(orgFrame);
		PM.tic();
		gDetector->process(frame, detections);
		logld("[TagDetector] process time = "<<PM.toc()<<" sec.");

		std::vector<cv::Point3f> worldPts;
		std::vector<cv::Point2f> imagePts;
		worldPts.reserve(detections.size());
		imagePts.reserve(detections.size());

		logld(">>> find: ");
		for(int i=0; i<(int)detections.size(); ++i) {
			TagDetection &dd = detections[i];
			if(dd.hammingDistance>0) continue; //strict!

			double worldPt[3];
			if(!gRig.name2worldPt(dd.name(), worldPt)) {
				logli("[AprilCalibprocessor info] ignore tag with id="<<dd.id);
				continue;
			}
			worldPts.push_back(cv::Point3f(worldPt[0],worldPt[1],worldPt[2]));
			imagePts.push_back(cv::Point2f(dd.cxy[0], dd.cxy[1]));

			//visualization
			logld("id="<<dd.id<<", hdist="<<dd.hammingDistance<<", rotation="<<dd.rotation);
			cv::putText( frame, dd.toString(), cv::Point(dd.cxy[0],dd.cxy[1]),
				         CV_FONT_NORMAL, tagTextScale, helper::CV_BLUE, tagTextThickness );
			cv::Mat Homo = cv::Mat(3,3,CV_64FC1,dd.homography[0]);
			helper::drawHomography(frame, Homo);
			cv::circle(frame, cv::Point2d(dd.p[0][0],dd.p[0][1]), 3, helper::CV_GREEN, 2);
		}

		if((worldPts.size()>=8 && gRig.isTag3d)
			|| (worldPts.size()>=4 && !gRig.isTag3d))
		{//TODO: need to judge whether is planar structure
			cv::Mat Ut, Xwt, P;
			cv::Mat(imagePts).reshape(1).convertTo(Ut, cv::DataType<double>::type);
			cv::Mat(worldPts).reshape(1).convertTo(Xwt, cv::DataType<double>::type);
			if(gRig.isTag3d) {
				cv::Mat K0,Rwc,twc;
				helper::dlt3<double>(Ut.t(), Xwt.t(), P);
				helper::decomposeP10<double>(P, K0, Rwc, twc);
				logli("K_dlt="<<K0);
				if(this->K.empty()) K0.copyTo(this->K);
			}

			if(doLog || (isPhoto && useEachValidPhoto)) {
				doLog=false;
				static int cnt=0;
				std::ofstream ofs((outputDir+"/AprilCalib_log_"+helper::num2str(cnt,5)+".m").c_str());
				ofs<<"% AprilCalib log "<<cnt<<std::endl;
				ofs<<"% CalibRig::mode="<<(gRig.isTag3d?"3d":"2d")<<std::endl;
				ofs<<"% @ "<<LogHelper::getCurrentTimeString()<<std::endl;
				ofs<<"U="<<Ut.t()<<";"<<std::endl;
				ofs<<"Xw="<<Xwt.t()<<";"<<std::endl;
				if(gRig.isTag3d) ofs<<"P="<<P<<";"<<std::endl;
	
				if(logVisFrame) cv::imwrite(outputDir+"/AprilCalib_frame_"+helper::num2str(cnt,5)+".png", frame);
				if(!isPhoto) cv::imwrite(outputDir+"/AprilCalib_orgframe_"+helper::num2str(cnt,5)+".png", orgFrame);
				++cnt;

				this->imagePtsArr.push_back(imagePts);
				this->worldPtsArr.push_back(worldPts);
				if(cnt>=2) {
					cv::Mat distCoeffs, CovK;
					if(this->nDistCoeffs>0) {
						distCoeffs=cv::Mat::zeros(nDistCoeffs,1,CV_64FC1);
					}
					std::vector<cv::Mat> rvecs, tvecs, Covrs, Covts;
					double rms=0;
					helper::intrinsicCalibration(imagePtsArr, worldPtsArr,
						gRig.isTag3d,
						cv::Size(frame.cols, frame.rows), K, distCoeffs,
						rvecs, tvecs, rms, &CovK, &Covrs, &Covts);

					if(rms>rmsThresh) { //rms too large usually means gross error
						logli("[AprilCalib warn] rms="<<rms<<", too large, ignore this image.");
						--cnt;
						doLog=true;
						this->imagePtsArr.pop_back();
						this->worldPtsArr.pop_back();
					} else {
						ofs<<"%After LM:"<<std::endl;
						ofs<<"K="<<K<<";"<<std::endl;
						ofs<<"distCoeffs="<<distCoeffs<<";"<<std::endl;
						ofs<<"CovK="<<CovK<<";"<<std::endl;
						ofs<<"%rms="<<rms<<std::endl;
						for(int i=0; i<(int)rvecs.size(); ++i) {
							ofs<<"r"<<i<<"="<<rvecs[i]<<";"<<std::endl;
							ofs<<"t"<<i<<"="<<tvecs[i]<<";"<<std::endl;
							ofs<<"Covr"<<i<<"="<<Covrs[i]<<";"<<std::endl;
							ofs<<"Covt"<<i<<"="<<Covts[i]<<";"<<std::endl;
						}
					}//if rms>rmsThresh
				}//if cnt>=2
				ofs.close();
			}//if doLog
		}//if leftCnt>=4
	}

	void handle(char key) {
		switch (key) {
		case 'd':
			gDetector->segDecimate = !(gDetector->segDecimate);
			logli("[ProcessVideo] detector.segDecimate="<<gDetector->segDecimate); break;
		case '1':
			LogHelper::GetOrSetLogLevel(LogHelper::LOG_DEBUG); break;
		case '2':
			LogHelper::GetOrSetLogLevel(LogHelper::LOG_INFO); break;
		case 'l':
			doLog=true; break;
		case 'h':
			cout<<"d: segDecimate\n"
				"l: do log\n"
				"1: debug output\n"
				"2: info output\n"<<endl; break;
		}
	}

};//end of struct AprilCalibprocessor

void usage(const int argc, const char **argv ) {
	cout<< "[usage] " <<argv[0]<<" [url] [tagfamiliesID]"<<endl;
	cout<< "Supported TagFamily ID List:\n";
	for(int i=0; i<(int)TagFamilyFactory::TAGTOTAL; ++i) {
		cout<<"\t"<<april::tag::TagFamilyFactory_SUPPORT_NAME[i]<<" id="<<i<<endl;
	}
	cout<<"Combination of TagFamily ID: 014 (use tagFamily 0, 1 and 4)"<<endl;
	cout<<"default tagfamiliesID=0"<<endl;
	cout<<"default url=camera://0"<<endl;
	cout<<"Example ImageSource url:\n";
	cout<<"url=photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/*\n";
	cout<<"url=camera://0?w=640?h=480?f=60\n";
	cout<<"url=video:///home/simbaforrest/Videos/Webcam/keg_april.ogv"<<endl;
#ifdef USE_FLYCAP
	cout<<"url=pgr://0?v=5?f=4"<<endl;
#endif
}

int main(const int argc, const char **argv )
{
	LogHelper::GetOrSetLogLevel(LogHelper::LOG_INFO);

	if (argc > 1) {
		std::string arg1(argv[1]);
		if (arg1 == "-h" || arg1 == "/?" || arg1 == "--help") {
			usage(argc, argv);
			return -1;
		}
	}

	ConfigHelper::Config& cfg = GConfig::Instance();
	if(!cfg.autoLoad("AprilCalib.cfg",DirHelper::getFileDir(argv[0]))) {
		logli("[main] no AprilCalib.cfg file loaded");
		return -1;
	}
	if(argc>1) {
		cfg.reset(argc-1, argv+1);
	}
	logli("[main] final Config:");
	cfg->print(std::cout);
	logli("");

	cv::Ptr<ImageSource> is = helper::createImageSource(cfg.get("url","camera://0"));
	if(is.empty()) {
		tagle("createImageSource failed!");
		return -1;
	}
	is->reportInfo();

	//// create tagFamily
	string tagid = cfg.get("tagfamiliesID","0"); //defaul Tag16h5
	TagFamilyFactory::create(tagid, gTagFamilies);
	if(gTagFamilies.size()<=0) {
		tagle("create TagFamily failed all! exit...");
		return -1;
	}

	gDetector = new TagDetector(gTagFamilies);
	if(gDetector.empty()) {
		tagle("create TagDetector fail!");
		return -1;
	}

	gRig.loadFromConfig();
	tagli("the Calibration Rig is:\n"<<std::string(gRig));
	AprilCalibprocessor processor;
	processor.isPhoto = is->isClass<helper::ImageSource_Photo>();
	processor.outputDir = cfg.get("outputDir", is->getSourceDir());
	logli("[main] detection will be logged to outputDir="<<processor.outputDir);
	is->run(processor,-1, false,
			cfg.get<bool>("ImageSource:pause", is->getPause()),
			cfg.get<bool>("ImageSource:loop", is->getLoop()) );

	cout<<"[main] DONE...exit!"<<endl;
	return 0;
}
