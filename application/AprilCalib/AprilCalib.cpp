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
CalibRig::nTags=N
CalibRig::start_id=S
#repeat follow lines N times with i=S...S+N-1, (Xi, Yi, Zi) is the world coordinate
#of the center of the tagi
CalibRig::tagi=Xi Yi Zi
...

For example, if there are 18 Tags used as a calibration rig, id start from 3, then
the following lines should appear in AprilCalib.cfg file
CalibRig::mode=3d
#optional, default start_id is 0
CalibRig::start_id=3
CalibRig::nTags=18
CalibRig::tagCenters=[X3 Y3 Z3;
 X4 Y4 Z4;
...
 X20 Y20 Z20]

Calibration rig defined above is a 3d rig, we can also use 2d rig (planar rig), which
can be defined as follows:
CalibRig::mode=2d
#assume the upper left tag is the first tag
#i.e. tags are assumed to be arrange as follows:
#tag0 tag1 tag2 ... tagN-1
#tagN ...           tag2N-1
#...
#tagMN-N ...        tagMN-1
CalibRig::start_id=S
CalibRig::w=N
CalibRig::h=M
#dx means distance between two neighboring tags along width direction
#dy means distance between two neighboring tags along height direction
#center of tag0 is assumed to have coordinate (0,0) in the world plane
CalibRig::dx=Dx
CalibRig::dy=Dy
*/
struct CalibRig {
	bool isTag3d;
	int start_id;

	//for 3d rig
	int nTags;
	std::vector<cv::Point3d> id2Xw;

	//for 2d rig
	int w,h;
	double dx,dy;

	CalibRig() : isTag3d(true), start_id(-1),
		nTags(-1),
		w(0),h(0),dx(0),dy(0)
	{}

	operator std::string() const {
		std::stringstream ss;
		ss<<"CalibRig::start_id="<<this->start_id<<"\n";
		if(isTag3d) {
			ss<<"CalibRig::mode=3d\n";
			ss<<"CalibRig::nTags="<<this->nTags<<"\n"
			  <<"CalibRig::tagCenters=[\n";
			for(int i=0; i<(int)id2Xw.size(); ++i) {
				const cv::Point3d& pt=id2Xw[i];
				ss<<pt.x<<" "<<pt.y<<" "<<pt.z<<"\n";
			}
			ss<<"]\n";
		} else {
			ss<<"CalibRig::mode=2d\n";
			ss<<"CalibRig::w="<<this->w<<"\n"
			  <<"CalibRig::h="<<this->h<<"\n"
			  <<"CalibRig::dx="<<this->dx<<"\n"
			  <<"CalibRig::dy="<<this->dy<<"\n";
		}
		return ss.str();
	}

	void loadFromConfig() {
		ConfigHelper::Config& cfg = GConfig::Instance();
		
		//get rig mode and start_id
		std::string mode=cfg.get<std::string>("CalibRig::mode");
		this->start_id=cfg.get<int>("CalibRig::start_id",0);
		if(mode.compare("3d")==0) {
			this->isTag3d=true;
			this->nTags=cfg.get<int>("CalibRig::nTags",0);
			if(nTags<8) {
				logle("[CalibRig error] a 3D calibration rig must have more than 8 tags!");
				exit(-1);
			}
			std::vector<double> buf(nTags*3);
			if( !cfg.get< double, std::vector<double> >("CalibRig::tagCenters", nTags*3, buf) ) {
				logle("[CalibRig error] not CalibRig::tagCenters in Config!");
				exit(-1);
			}//TODO: check if the points forms a 3d rig really by PCA
			this->id2Xw.resize(nTags);
			for(int i=0; i<nTags; ++i) {
				id2Xw[i]=cv::Point3d(buf[i*3+0],buf[i*3+1],buf[i*3+2]);
			}
		} else {
			this->isTag3d=false; //2d rig
			this->w=cfg.get<int>("CalibRig::w",0);
			this->h=cfg.get<int>("CalibRig::h",0);
			this->nTags=this->w*this->h;
			if(this->nTags<4) {
				logle("[CalibRig error] a 2D calibration rig must have at least 4 tags!");
				exit(-1);
			}
			if(this->w<=1 || this->h<=1) {
				logle("[CalibRig error] a 2D calibration rig must ensure w>1 and h>1!");
				exit(-1);
			}
			this->dx=cfg.get<double>("CalibRig::dx",0);
			this->dy=cfg.get<double>("CalibRig::dy",0);
			if(this->dx<=0 || this->dy<=0) {
				logle("[CalibRig error] a 2D calibration rig must ensure dx>0 and dy>0!");
				exit(-1);
			}
		}
	}

	bool id2worldPt(const int id, double worldPt[3]) const {
		const int rid=id-start_id;
		if(rid<0 || rid>=nTags) return false;
		if(isTag3d) {
			const cv::Point3d& pt=id2Xw[rid];
			worldPt[0]=pt.x;
			worldPt[1]=pt.y;
			worldPt[2]=pt.z;
		} else {
			int r=rid/this->w;
			int c=rid-r*this->w;
			worldPt[0]=c*this->dx;
			worldPt[1]=r*this->dy;
			worldPt[2]=0;
		}
		return true;
	}
} gRig;

struct AprilCalibprocessor : public ImageHelper::ImageSource::Processor {
	double tagTextScale;
	int tagTextThickness;
	bool doLog;
	bool isPhoto;
	bool useEachValidPhoto;
	std::string outputDir;

	int nDistCoeffs;
	std::vector<std::vector<cv::Point2f> > imagePtsArr;
	std::vector<std::vector<cv::Point3f> > worldPtsArr;
	cv::Mat K;
	double rmsThresh;
	AprilCalibprocessor() : doLog(false), isPhoto(false) {
		tagTextScale = GConfig::Instance().get<double>("AprilCalibprocessor::tagTextScale",1.0f);
		tagTextThickness = GConfig::Instance().get<int>("AprilCalibprocessor::tagTextThickness",1);
		useEachValidPhoto = GConfig::Instance().get<int>("AprilCalibprocessor::useEachValidPhoto",false);
		rmsThresh = GConfig::Instance().get<int>("AprilCalibprocessor::rmsThresh",2);
		nDistCoeffs = GConfig::Instance().get<int>("AprilCalib::nDistCoeffs",0);
		if(nDistCoeffs>5) {
			nDistCoeffs=5;
			logli("[AprilCalibprocessor warn] AprilCalib::nDistCoeffs>5, set back to 5!");
		}
		gDetector->segDecimate = GConfig::Instance().get<bool>("AprilTag::segDecimate",false);
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
			if(!gRig.id2worldPt(dd.id, worldPt)) {
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
	
				cv::imwrite(outputDir+"/AprilCalib_frame_"+helper::num2str(cnt,5)+".png", frame);
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

		if(frame.cols>640) {
			cv::resize(frame, frame, cv::Size(640,480));
		}
	}

	void handle(char key) {
		switch (key) {
		case 'd':
			gDetector->segDecimate = !(gDetector->segDecimate);
			logli("[ProcessVideo] detector.segDecimate="<<gDetector->segDecimate); break;
		case '1':
			LogHelper::GLogControl::Instance().level = LogHelper::LOG_DEBUG; break;
		case '2':
			LogHelper::GLogControl::Instance().level = LogHelper::LOG_INFO; break;
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
	cout<< "[usage] " <<argv[0]<<" <url> [TagFamilies ID]"<<endl;
	cout<< "Assume an AprilCalib.cfg file at the same directory with"<<argv[0]<<std::endl;
	cout<< "Supported TagFamily ID List:\n";
	for(int i=0; i<(int)TagFamilyFactory::TAGTOTAL; ++i) {
		cout<<"\t"<<april::tag::TagFamilyFactory_SUPPORT_NAME[i]<<" id="<<i<<endl;
	}
	cout<<"default ID: 0"<<endl;
}

bool loadConfig(std::string fname, std::string fdir) {
	ConfigHelper::Config& cfg = GConfig::Instance();
	std::string fpath = fdir+fname;
	if (cfg.load(fpath))
		return true;
	logli("tried but failed to load "<<fpath);

	//1. try to use current dir
	fpath = DirHelper::getCurrentDir()+"/"+fname;
	if (cfg.load(fpath))
		return true;
	logli("tried but failed to load "<<fpath);

	//2. try to search in path
	std::vector<std::string> all_path=DirHelper::getEnvPath();
	for(int i=0; i<(int)all_path.size(); ++i) {
		std::string& path=all_path[i];
		if(cfg.load(path+"/"+fname)) return true;
		logli("tried but failed to load "<<path);
	}
	return false;
}
int main(const int argc, const char **argv )
{
	LogHelper::GLogControl::Instance().level = LogHelper::LOG_INFO;

	const int MIN_ARGS=2, CFG_ARGS_START=3;
	if(argc<MIN_ARGS) {
		usage(argc,argv);
		return -1;
	}

	ConfigHelper::Config& cfg = GConfig::Instance();
	if(!loadConfig("AprilCalib.cfg",DirHelper::getFileDir(argv[0]))) {
		logli("[main] no AprilCalib.cfg file loaded");
		return -1;
	}
	if(argc>CFG_ARGS_START) {
		logli("[main] add/reset config info from command line arguments.");
		cfg.set(argc-CFG_ARGS_START, argv+CFG_ARGS_START);
	}

	cv::Ptr<ImageSource> is = helper::createImageSource(argv[1]);
	if(is.empty()) {
		tagle("createImageSource failed!");
		return -1;
	}
	is->reportInfo();

	//// create tagFamily
	string tagid("0"); //default tag16h5
	if(argc>2) tagid = string(argv[2]);
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
	processor.outputDir = cfg.getValAsString("AprilCalibprocessor::outputDir",
		is->getSourceDir());
	logli("[main] detection will be logged to outputDir="<<processor.outputDir);
	is->run(processor,-1, false,
			cfg.get<bool>("ImageSource::pause", is->getPause()),
			cfg.get<bool>("ImageSource::loop", is->getLoop()) );

	cout<<"[main] DONE...exit!"<<endl;
	return 0;
}
