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
#define TAG_DEBUG_PERFORMANCE 0
#define TAG_DEBUG_DRAW 0
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

//A set of markers to be used for camera pose optimization
struct MarkerSet {
	enum OPTMETHOD{
		OPT_LM = 0, OPT_EPNP, OPT_CAM, OPT_RAW, OPT_NONE
	} optimizeMethod;
	int nTags;
	std::vector<cv::Point3f> markerXs; //Marker corner's coordinate in Marker's local frame (i.e. model frame)
	std::map<int, int> markerOrder; //markerOrder[detection.id] is the row index of the tag

	MarkerSet() {
		ConfigHelper::Config& cfg = GConfig::Instance();
		std::string method = cfg.getValAsString("MarkerSet::optimizeMethod","lm");
		if(method.compare("lm")==0) {
			optimizeMethod=OPT_LM;
		} else if(method.compare("epnp")==0) {
			optimizeMethod=OPT_EPNP;
		} else if(method.compare("cam")==0) {
			optimizeMethod=OPT_CAM;
		} else if(method.compare("raw")==0) {
			optimizeMethod=OPT_RAW;
		} else {
			optimizeMethod=OPT_NONE;
			logli("[MarkerSet] optimizeMethod not recognized, set to none.");
			return;
		}

		nTags = cfg.get<int>("MarkerSet::nTags",0);
		if(nTags<2) {
			optimizeMethod=OPT_NONE;
			logli("[MarkerSet] MarkerSet::nTags("<<nTags<<")<2, optimization not necessary!");
			return;
		}

		std::vector<int> markerIds(nTags);
		try {
			const int readNumber=cfg.get< std::vector<int> >(
					"MarkerSet::ids",nTags,markerIds);
			if(nTags==readNumber) {
				for(int i=0; i<nTags; ++i) {
					markerOrder[markerIds[i]]=i;
				}
			} else {
				logle("[MarkerSet error] MarkerSet::nTags="<<nTags
						<<", but length of MarkerSet::ids="<<readNumber);
				optimizeMethod=OPT_NONE;
				return;
			}
		} catch (std::exception& e) {
			logle("[MarkerSet error] skip parsing error for "
					"MarkerSet::ids:\n"<<e.what());
			optimizeMethod=OPT_NONE;
			return;
		}

		std::vector<double> buf(nTags*4*3);
		try {
			const int readNumber=cfg.get< std::vector<double> >(
					"MarkerSet::Xs",buf.size(),buf);
			if((int)buf.size()==readNumber) {
				const int nCorners = nTags*4;
				markerXs.resize(nCorners);
				for(int i=0; i<nCorners; ++i) {
					markerXs[i]=cv::Point3f(buf[i*3+0],buf[i*3+1],buf[i*3+2]);
				}
			} else {
				logle("[MarkerSet error] fail to read "<<buf.size()
						<<" numbers for MarkerSet::Xs");
				optimizeMethod=OPT_NONE;
				return;
			}
		} catch (std::exception& e) {
			logle("[MarkerSet error] skip parsing error for "
					"MarkerSet::Xs:\n"<<e.what());
			optimizeMethod=OPT_NONE;
			return;
		}
	}

	double reProjError(const std::vector<cv::Point3f>& X,
			const std::vector<cv::Point2f>& U, const cv::Mat& K,
			const cv::Mat& distCoeffs, cv::Mat& r, cv::Mat& t) const {
		assert(X.size()>0);
		std::vector<cv::Point2f> V;
		cv::projectPoints(X, r, t, K, distCoeffs, V);
		assert(V.size()==U.size());
		double ret=0;
		for(int i=0; i<(int)V.size(); ++i) {
			const cv::Point2f& u=U[i];
			const cv::Point2f& v=V[i];
			ret+=(u.x-v.x)*(u.x-v.x)+(u.y-v.y)*(u.y-v.y);
		}
		return cv::sqrt(ret/(double)V.size());
	}

	//X: marker corner's 3D coords
	//U: corresponding raw image points (i.e. without undistort yet)
	//K: camera calibration matrix
	//distCoeffs: distortion coefficients
	//R: Rwc, from world (or model) frame to camera frame
	//t: twc, similar to R
	//return re-projection error
	double optimize(const std::vector<cv::Point3f>& X,
			const std::vector<cv::Point2f>& U, const cv::Mat& K,
			const cv::Mat& distCoeffs, cv::Mat& R, cv::Mat& t) const {
		if(optimizeMethod==OPT_NONE) return -1; //shouldn't reach here, just for safe
		if(optimizeMethod==OPT_EPNP || optimizeMethod==OPT_LM) {
			cv::Mat r;
			cv::solvePnP(X,U,K,distCoeffs,r,t,false,optimizeMethod==OPT_EPNP?CV_EPNP:CV_ITERATIVE);
			cv::Rodrigues(r,R);
			return reProjError(X,U,K,distCoeffs,r,t);
		}
		if (optimizeMethod == OPT_RAW) {
			//TODO: temporarily assumes X are xy-planar and use homography and RTfromKH for OPT_RAW
			cv::Mat u; //undistorted image points
			cv::undistortPoints(U, u, K, distCoeffs);
			std::vector<cv::Point2f> x(X.size());
			for (int i = 0; i < (int) x.size(); ++i) {
				if (X[i].z != 0) {
					logle("[MarkerSet::optimize error] all z coordinate"
							" of MarkerSet::X should be zero if OPT_RAW!");
					return -1;
				}
				x[i] = cv::Point2f(X[i].x, X[i].y);
			}
			cv::Mat Hmi = cv::findHomography(x, u); //u=Hmi*x, Hmi maps marker coords => image coords
			R.create(3, 3, CV_64FC1);
			t.create(3, 1, CV_64FC1);
			helper::RTfromKH(K.ptr<double>(0), Hmi.ptr<double>(0),
					R.ptr<double>(0), t.ptr<double>(0), true);
			cv::Mat r;
			cv::Rodrigues(R,r);
			return reProjError(X,U,K,distCoeffs,r,t);
		}
		if (optimizeMethod == OPT_CAM) { //TODO: add support to optimize both intrinsic and extrinsics together
			logle("[MarkerSet::optimize error] OPT_CAM not supported yet...");
			return -1;
		}
		return -1;
	}
};

struct AprilTagprocessor : public ImageHelper::ImageSource::Processor {
	double tagTextScale;
	int tagTextThickness;
	bool doLog,doRecord;
	bool isPhoto; //whether image source is photo/list or others
	bool useEachValidPhoto; //whether do log for each frame
	bool logVisFrame;
	std::string outputDir;

	bool undistortImage;
	int hammingThresh;

	cv::Mat K, distCoeffs;
	bool no_distortion;

	MarkerSet markerSet;

	virtual ~AprilTagprocessor() {}
	AprilTagprocessor() : doLog(false), doRecord(false), isPhoto(false), no_distortion(true) {
		ConfigHelper::Config& cfg = GConfig::Instance();
		tagTextScale = cfg.get<double>("AprilTagprocessor::tagTextScale",1.0f);
		tagTextThickness = cfg.get<int>("AprilTagprocessor::tagTextThickness",1);
		useEachValidPhoto = cfg.get<bool>("AprilTagprocessor::useEachValidPhoto",false);
		hammingThresh = cfg.get<int>("AprilTagprocessor::hammingThresh",0);
		logVisFrame = cfg.get<bool>("AprilTagprocessor::logVisFrame",false);
		undistortImage = cfg.get<int>("AprilTagprocessor::undistortImage",false);
		gDetector->segDecimate = cfg.get<bool>("AprilTag::segDecimate",false);

		loadIntrinsics();
	}

	void loadIntrinsics() {
		{
			double K_[9];
			if(9!=GConfig::Instance().get<double[9]>("K",9,K_)) {
				logli("[loadIntrinsics warn] calibration matrix K"
					" not correctly specified in config!");
				this->undistortImage=false;
				this->no_distortion=true;
				return;
			}
			cv::Mat(3,3,CV_64FC1,K_).copyTo(K);
			K/=K_[8];
		}

		{
			double distCoeffs_[5]={0,0,0,0,0};
			if(5!=GConfig::Instance().get<double[5]>("distCoeffs",5,distCoeffs_)) {
				logli("[loadIntrinsics warn] distortion coefficients "
					"distCoeffs not correctly specified in config! Assume all zero!");
				for(int i=0; i<5; ++i) distCoeffs_[i]=0;
			}
			double sum=distCoeffs_[0]+distCoeffs_[1]
			+distCoeffs_[2]+distCoeffs_[3]+distCoeffs_[4];
			this->no_distortion=(sum==0);
			if(this->no_distortion) this->undistortImage=false;
			cv::Mat(5,1,CV_64FC1,distCoeffs_).copyTo(distCoeffs);
		}

		logli("[loadIntrinsics] K="<<K);
		logli("[loadIntrinsics] distCoeffs="<<distCoeffs);
	}

	/**
	write the detection in matlab script format:
	tag.id <1x1>, tag.H <3x3>, tag.p <2x4>, tag.c <2x1> [tag.up <2x4>]
	*/
	void writeMatlab(std::ostream& os, TagDetection &dd,
		std::string varname="tag", bool ud=false)
	{
		const cv::Mat H(3,3,CV_64FC1,(void*)dd.homography[0]);
		const cv::Mat p(4,2,CV_64FC1,(void*)dd.p[0]);
		const cv::Mat c(2,1,CV_64FC1,(void*)dd.cxy);
		os<<varname<<".id="<<dd.id<<";\n"
			<<varname<<".hammingDistance="<<dd.hammingDistance<<";\n"
			<<varname<<".familyName='"<<dd.familyName<<"';\n"
			<<varname<<".H="<<H<<";\n"
			<<varname<<".p="<<p<<"';\n"
			<<varname<<".c="<<c<<";"<<std::endl;

		if(!ud) return;

		dd.undistort<double>(this->K, this->distCoeffs);
		os<<varname<<".uH="<<H<<"';"<<std::endl;
		os<<varname<<".up="<<p<<"';"<<std::endl;
		os<<varname<<".uc="<<c<<";"<<std::endl;
	}
/////// Override
	void operator()(cv::Mat& frame) {
		static helper::PerformanceMeasurer PM;
		vector<TagDetection> detections;
		cv::Mat orgFrame;
		frame.copyTo(orgFrame);
		PM.tic();
		if(this->undistortImage) cv::undistort(orgFrame,frame,K,distCoeffs);
		gDetector->process(frame, detections);
		logld("[TagDetector] process time = "<<PM.toc()<<" sec.");

		//visualization
		int nValidDetections=0;
		logld(">>> find: ");
		for(int i=0; i<(int)detections.size(); ++i) {
			TagDetection &dd = detections[i];
			if(dd.hammingDistance>this->hammingThresh) continue;
			++nValidDetections;

			logld("id="<<dd.id<<", hdist="<<dd.hammingDistance<<", rotation="<<dd.rotation);
			cv::putText( frame, dd.toString(), cv::Point(dd.cxy[0],dd.cxy[1]),
				         CV_FONT_NORMAL, tagTextScale, helper::CV_BLUE, tagTextThickness );
			cv::Mat Homo = cv::Mat(3,3,CV_64FC1,dd.homography[0]);
			helper::drawHomography(frame, Homo);
			cv::circle(frame, cv::Point2d(dd.p[0][0],dd.p[0][1]), 3, helper::CV_GREEN, 2);
		}

		//logging results
		if(nValidDetections>0 && (doLog || (isPhoto && useEachValidPhoto) || (!isPhoto && doRecord))) {
			doLog=false;
			static int cnt=0;
			std::string fileid = helper::num2str(cnt, 5);

			//log images
			if(logVisFrame) cv::imwrite(outputDir+"/AprilTagFinder_frame_"+fileid+".png", frame);
			if(!isPhoto) cv::imwrite(outputDir+"/AprilTagFinder_orgframe_"+fileid+".png", orgFrame);

			//log detections
			std::ofstream fs((outputDir+"/AprilTagFinder_log_"+fileid+".m").c_str());
			fs<<"% AprilTagFinder log "<<cnt<<std::endl;
			fs<<"% @ "<<LogHelper::getCurrentTimeString()<<std::endl;
			fs<<"K="<<K<<";"<<std::endl;
			fs<<"distCoeffs="<<distCoeffs<<";"<<std::endl;
			fs<<"tags={};\n"<<std::endl;
			for(int i=0,j=0; i<(int)detections.size(); ++i) {
				TagDetection &dd = detections[i];
				if(dd.hammingDistance>this->hammingThresh) continue;
				++j;//note matlab uses 1-based index

				writeMatlab(fs, dd, "tag", !this->undistortImage && !this->no_distortion);
				fs<<"tags{"<<j<<"}=tag;\n"<<std::endl;
			}
			fs.close();

			//camera pose optimization using MarkerSet
			if(markerSet.optimizeMethod!=MarkerSet::OPT_NONE && nValidDetections>1) {
				std::vector< cv::Point2f > U;
				std::vector< cv::Point3f > X;
				U.reserve(nValidDetections*4);
				X.reserve(nValidDetections*4);
				for(int i=0; i<(int)detections.size(); ++i) {
					TagDetection &dd = detections[i];
					if(dd.hammingDistance>this->hammingThresh) continue;

					std::map<int,int>::const_iterator itr=markerSet.markerOrder.find(dd.id);
					if(itr==markerSet.markerOrder.end()) continue;

					const int rowID=itr->second*4;
					for(int k=0; k<4; ++k) { //each marker <=> 4 corners <=> 4 2D-3D correspondences
						U.push_back(cv::Point2f(dd.p[k][0], dd.p[k][1]));
						X.push_back(markerSet.markerXs[rowID+k]);
					}
				}
				cv::Mat Rwc, twc;
				double err=-1;
				if((err=markerSet.optimize(X,U,this->K,this->distCoeffs,Rwc,twc))<0) {
					logle("[MarkerSet::optimize error] optimization failed, strange...");
				} else {
					//log optimization
					std::ofstream os(
							(outputDir + "/AprilTagFinder_opt_" + fileid + ".m").c_str());
					os << "% AprilTagFinder optimization log " << cnt << std::endl;
					os << "% @ " << LogHelper::getCurrentTimeString() << std::endl;
					os << "K=" << K << ";" << std::endl;
					os << "distCoeffs=" << distCoeffs << ";" << std::endl;
					os << "% note U are distorted image points (raw image points)" << std::endl;
					os << "U=" << cv::Mat(U).reshape(1) <<"';" << std::endl;
					os << "X=" << cv::Mat(X).reshape(1) <<"';" << std::endl;
					os << "Rwc=" << Rwc << ";" << std::endl;
					os << "twc=" << twc << ";" << std::endl;
					os << "%rms=" << err << std::endl;
					os << "optimizeMethod=" << (int)markerSet.optimizeMethod << std::endl;
					os.close();
				}
			}//if(!OPT_NONE)

			++cnt;
		}//if doLog

		if(frame.cols>640) {
			cv::resize(frame, frame, cv::Size(640,480));
		}
	}

	void handle(char key) {
		switch (key) {
		case 'd':
			gDetector->segDecimate = !(gDetector->segDecimate);
			logli("[ProcessVideo] gDetector.segDecimate="<<gDetector->segDecimate); break;
		case 'l':
			doLog=true; break;
		case 'r':
			doRecord=true; break;
		case '1':
			LogHelper::GLogControl::Instance().level = LogHelper::LOG_DEBUG; break;
		case '2':
			LogHelper::GLogControl::Instance().level = LogHelper::LOG_INFO; break;
		case 'h':
			cout<<"d: segDecimate\n"
				"l: do log\n"
				"r: do record\n"
				"1: debug output\n"
				"2: info output\n"<<endl; break;
		}
	}

};//end of struct AprilTagprocessor

void usage(const int argc, const char **argv ) {
	cout<< "[usage] " <<argv[0]<<" <url> [TagFamilies ID]"<<endl;
	cout<< "Supported TagFamily ID List:\n";
	for(int i=0; i<(int)TagFamilyFactory::TAGTOTAL; ++i) {
		cout<<"\t"<<april::tag::TagFamilyFactory_SUPPORT_NAME[i]<<" id="<<i<<endl;
	}
	cout<<"Combination of TagFamily ID: 014 (use tagFamily 0, 1 and 4)"<<endl;
	cout<<"default ID: 0"<<endl;
	cout<<"Example ImageSource url:\n";
	cout<<"photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/*\n";
	cout<<"camera://0?w=640?h=480?f=60\n";
	cout<<"video:///home/simbaforrest/Videos/Webcam/keg_april.ogv"<<endl;
#ifdef USE_FLYCAP
	cout<<"pgr://0?v=5?f=4"<<endl;
#endif
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
	const int URL_POS=1, TAG_FAMILY_ID_POS=2;
	if(argc<MIN_ARGS) {
		usage(argc,argv);
		return -1;
	}

	ConfigHelper::Config& cfg = GConfig::Instance();
	if(!loadConfig("AprilTagFinder.cfg",DirHelper::getFileDir(argv[0]))) {
		logli("[main] no AprilTagFinder.cfg file loaded");
	}
	if(argc>CFG_ARGS_START) {
		logli("[main] add/reset config info from command line arguments.");
		cfg.set(argc-CFG_ARGS_START, argv+CFG_ARGS_START);
	}

	cv::Ptr<ImageSource> is = helper::createImageSource(argv[URL_POS]);
	if(is.empty()) {
		tagle("createImageSource failed!");
		return -1;
	}
	is->reportInfo();

	//// create tagFamily
	string tagid("0"); //default tag16h5
	if(argc>TAG_FAMILY_ID_POS) tagid = string(argv[TAG_FAMILY_ID_POS]);
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

	AprilTagprocessor processor;
	processor.isPhoto = is->isClass<helper::ImageSource_Photo>();
	processor.outputDir = cfg.getValAsString("AprilTagprocessor::outputDir",
		is->getSourceDir());
	logli("[main] detection will be logged to outputDir="<<processor.outputDir);
	is->run(processor,-1, false,
		cfg.get<bool>("ImageSource::pause", is->getPause()),
		cfg.get<bool>("ImageSource::loop", is->getLoop()) );

	cout<<"[main] DONE...exit!"<<endl;
	return 0;
}
