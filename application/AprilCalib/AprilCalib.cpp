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

#include "OpenCVHelper.h"
#include "apriltag/apriltag.hpp"
#include "apriltag/TagFamilyFactory.hpp"
#include "config.hpp"

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
Calibration rig is composed of two planes, usually one on left and one on right.
World Frame is defined as the left frame. Left plane and right plane are grids
of AprilTags, each with rows*cols AprilTags with id start from start_id. Left
frame origin is at the center of AprilTags on the left plane, the orientation of
x, y, z axis is the same as the AprilTags orientation; similarly for right frame.
For example:
if left_start_id=0, left_rows=3, left_cols=3, then the left frame looks like
  ^y
  |
0 1 2
3 4-5->x
6 7 8
the origin of left frame is the center of tag 4, tag 4 to tag 5 defines positive x
direction and tag4 to tag 1 defines positive y direction, positive z direction is
thus pointing out of screen by right-hand-rule
*/
struct CalibRig {
	double Rrl[9], trl[3]; //points on left plane = [Rrl, trl;000, 1] * points on right plane
	int left_start_id, left_rows, left_cols;
	double left_tag_black_length;
	int right_start_id, right_rows, right_cols;
	double right_tag_black_length;

	double left_tag_len, right_tag_len;

	CalibRig(): left_start_id(-1), left_rows(-1), left_cols(-1), left_tag_black_length(-1),
		right_start_id(-1), right_rows(-1), right_cols(-1), right_tag_black_length(-1),
		left_tag_len(-1), right_tag_len(-1)
	{
		for(int i=0; i<3; ++i) {
			for(int j=0; j<3; ++j) {
				Rrl[i*3+j]=0;
			}
			trl[i]=0;
			Rrl[i*3+i]=1;
		}
	}

	operator std::string() const {
		std::stringstream ss;
#define report_arr(varname,len) ss<< #varname << "="; \
	for(int i=0; i<len; ++i) ss<<varname[i]<<" "; ss<<std::endl
		report_arr(Rrl,9);
		report_arr(trl,3);
#undef report_arr
#define report_var(varname) ss<< #varname << "=" << varname <<std::endl
		report_var(left_start_id);
		report_var(left_rows);
		report_var(left_cols);
		report_var(left_tag_black_length);
		report_var(left_tag_len);

		report_var(right_start_id);
		report_var(right_rows);
		report_var(right_cols);
		report_var(right_tag_black_length);
		report_var(right_tag_len);
#undef report_var
		return ss.str();
	}

	void loadFromConfig() {
		{std::stringstream ss;
		ss.str(GConfig::Instance().getRawString("Rrl"));
		for(int i=0; i<9; ++i) ss>>Rrl[i];}

		{std::stringstream ss;
		ss.str(GConfig::Instance().getRawString("trl"));
		for(int i=0; i<3; ++i) ss>>trl[i];}

#define get_var(varname) {std::stringstream ss; \
	ss.str(GConfig::Instance().getRawString( #varname )); \
	ss>>varname;}
		get_var(left_start_id);
		get_var(left_rows);
		get_var(left_cols);
		get_var(left_tag_black_length);

		get_var(right_start_id);
		get_var(right_rows);
		get_var(right_cols);
		get_var(right_tag_black_length);
#undef get_var

		left_tag_len=(double)(left_tag_black_length*gTagFamilies[0]->getTagRenderDimension())
					 /(double)(gTagFamilies[0]->d+2*gTagFamilies[0]->blackBorder);
		right_tag_len=(double)(right_tag_black_length*gTagFamilies[0]->getTagRenderDimension())
					  /(double)(gTagFamilies[0]->d+2*gTagFamilies[0]->blackBorder);
	}

	int id2worldPt(const int id, double worldPt[3]) const {
		if(left_start_id<=id && id<(left_start_id+left_rows*left_cols)) {
			const double worldW=left_tag_len*left_cols, worldH=left_tag_len*left_rows;
			const int nid=id-left_start_id;
			const int r=nid/left_cols;
			const int c=nid-r*left_cols;
			double worldx=left_tag_len/2+c*left_tag_len;
			double worldy=left_tag_len/2+r*left_tag_len;
			worldx-=worldW/2;
			worldy=worldH/2-worldy;
			worldPt[0]=worldx;
			worldPt[1]=worldy;
			worldPt[2]=0;
			return 1;
		} else if(right_start_id<=id && id<(right_start_id+right_rows*right_cols)) {
			const double worldW=right_tag_len*right_cols, worldH=right_tag_len*right_rows;
			const int nid=id-right_start_id;
			const int r=nid/right_cols;
			const int c=nid-r*right_cols;
			double worldx=right_tag_len/2+c*right_tag_len;
			double worldy=right_tag_len/2+r*right_tag_len;
			worldx-=worldW/2;
			worldy=worldH/2-worldy;
			//transform to world frame (i.e. left frame)
			worldPt[0]=Rrl[0]*worldx+Rrl[1]*worldy+trl[0];
			worldPt[1]=Rrl[3]*worldx+Rrl[4]*worldy+trl[1];
			worldPt[2]=Rrl[6]*worldx+Rrl[7]*worldy+trl[2];
			return -1;
		}
		return 0;
	}
} gRig;

struct AprilCalibprocessor : public ImageHelper::ImageSource::Processor {
	double tagTextScale;
	int tagTextThickness;
	bool doLog;
	bool isPhoto;
	std::vector<std::vector<cv::Point2f> > imagePtsArr;
	std::vector<std::vector<cv::Point3f> > worldPtsArr;
	AprilCalibprocessor() : doLog(false), isPhoto(false) {
		tagTextScale = GConfig::Instance().get<double>("AprilCalibprocessor::tagTextScale",1.0f);
		tagTextThickness = GConfig::Instance().get<int>("AprilCalibprocessor::tagTextThickness",1);
	}
/////// Override
	void operator()(cv::Mat& frame) {
		static helper::PerformanceMeasurer PM;
		vector<TagDetection> detections;
		PM.tic();
		cv::Mat orgFrame;
		frame.copyTo(orgFrame);
		gDetector->process(frame, detections);
		logld("[TagDetector] process time = "<<PM.toc()<<" sec.");

		std::vector<cv::Point3f> worldPts;
		std::vector<cv::Point2f> imagePts;
		worldPts.reserve(detections.size());
		imagePts.reserve(detections.size());
		int leftCnt=0, rightCnt=0;

		logld(">>> find: ");
		for(int id=0; id<(int)detections.size(); ++id) {
			TagDetection &dd = detections[id];
			if(dd.hammingDistance>0) continue; //strict!

			double worldPt[3];
			int leftRightCnt=0;
			if((leftRightCnt=gRig.id2worldPt(dd.id, worldPt))==0) {
				logli("[AprilCalibprocessor info] ignore tag with id="<<dd.id);
				continue;
			}
			if(leftRightCnt>0) leftCnt++;
			else rightCnt++;
			worldPts.push_back(cv::Point3f(worldPt[0],worldPt[1],worldPt[2]));
			imagePts.push_back(cv::Point2f(dd.cxy[0], dd.cxy[1]));

			//visualization
			logld("id="<<dd.id<<", hdist="<<dd.hammingDistance<<", rotation="<<dd.rotation);
			cv::putText( frame, dd.toString(), cv::Point(dd.cxy[0],dd.cxy[1]),
				         CV_FONT_NORMAL, tagTextScale, helper::CV_BLUE, tagTextThickness );
			cv::Mat Homo = cv::Mat(3,3,CV_64FC1,dd.homography[0]);
			static double crns[4][2]={
				{-1, -1},
				{ 1, -1},
				{ 1,  1},
				{-1,  1}
			};
			helper::drawHomography(frame, Homo, crns);
			cv::circle(frame, cv::Point2d(dd.p[0][0],dd.p[0][1]), 3, helper::CV_GREEN, 2);
		}

		if(leftCnt>=4 && rightCnt>=4) {
			cv::Mat P,K,Rwc,twc;
			cv::Mat Ut, Xwt;
			cv::Mat(imagePts).reshape(1).convertTo(Ut, cv::DataType<double>::type);
			cv::Mat(worldPts).reshape(1).convertTo(Xwt, cv::DataType<double>::type);
			helper::dlt3<double>(Ut.t(), Xwt.t(), P);
			helper::decomposeP10<double>(P, K, Rwc, twc);
			//cv::decomposeProjectionMatrix(P, K, Rwc, twc);
			//K/=K.at<double>(2,2);
			std::cout<<"K="<<K<<std::endl;

			if(doLog || isPhoto) {
				doLog=false;
				static int cnt=0;
				std::clog<<"log "<<cnt<<std::endl;
				std::clog<<"U="<<Ut.t()<<std::endl;
				std::clog<<"Xw="<<Xwt.t()<<std::endl;
				std::clog<<"P="<<P<<std::endl;
				std::clog<<"K="<<K<<std::endl;
				
				cv::imwrite(helper::num2str(cnt,5)+"_frame.png", frame);
				cv::imwrite(helper::num2str(cnt,5)+"_orgFrame.png", orgFrame);
				++cnt;

				this->imagePtsArr.push_back(imagePts);
				this->worldPtsArr.push_back(worldPts);
				if(cnt>=2) {
					cv::Mat distCoeffs;
					std::vector<cv::Mat> rvecs, tvecs;
					double rms=0;
					helper::calibration3d(imagePtsArr, worldPtsArr,
						cv::Size(frame.cols, frame.rows), K, distCoeffs,
						rvecs, tvecs, rms);
					std::clog<<"After LM:"<<std::endl;
					std::clog<<"K="<<K<<std::endl;
					std::clog<<"distCoeffs="<<distCoeffs<<std::endl;
					std::clog<<"rms="<<rms<<std::endl;
				}
			}
		}

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

void usage( int argc, char **argv ) {
	cout<< "[usage] " <<argv[0]<<" <url> [TagFamilies ID]"<<endl;
	cout<< "Assume an AprilCalib.cfg file at the same directory with"<<argv[0]<<std::endl;
	cout<< "Supported TagFamily ID List:\n";
	for(int i=0; i<(int)TagFamilyFactory::TAGTOTAL; ++i) {
		cout<<"\t"<<april::tag::TagFamilyFactory_SUPPORT_NAME[i]<<" id="<<i<<endl;
	}
	cout<<"default ID: 0"<<endl;
}

int main( int argc, char **argv )
{
	LogHelper::GLogControl::Instance().level = LogHelper::LOG_INFO;

	if(argc<2) {
		usage(argc,argv);
		return -1;
	}

	ConfigHelper::Config& cfg = GConfig::Instance();
	std::string exeDir=helper::getFileDir(argv[0]);
	if(!cfg.load(exeDir+"AprilCalib.cfg")) {
		logli("[main] no "<<exeDir<<"AprilCalib.cfg file");
		return -1;
	} else {
		logli("[main] loaded "<<exeDir<<"AprilCalib.cfg");
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
	for(int i=0; i<(int)tagid.size(); ++i) {
		const char curchar[] = {tagid[i],'\0'};
		unsigned int curid = atoi(curchar);//atoi works on an array of char, not on a single char!!
		cv::Ptr<TagFamily> tagFamily = TagFamilyFactory::create(curid);
		if(tagFamily.empty()) {
			tagle("create TagFamily "<<curid<<" fail, skip!");
			continue;
		}
		gTagFamilies.push_back(tagFamily);
		break;//in this app we only need one tagFamily, keep the first one
	}
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
	processor.isPhoto = (is->classname()==std::string("ImageSource_Photo"));
	is->run(processor,-1, false,
			cfg.get<bool>("ImageSource::pause", is->getPause()),
			cfg.get<bool>("ImageSource::loop", is->getLoop()) );

	cout<<"[main] DONE...exit!"<<endl;
	return 0;
}
