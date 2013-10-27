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
#include "Log.h"
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

std::vector< cv::Ptr<TagFamily> > tagFamilies;
cv::Ptr<TagDetector> detector;

void usage( int argc, char **argv ) {
	cout<< "[usage] " <<argv[0]<<" <left.png> <right.png> <parameters.cfg> [TagFamilies ID]"<<endl;
	cout<< "Supported TagFamily ID List:\n";
	for(int i=0; i<(int)TagFamilyFactory::TAGTOTAL; ++i) {
		cout<<"\t"<<april::tag::TagFamilyFactory_SUPPORT_NAME[i]<<" id="<<i<<endl;
	}
	cout<<"default ID: 0"<<endl;
}

int getImageWithDetections(std::string fname,
						   cv::Mat& img,
						   std::vector<TagDetection>& detections)
{
	img = cv::imread(fname);
	if(img.empty()) return 0;
	detections.clear();
	detector->process(img, detections);
	return detections.size();
}

#define rep_var(varname) std::cout<< #varname << "=" << varname <<std::endl

void computeHomography(const std::vector<TagDetection>& detections,
					   const int start_id, const int rows,
					   const int cols, const double black_length,
					   double H[9])
{
	april::tag::Homography33b homo_calc;
	const double tag_len=black_length*tagFamilies[0]->getTagRenderDimension()
		                 /(tagFamilies[0]->d+2*tagFamilies[0]->blackBorder);
	const double worldW=tag_len*cols, worldH=tag_len*rows;

	//rep_var(tagFamilies[0]->getTagRenderDimension());
	//rep_var(tagFamilies[0]->d);
	//rep_var(tagFamilies[0]->blackBorder);
	//rep_var(tag_len);
	//rep_var(worldW);
	//rep_var(worldH);

	double crns[4][2]={
		{-black_length/2,-black_length/2},
		{ black_length/2,-black_length/2},
		{ black_length/2, black_length/2},
		{-black_length/2, black_length/2}
	};

	for(int i=0; i<(int)detections.size(); ++i) {
		const TagDetection& dd=detections[i];
		if(dd.id<start_id || dd.id>=(start_id+rows*cols)) {
			flogli("[computeHomography info] ignore tag with id="<<dd.id);
			continue;
		}
		//int scale = tagFamilies[0]->getTagRenderDimension();
		//compute worldx and worldy of the tag center
		const int id=dd.id-start_id;
		const int r=id/cols;
		const int c=id-r*cols;
		double worldx=tag_len/2+c*tag_len;
		double worldy=tag_len/2+r*tag_len;
		worldx-=worldW/2;
		worldy=worldH/2-worldy;

		for(int k=0; k<4; ++k) {
			homo_calc.addCorrespondence(
				worldx+crns[k][0], worldy+crns[k][1],
				dd.p[k][0], dd.p[k][1]);
		}
	}

	homo_calc.compute();
	std::copy((double*)homo_calc.H[0], (double*)(homo_calc.H[0]+9), H);
}

void process(int argc, char **argv) {
	cv::Mat leftImg, rightImg;
	std::vector<TagDetection> leftTags, rightTags;
	if(getImageWithDetections(argv[1], leftImg, leftTags) <1) {
		flogle("[process error] no tag in left image: "<<argv[1]);
		return;
	}
	if(getImageWithDetections(argv[2], rightImg, rightTags) <1) {
		flogle("[process error] no tag in right image: "<<argv[2]);
		return;
	}

	//get parameters
	double Kl[9], Kr[9], Rrl[9], trl[3];
	int left_start_id=-1, left_rows=-1, left_cols=-1;
	double left_tag_black_length=-1;
	int right_start_id=-1, right_rows=-1, right_cols=-1;
	double right_tag_black_length=-1;
	{
		{std::stringstream ss;
		ss.str(GConfig::Instance().getRawString("Kl"));
		for(int i=0; i<9; ++i) ss>>Kl[i];}

		{std::stringstream ss;
		ss.str(GConfig::Instance().getRawString("Kr"));
		for(int i=0; i<9; ++i) ss>>Kr[i];}

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
#define report_arr(varname,len) std::cout<< #varname << "=" <<std::endl; \
	for(int i=0; i<len; ++i) std::cout<<varname[i]<<" "; std::cout<<std::endl
		report_arr(Kl,9);
		report_arr(Kr,9);
		report_arr(Rrl,9);
		report_arr(trl,3);
#undef report_arr
#define report_var(varname) std::cout<< #varname << "=" << varname <<std::endl
		report_var(left_start_id);
		report_var(left_rows);
		report_var(left_cols);
		report_var(left_tag_black_length);

		report_var(right_start_id);
		report_var(right_rows);
		report_var(right_cols);
		report_var(right_tag_black_length);
#undef report_var
	}

	//calculate homography matrix
	double leftH[9], rightH[9];
	computeHomography(leftTags, left_start_id, left_rows,
		              left_cols, left_tag_black_length, leftH);
	computeHomography(rightTags, right_start_id, right_rows,
		              right_cols, right_tag_black_length, rightH);

	const double left_tag_len=left_tag_black_length*tagFamilies[0]->getTagRenderDimension()
		                 /(tagFamilies[0]->d+2*tagFamilies[0]->blackBorder);
	const double right_tag_len=right_tag_black_length*tagFamilies[0]->getTagRenderDimension()
		                 /(tagFamilies[0]->d+2*tagFamilies[0]->blackBorder);

	//visualization
	{
		const double left_crns[4][2]={
			{-left_tag_len, -left_tag_len},
			{ left_tag_len, -left_tag_len},
			{ left_tag_len,  left_tag_len},
			{-left_tag_len,  left_tag_len}
		};
		const double right_crns[4][2]={
			{-right_tag_len, -right_tag_len},
			{ right_tag_len, -right_tag_len},
			{ right_tag_len,  right_tag_len},
			{-right_tag_len,  right_tag_len}
		};
		helper::drawHomography(leftImg, cv::Mat(3,3,CV_64FC1,leftH), left_crns);
		helper::drawHomography(rightImg, cv::Mat(3,3,CV_64FC1,rightH), right_crns);
		cv::namedWindow("left"); cv::imshow("left", leftImg);
		cv::namedWindow("right"); cv::imshow("right", rightImg);
	}

	//calibration transform
	double Rl[9],tl[3],Rr[9],tr[3];
	helper::RTfromKH(Kl, leftH, Rl, tl, true);
	helper::RTfromKH(Kr, rightH, Rr, tr, true);
	cv::Mat Tl=cv::Mat::eye(4,4,CV_64FC1);
	cv::Mat Tr=cv::Mat::eye(4,4,CV_64FC1);
	cv::Mat Trl=cv::Mat::eye(4,4,CV_64FC1);

	cv::Mat(3,3,CV_64FC1,Rl).copyTo(Tl(cv::Rect(0,0,3,3)));
	cv::Mat(3,1,CV_64FC1,tl).copyTo(Tl(cv::Rect(3,0,1,3)));

	cv::Mat(3,3,CV_64FC1,Rr).copyTo(Tr(cv::Rect(0,0,3,3)));
	cv::Mat(3,1,CV_64FC1,tr).copyTo(Tr(cv::Rect(3,0,1,3)));

	cv::Mat(3,3,CV_64FC1,Rrl).copyTo(Trl(cv::Rect(0,0,3,3)));
	cv::Mat(3,1,CV_64FC1,trl).copyTo(Trl(cv::Rect(3,0,1,3)));

	cv::Mat output = (Tl*Trl)*Tr.inv();

	rep_var(Tl);
	rep_var(Tr);
	rep_var(Trl);
	rep_var(output);
}

int main( int argc, char **argv )
{
	LogHelper::GLogControl::Instance().level = LogHelper::LOG_INFO;

	if(argc<4) {
		usage(argc,argv);
		return -1;
	}

	ConfigHelper::Config& cfg = GConfig::Instance();
	std::string cfgpath(argv[3]);
	if(!cfg.load(cfgpath)) {
		flogli("[main] no "<<cfgpath);
	} else {
		flogli("[main] loaded "<<cfgpath);
	}

	//// create tagFamily
	string tagid("0"); //default tag16h5
	if(argc>4) tagid = string(argv[4]);
	for(int i=0; i<(int)tagid.size(); ++i) {
		const char curchar[] = {tagid[i],'\0'};
		unsigned int curid = atoi(curchar);//atoi works on an array of char, not on a single char!!
		cv::Ptr<TagFamily> tagFamily = TagFamilyFactory::create(curid);
		if(tagFamily.empty()) {
			tagle<<"create TagFamily "<<curid<<" fail, skip!";
			continue;
		}
		tagFamilies.push_back(tagFamily);
		break;//in this app we only need one tagFamily, keep the first one
	}
	if(tagFamilies.size()<=0) {
		tagle<<"create TagFamily failed all! exit...";
		return -1;
	}

	detector = new TagDetector(tagFamilies);
	if(detector.empty()) {
		tagle<<"create TagDetector fail!";
		return -1;
	}

	process(argc,argv);

	cv::waitKey(-1);

	cout<<"[main] DONE...exit!"<<endl;
	return 0;
}
