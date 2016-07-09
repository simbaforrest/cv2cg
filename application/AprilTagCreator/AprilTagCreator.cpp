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

void usage( int argc, char **argv ) {
	cout<< "[usage] " <<argv[0]<<" <output dir> [TagFamily ID] [Tag Scale] [start_id,rows,cols,fname]"<<endl;
	cout<< "Supported TagFamily ID List:\n";
	for(int i=0; i<(int)TagFamilyFactory::TAGTOTAL; ++i) {
		cout<<"\t"<<april::tag::TagFamilyFactory_SUPPORT_NAME[i]<<" id="<<i<<endl;
	}
	cout<<"default ID:    0"<<endl;
	cout<<"default Scale: 1.0"<<endl;
}

bool write_calib_pattern_cfg(
	const TagFamily &tagFamily,
	const std::string& filepath, const int start_id,
	const int rows, const int cols, const int scale)
{
	const int dim = tagFamily.getTagRenderDimension()*scale;
	std::vector<float> tagCenters;
	std::vector<std::string> markerNames;

	for(int r=0,id=start_id; r<rows; ++r) {
		for(int c=0; c<cols; ++c,++id) {
			tagCenters.push_back((float)(c*dim+dim*0.5)); //x
			tagCenters.push_back((float)(r*dim+dim*0.5)); //y
			tagCenters.push_back(0); //z

			markerNames.push_back(tagFamily.familyName()+cv::format(".%d", id));
		}
	}

	std::ofstream fout((filepath+".cfg").c_str());
	if(!fout.is_open()) {
		return false;
	}

	fout<<"CalibRig={"<<std::endl;
	fout<<"\tmode=2d\n"
		  "\tmarkerNames=["<<markerNames[0];
	for(size_t i=1; i<markerNames.size(); ++i) {
		fout<<", "<<markerNames[i];
	}
	fout<<"];\n"
		  "\ttagCenters=["<<std::endl;
	for(size_t i=0; i<markerNames.size(); ++i) {
		fout<<"\t"<<tagCenters[i*3+0]<<" "<<tagCenters[i*3+1]<<" "<<tagCenters[i*3+2]<<std::endl;
	}
	fout<<"\t];\n}"<<std::endl;

	return true;
}

int main( int argc, char **argv )
{
	LogHelper::GetOrSetLogLevel(LogHelper::LOG_INFO);
	if(argc<2) {
		usage(argc,argv);
		return -1;
	}

	//// create tagFamily
	int tagid = 0; //default tag16h5
	if(argc>2) tagid = atoi(argv[2]);
	cv::Ptr<TagFamily> tagFamily = TagFamilyFactory::create(tagid);
	if(tagFamily.empty()) {
		tagle("create TagFamily fail!");
		return -1;
	}

	string dir(argv[1]);
	double tagscale = 1.0;
	if(argc>3) tagscale = atof(argv[3]);
	helper::legalDir(dir);
	if(argc<=4) {
		tagFamily->writeAllImages(dir,(int)tagscale);
		tagFamily->writeAllImagesMosaic(dir+"mosaic.png");
		tagFamily->writeAllImagesPostScript(dir+"all.ps");
	} else {
		std::vector<string> pars=helper::split(argv[4], ',');
		tagFamily->writeImagesMosaic(
			dir+pars[3], atoi(pars[0].c_str()),
			atoi(pars[1].c_str()), atoi(pars[2].c_str()), (int)tagscale);
		write_calib_pattern_cfg(*tagFamily,
			dir+pars[3], atoi(pars[0].c_str()),
			atoi(pars[1].c_str()), atoi(pars[2].c_str()), (int)tagscale);
	}

	std::cout<<"DONE!"<<std::endl;
	return 0;
}
