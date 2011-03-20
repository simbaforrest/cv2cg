/*
 *  Copyright (c) 2011  Chen Feng (cforrest (at) umich.edu)
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

/* MultiViewSparseRec.cpp */

#include "MultiViewSparseRec.h"
#include "Log.h"

MVSR::MVSR(void)
{
	detector = new cv::SurfFeatureDetector();
	TagI("detector = SURF\n");

	descriptor = new cv::SurfDescriptorExtractor();
	TagI("descriptor = SURF\n");

	matcher = new cv::FlannBasedMatcher();
	TagI("matcher = Flann\n");

	misMatchCnt = 0;
	matchEnhancedCnt = 0;
}

MVSR::~MVSR(void)
{
	delete detector;
	detector = 0;
	delete descriptor;
	descriptor = 0;
	delete matcher;
	matcher = 0;
}

bool MVSR::run(vector<string> imgnamelist, string outdir, string mainname)
{
	if( !detector || !descriptor || !matcher ) {
		TagE("No valid detector or descriptor or matcher\n");
		return false;
	}

	pictures.clear();
	clouds.clear();
	matcher->clear();
	misMatchCnt = 0;
	matchEnhancedCnt = 0;

	if(!loadimage(imgnamelist))	return false;
	if(!detect())	return false;
	if(!pairwise())	return false;
	if(!save(outdir, mainname))	return false;
	return true;
}

bool MVSR::loadimage(vector<string> imgnamelist)
{
	int nimg = (int)imgnamelist.size();
	pictures.resize(nimg);
	int im=0,in=0;
	for(; in<nimg; ++in) {
		string& name = imgnamelist[in];
		MVSRpicture& pic = pictures[im];
		pic.path = name;
		pic.img = imread(name);
		if(pic.img.empty()) {
			TagW("can't open %s\n", name.c_str());
			continue;
		} else {
			TagI("opened %s\n", name.c_str());
		}
		cvtColor(pic.img, pic.grey, CV_RGB2GRAY);
		++im;
	}
	pictures.resize(im);
	TagI("opened images/total images = %d/%d\n", im, nimg);
	return im==0?false:true;
}

bool MVSR::detect()
{
	double tt = (double)getTickCount();
	for(int i=0; i<(int)pictures.size(); ++i) {
		MVSRpicture& pic = pictures[i];
		detector->detect(pic.grey, pic.key);
		descriptor->compute(pic.grey, pic.key, pic.des);
		//init k2o, from image keypoint to object point
		pic.k2o.resize(pic.key.size());
		std::fill(pic.k2o.begin(), pic.k2o.end(), -1);
	}
	tt = (double)getTickCount() - tt;
	TagI("detect+describe time = %lf s\n", tt/getTickFrequency());
	return true;
}

bool MVSR::pairwise()
{
	double tt = (double)getTickCount();
	if(pictures.size()<=1) {
		TagE("need at least 2 images!");
		return false;
	}

	TagI("begin {\n");
	//init pairwiseFMat
	int N = (int)pictures.size();
	pairwiseFMat.resize( (N-2)*(N-1)/2+N-1 );
	//find matches for each pair of images
	for(int i=0; i<(int)pictures.size(); ++i) {
		for(int j=i+1; j<(int)pictures.size(); ++j) {
			match(i,j);//process a pair of image
		}
	}
	tt = (double)getTickCount() - tt;
	TagI("end} time = %lf s\n", tt/getTickFrequency());
	TagI("misMatchCnt/matchEnhancedCnt=%d/%d=%lf\n",
		misMatchCnt, matchEnhancedCnt,
		(double)misMatchCnt/matchEnhancedCnt);
	return true;
}

void MVSR::match(int imgi, int imgj)
{
	CV_Assert(imgi!=imgj);
	if(imgi>imgj) std::swap(imgi,imgj);//make sure imgi<imgj

	MVSRpicture& pici = pictures[imgi];
	MVSRpicture& picj = pictures[imgj];
	matcher->clear();//clear last time's
	vector<DMatch> matches;
	//pici as query, picj as train
	matcher->match(pici.des, picj.des, matches);
	TagI("pic%d.key.size=%d,pic%d.key.size=%d,matches.size=%d\n",
			imgi, (int)pici.key.size(),
			imgj, (int)picj.key.size(),
			(int)matches.size());

	//calc F matrix and remove match outliers
	vector<Point2f> pi, pj;//matched image points
	vector<uchar> inliers;//inliers in pi-pj for fmatrix, 0 means outlier
	pi.reserve(matches.size());
	pj.reserve(matches.size());
	for(int i=0; i<(int)matches.size(); ++i) {
		const DMatch& m = matches[i];
		pj.push_back(picj.key[m.trainIdx].pt);
		pi.push_back(pici.key[m.queryIdx].pt);
	}
	Mat& fmat = getFMat(imgi,imgj);
	fmat = cv::findFundamentalMat(Mat(pi), Mat(pj), inliers);

	//parse matches
	for(int k=0; k<(int)matches.size(); ++k) {
		if(!inliers[k]) continue;
		const DMatch& m = matches[k];
		TagI("addpair: (img%d, key%d)<->(img%d, key%d)\n",
				imgi, m.queryIdx, imgj, m.trainIdx);
		addmatchpair(imgi, m.queryIdx, imgj, m.trainIdx, m.distance);
	}
}

//add linkage between object points and image's keypoint
void MVSR::linkimgobj(int objIdx, int imgi, int keyi, float dist)
{
	CV_Assert(0<=objIdx && objIdx<(int)clouds.size());

	MVSRobjpoint& obj = clouds[objIdx];
	int existkey = -1;
	for(int i=0; i<(int)obj.obs.size(); ++i) {
		if(obj.obs[i].imgi==imgi && obj.obs[i].keyj!=-1) {
			if(obj.obs[i].dist>dist) { //modify old
				existkey = obj.obs[i].keyj;
				pictures[imgi].k2o[keyi] = objIdx;
				pictures[imgi].k2o[existkey] = -1;
				obj.obs[i].keyj = keyi;
				obj.obs[i].dist = dist;
				TagI("modify obj%d-<img%d,key%d> to key%d\n",
						objIdx,imgi,existkey,keyi);
			} else {
				TagI("match no strong enough, ignore...\n");
			}
			return;
		}
	}
	// old not found, adding new
	obj.observedAt(imgi, keyi, dist);
	pictures[imgi].k2o[keyi] = objIdx;
}

void MVSR::addmatchpair(int imgi, int keyi, int imgj, int keyj, float dist)
{
	CV_Assert(0<=imgi && imgi<(int)pictures.size());
	CV_Assert(0<=imgj && imgj<(int)pictures.size());
	CV_Assert(0<=keyi && keyi<(int)pictures[imgi].k2o.size());
	CV_Assert(0<=keyj && keyj<(int)pictures[imgj].k2o.size());

	MVSRpicture& pici = pictures[imgi];
	MVSRpicture& picj = pictures[imgj];
	int oiIdx = pici.k2o[keyi];
	int ojIdx = picj.k2o[keyj];
	if( oiIdx==-1 && ojIdx==-1 ) {
		//need a new object point
		clouds.push_back( MVSRobjpoint() );
		linkimgobj(clouds.size()-1, imgi, keyi, dist);
		linkimgobj(clouds.size()-1, imgj, keyj, dist);
	} else if( oiIdx==-1 && ojIdx!=-1 ) {
		linkimgobj(ojIdx, imgi, keyi, dist);
	} else if( ojIdx==-1 && oiIdx!=-1 ) {
		linkimgobj(oiIdx, imgj, keyj, dist);
	} else {
		if(oiIdx!=ojIdx) {
			//temporary ignor, TODO : fix mismatch by dist
			LogE("%d : mismatch may exist: "
					"(img%d, key%d)<->(img%d, key%d), ignore!\n",
				++misMatchCnt, imgi, keyi, imgj, keyj);
		} else {
			enhanceLink(oiIdx, dist);
			LogI("%d : (img%d, key%d)<->(img%d, key%d) enhanced!\n",
				++matchEnhancedCnt, imgi, keyi, imgj, keyj);
		}
	}
}

void MVSR::enhanceLink(int objIdx, int dist) {
//	MVSRobjpoint& obj = clouds[objIdx];
//	for(int i=0; i<(int)obj.obs.size(); ++i) {
//
//	}
}

bool MVSR::save(string outdir, string mainname)
{
	//tmp
	std::string dumppicname = outdir + mainname + std::string(".pic");
	std::ofstream dumppic(dumppicname.c_str());
	for(int i=0; i<(int)pictures.size(); ++i) {
		dumppic << pictures[i] << std::endl;
	}
	dumppic.close();

	std::string dumpobjname = outdir + mainname + std::string(".xyz");
	std::ofstream dumpobj(dumpobjname.c_str());
	for(int i=0; i<(int)clouds.size(); ++i) {
		dumpobj << clouds[i] << std::endl;
	}
	dumpobj.close();

	std::string dumpfmatname = outdir + mainname + std::string(".fmatrices");
	std::ofstream dumpfmat(dumpfmatname.c_str());
	for(int i=0; i<(int)pictures.size(); ++i) {
		for(int j=i+1; j<(int)pictures.size(); ++j) {
			const Mat& fmat = getFMat(i,j);
			dumpfmat << "F matrix between image ("
				<< i << "," << j << ")=" << std::endl;
			dumpfmat << fmat << std::endl;
		}
	}
	dumpfmat.close();
	return false;
}
