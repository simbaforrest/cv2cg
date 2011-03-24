#pragma once
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

/* MultiViewSparseRec.h */

//standard include
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "OpenCVHelper.h"

using namespace std;
using namespace cv;

struct MVSRpicture {
    string path; //path of the image
	Mat img,grey;
	vector<KeyPoint> key;//keypoints to be reconstructed
	Mat des;//descriptors for keypoints

	double K[9];
	double R[9],t[3];

	//map to object point idx,
	//key[i]<->the k2o[i]-th object point
	vector<int> k2o; // k2o.size() should equal to key.size()

	MVSRpicture() {
		helper::zeros(3,3,K);
		helper::zeros(3,3,R);
		helper::zeros(3,1,t);
	}

	//output format:
	//path
	//n pt1 map1 pt2 map2 ... ptn mapn
	friend inline std::ostream& operator<<(
		std::ostream& o,
		MVSRpicture const & pic ) {
		o << pic.path << std::endl;
		o<<">>K\n"<<helper::PrintMat<std::ios::fixed>(3,3,pic.K);
		o<<">>R\n"<<helper::PrintMat<std::ios::fixed>(3,3,pic.R);
		o<<">>T\n"<<helper::PrintMat<std::ios::fixed>(1,3,pic.t);
		o << (int) pic.k2o.size() << ":";
		for(int i=0; i<(int)pic.k2o.size(); ++i) {
			o << " <" << pic.key[i].pt <<
				"-" << pic.k2o[i] << ">";
		}
		return o;
	}
};

struct MVSRobjpoint {
	//<i,j> means this obj point is observed in jth keypoint on picture i
	struct Record {
		int imgi, keyj;
		float dist;//confidence of mismatch, less is better
		Record(int i=-1, int j=-1, float d=DBL_MAX) {imgi=i; keyj=j; dist=d;}
	};
	vector<Record> obs;
	Point3d pos;
	bool flag;//true if reconstructed, otherwise false

	MVSRobjpoint() {flag=false;}
	MVSRobjpoint(double x, double y, double z) : pos(x,y,z)
	{flag=true;}
	inline void observedAt(int imgi, int keyj, float dist) {
		obs.push_back(Record(imgi,keyj,dist));
	}

	//output format
	//pos flag
	//n img1 key1 cfd1 img1 key2 cfd1 ... imgx keyx cfdx
	friend inline std::ostream& operator<<(
		std::ostream& o,
		MVSRobjpoint const & obj ) {
		o << obj.pos << " " << obj.flag << std::endl;
		o << (int)obj.obs.size() << ":";
		for(int i=0; i<(int)obj.obs.size(); ++i) {
			o << " <" << obj.obs[i].imgi <<
				"-" << obj.obs[i].keyj << ">:"
				<< obj.obs[i].dist;
		}
		return o;
	}
};

class MVSR //MultiViewSparseRec
{
private:
	FeatureDetector* detector;
	DescriptorExtractor* descriptor;
	DescriptorMatcher* matcher;

	vector<MVSRpicture> pictures;
	vector<MVSRobjpoint> clouds;

	//Info for img pair <i,j> (i<j) is at
	//i*(N-1)+j-1-i*(i+1)/2, where N=pictures.size()
	//e.g. for N=5, img0,1,2,3,4
	//(0, 1) : 0 | (0, 2) : 1 | (0, 3) : 2 | (0, 4) : 3 |
	//(1, 2) : 4 | (1, 3) : 5 | (1, 4) : 6 |
	//(2, 3) : 7 | (2, 4) : 8 |
	//(3, 4) : 9 |
	struct PairwiseInfo {
		int imgi, imgj; //imgi < imgj
		Mat Fij; // fundamental matrix s.t. xj'*Fij*xi ~ 0
		float Frms; // root-mean-square error of xj'*Fij*xi
		vector<DMatch> matches;
		vector<uchar> inliers;//inliers in matches, 0 means outlier
		double Rj[9]; //rotation of j w.r.t. i, i.e. Ri=I
		double tj[3]; //translation of j w.r.t. i, i.e. ti = [0,0,0]

		PairwiseInfo() {imgi=imgj=-1; Frms=0;}
		friend inline std::ostream& operator<<(
			std::ostream& o, PairwiseInfo const & info) {
			o<<"<"<<info.imgi<<","<<info.imgj<<">"<<std::endl;
			o<<"Frms="<<info.Frms<<std::endl;
			o<<"matches.size="<<(int)info.matches.size()<<std::endl;
			o<<"inliers.size="<<(int)info.inliers.size()<<std::endl;
			o<<info.Fij<<std::endl;
			o<<">>R"<<info.imgj<<"\n"
				<<helper::PrintMat<>(3,3,info.Rj);
			o<<">>t"<<info.imgj<<"\n"
				<<helper::PrintMat<>(1,3,info.tj);
			return o;
		}
	};
	vector<PairwiseInfo> pwinfo;
	inline PairwiseInfo& getPairwiseInfo(int imgi, int imgj) {
		CV_Assert(imgi!=imgj);
		int N = (int)pictures.size();
		int i,j;
		imgi<imgj?(i=imgi,j=imgj):(i=imgj,j=imgi);
		return pwinfo[i*(N-1)+j-1-i*(i+1)/2];
	}

	//statistics
	int matchEnhancedCnt;
	int misMatchCnt;
public:
	MVSR(void);
	~MVSR(void);

	bool run(vector<string>& pathlist, vector< vector<double> >& caliblist,
			string outdir, string mainname);
private:
	bool loadimage(vector<string>& pathlist,
			vector< vector<double> >& caliblist);
	bool detect();
	bool pairwise();//pairwise matching and F matrix
	bool initbest();//find best pair and init reconstruction
	bool save(string outdir, string mainname);

	void match(int imgi, int imgj);
	//add match pair for
	//(keypoint[keyi] in img[imgi])<->(keypoint[keyj] in img[imgj])
	//with match distance dist indicating the mismatch confidence
	void addmatchpair(int imgi, int keyi, int imgj, int keyj, float dist);
	//add linkage between object points and image's keypoint
	void linkimgobj(int objIdx, int imgi, int keyi, float dist);
	//match loop will enhance the whole by their average dist
	void enhanceLink(int objIdx, int dist);
	// estimate pairwise relative pose
	void estimateRelativePose(int imgi, int imgj);
};
