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

#include <algorithm>

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

bool MVSR::run(vector<string>& pathlist, vector< vector<double> >& caliblist,
			string outdir, string mainname)
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
	pwinfo.clear();

	if(!loadimage(pathlist, caliblist))	return false;
	if(!detect())	return false;
	if(!pairwise())	return false;
	if(!initbest()) return false;
	if(!save(outdir, mainname))	return false;
	return true;
}

bool MVSR::loadimage(vector<string>& pathlist,
	vector< vector<double> >& caliblist)
{
	int nimg = (int)pathlist.size();
	pictures.resize(nimg);
	int im=0,in=0;
	for(; in<nimg; ++in) {
		string& name = pathlist[in];
		vector<double>& calib = caliblist[in];
		MVSRpicture& pic = pictures[im];
		std::copy(calib.begin(), calib.end(), pic.K);
		pic.path = name;
		pic.img = imread(name);
		if(pic.img.empty()) {
			TagI("can't open %s\n", name.c_str());
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
	//init pairwiseInfo
	int N = (int)pictures.size();
	this->pwinfo.resize( (N-2)*(N-1)/2+N-1 );

	//find matches for each pair of images
	for(int i=0; i<(int)pictures.size(); ++i) {
		for(int j=i+1; j<(int)pictures.size(); ++j) {
			match(i,j);//process a pair of image
			estimateRelativePose(i,j);
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

	PairwiseInfo& ijinfo = this->getPairwiseInfo(imgi,imgj);
	ijinfo.imgi = imgi;
	ijinfo.imgj = imgj;

	MVSRpicture& pici = pictures[imgi];
	MVSRpicture& picj = pictures[imgj];
	matcher->clear();//clear last time's

	//pici as query, picj as train
	matcher->match(pici.des, picj.des, ijinfo.matches);
	TagD("pic%d.key.size=%d,pic%d.key.size=%d,matches.size=%d\n",
			imgi, (int)pici.key.size(),
			imgj, (int)picj.key.size(),
			(int)ijinfo.matches.size());

	//calc F matrix and remove match outliers
	vector<Point2f> pi, pj;//matched image points
	pi.reserve(ijinfo.matches.size());
	pj.reserve(ijinfo.matches.size());
	for(int i=0; i<(int)ijinfo.matches.size(); ++i) {
		const DMatch& m = ijinfo.matches[i];
		pj.push_back(picj.key[m.trainIdx].pt);
		pi.push_back(pici.key[m.queryIdx].pt);
	}
	ijinfo.Fij = cv::findFundamentalMat(Mat(pi), Mat(pj), ijinfo.inliers);

	double F[9];
	std::copy(ijinfo.Fij.begin<double>(), ijinfo.Fij.end<double>(), F);

	//parse matches
	ijinfo.Frms = 0;
	for(int k=0; k<(int)ijinfo.matches.size(); ++k) {
		if(!ijinfo.inliers[k]) continue;

		double y[3] = {pi[k].x,pi[k].y,1};
		double x[3] = {pj[k].x,pj[k].y,1};
		double tmp = helper::mulxtAy(3,x,F,y); // pj'*F*pi ~ 0
		ijinfo.Frms += (float)tmp*tmp;

		const DMatch& m = ijinfo.matches[k];
		TagD("addpair: (img%d, key%d)<->(img%d, key%d)\n",
				imgi, m.queryIdx, imgj, m.trainIdx);
		addmatchpair(imgi, m.queryIdx, imgj, m.trainIdx, m.distance);
	}
	ijinfo.Frms /= (float)ijinfo.inliers.size();
	ijinfo.Frms = sqrt(ijinfo.Frms);
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
				TagD("modify obj%d-<img%d,key%d> to key%d\n",
						objIdx,imgi,existkey,keyi);
			} else {
				TagD("match no strong enough, ignore...\n");
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
			++misMatchCnt;
			LogD("%d : mismatch may exist: "
					"(img%d, key%d)<->(img%d, key%d), ignore!\n",
				misMatchCnt, imgi, keyi, imgj, keyj);
		} else {
			enhanceLink(oiIdx, dist);
			++matchEnhancedCnt;
			LogD("%d : (img%d, key%d)<->(img%d, key%d) enhanced!\n",
				matchEnhancedCnt, imgi, keyi, imgj, keyj);
		}
	}
}

void MVSR::enhanceLink(int objIdx, int dist) {
//	MVSRobjpoint& obj = clouds[objIdx];
//	for(int i=0; i<(int)obj.obs.size(); ++i) {
//
//	}
}

bool MVSR::initbest()
{
	float min1=DBL_MAX,min2=DBL_MAX;
	//TODO remove i2,j2
	int i1,i2,j1,j2;
	for(int i=0; i<(int)pictures.size(); ++i) {
		for(int j=i+1; j<(int)pictures.size(); ++j) {
			const PairwiseInfo& info = this->getPairwiseInfo(i,j);
			float rms = info.Frms;
			if(rms<=min1) {
				// min1 -> min2
				min2 = min1;
				i2 = i1;
				j2 = j1;
				// cur -> min1
				min1 = rms;
				i1 = i;
				j1 = j;
			} else if(rms<=min2) {
				// cur -> min2
				min2 = rms;
				i2 = i;
				j2 = j;
			}
		}
	}
	TagI("<%d,%d> has min Frms=%lf\n",i1,j1,min1);
	TagI("<%d,%d> has second min Frms=%lf\n",i2,j2,min2);

	// set R of pic i1 as I
	helper::identity(3,pictures[i1].R);
	// set R of other pic
	for(int i=0; i<i1; ++i) {
		PairwiseInfo& info = this->getPairwiseInfo(i,i1);
		helper::transpose(3,3,info.Rj,pictures[i].R);
	}
	for(int j=i1+1; j<(int)pictures.size(); ++j) {
		PairwiseInfo& info = this->getPairwiseInfo(i1,j);
		std::copy(info.Rj,info.Rj+9,pictures[j].R);
		std::copy(info.tj,info.tj+3,pictures[j].t);
	}

	//triangulate from best init pair
	PairwiseInfo& ijinfo = this->getPairwiseInfo(i1,j1);
	MVSRpicture& pici = pictures[i1];
	MVSRpicture& picj = pictures[j1];
	double Pi[12];
	helper::compose(pici.K,pici.R,pici.t,Pi);
	double Pj[12];
	helper::compose(picj.K,picj.R,picj.t,Pj);
	double maxu1=-DBL_MAX,minu1=DBL_MAX,maxv1=-DBL_MAX,minv1=DBL_MAX;
	double maxu2=-DBL_MAX,minu2=DBL_MAX,maxv2=-DBL_MAX,minv2=DBL_MAX;
	for(int k=0; k<(int)ijinfo.matches.size(); ++k) {
		if(!ijinfo.inliers[k]) continue;
		const DMatch& m = ijinfo.matches[k];
		Point2f& pj = picj.key[m.trainIdx].pt;
		Point2f& pi = pici.key[m.queryIdx].pt;

		double X[3];
		helper::triangulate(pi.x, pi.y,
			pj.x, pj.y, Pi, Pj, X);
		//if(X[2]>0)
		//	results.push_back(cv::Point3d(X[0],X[1],X[2]));

		double u,v;
		helper::project(Pi,X[0],X[1],X[2], u,v);
		double du1=u-pi.x, dv1=v-pi.y;
		LogD(">P1\t%lf\t%lf\n",du1,dv1);
		helper::project(Pj,X[0],X[1],X[2], u,v);
		double du2=u-pj.x, dv2=v-pj.y;
		LogD(">P2\t%lf\t%lf\n",du2,dv2);
		maxu1 = std::max(maxu1,du1); minu1 = std::min(minu1,du1);
		maxv1 = std::max(maxv1,dv1); minv1 = std::min(minv1,dv1);
		maxu2 = std::max(maxu2,du2); minu2 = std::min(minu2,du2);
		maxv2 = std::max(maxv2,dv2); minv2 = std::min(minv2,dv2);
	}
	TagI("reproject error for img1 = (%g ~ %g, %g ~ %g)\n",
		minu1, maxu1, minv1, maxv1);
	TagI("reproject error for img2 = (%g ~ %g, %g ~ %g)\n",
		minu2, maxu2, minv2, maxv2);

	return true;
}

void MVSR::estimateRelativePose(int imgi, int imgj)
{
	CV_Assert(imgi!=imgj);
	if(imgi>imgj) std::swap(imgi,imgj);//make sure imgi<imgj
	PairwiseInfo& ijinfo = this->getPairwiseInfo(imgi,imgj);
	MVSRpicture& pici = pictures[imgi];
	MVSRpicture& picj = pictures[imgj];

	double U[9], S[9], VT[9];
	double W[9] =
	{  0.0, -1.0, 0.0,
	1.0, 0.0, 0.0,
	0.0, 0.0, 1.0 };
	double WT[9] =
	{  0.0, 1.0, 0.0,
	-1.0,  0.0, 0.0,
	0.0,  0.0, 1.0 };
	double E[9];
	double tmp9[9];
	double u3[3], Ra[9], Rb[9];
	double* Ki = pici.K;
	double* Kj = picj.K;
	double F[9];
	std::copy(ijinfo.Fij.begin<double>(), ijinfo.Fij.end<double>(), F);

	CreateCvMatHead(_U,3,3,U);
	CreateCvMatHead(_S,3,3,S);
	CreateCvMatHead(_VT,3,3,VT);
	CreateCvMatHead(_Ki,3,3,Ki);
	CreateCvMatHead(_Kj,3,3,Kj);
	CreateCvMatHead(_E,3,3,E);
	CreateCvMatHead(_F,3,3,F);
	CreateCvMatHead(_tmp9,3,3,tmp9);
	CreateCvMatHead(_u3,3,1,u3);
	CreateCvMatHead(_Ra,3,3,Ra);
	CreateCvMatHead(_Rb,3,3,Rb);
	CreateCvMatHead(_W,3,3,W);
	CreateCvMatHead(_WT,3,3,WT);

	//get essential matrix
	cvGEMM(&_Kj,&_F,1,0,0,&_tmp9, CV_GEMM_A_T);
	cvMatMul(&_tmp9, &_Ki, &_E); //Eij = Kj' * Fij * Ki;

	cvSVD(&_E, &_S, &_U, &_VT, CV_SVD_V_T);

	/* Now find R and t */
	u3[0] = U[2];  u3[1] = U[5];  u3[2] = U[8]; //u3 = U*[0;0;1];

	//two possible R UDVT, UDTVT
	cvMatMul(&_U, &_W, &_tmp9);
	cvMatMul(&_tmp9, &_VT, &_Ra); //Ra = U*W*V';
	cvMatMul(&_U, &_WT, &_tmp9);
	cvMatMul(&_tmp9, &_VT, &_Rb); //Rb = U*W'*V';

	if( cvDet(&_Ra) <0 )
		cvScale(&_Ra, &_Ra, -1);
	if( cvDet(&_Rb) <0 )
		cvScale(&_Rb, &_Rb, -1);

	double Pi[12] = {
		Ki[0],Ki[1],Ki[2],0,
		Ki[3],Ki[4],Ki[5],0,
		Ki[6],Ki[7],Ki[8],0
	};
	double Pj[12];
	CreateCvMatHead(_Pj,3,4,Pj);

	// test Ra
	Pj[0]=Ra[0]; Pj[1]=Ra[1]; Pj[2]=Ra[2]; Pj[3]=u3[0];
	Pj[4]=Ra[3]; Pj[5]=Ra[4]; Pj[6]=Ra[5]; Pj[7]=u3[1];
	Pj[8]=Ra[6]; Pj[9]=Ra[7]; Pj[10]=Ra[8]; Pj[11]=u3[2];
	cvMatMul(&_Kj,&_Pj,&_Pj); //Pj = Kj*[Ra,u3];

	int c1_pos = 0, c1_neg = 0;
	int c2_pos = 0, c2_neg = 0;
	for(int k=0; k<(int)ijinfo.matches.size(); ++k) {
		if(!ijinfo.inliers[k]) continue;
		const DMatch& m = ijinfo.matches[k];
		Point2f& pj = picj.key[m.trainIdx].pt;
		Point2f& pi = pici.key[m.queryIdx].pt;

		double X[3];
		helper::triangulate(pi.x, pi.y,
			pj.x, pj.y, Pi, Pj, X);
		double X2[3]={X[0],X[1],X[2]};
		CreateCvMatHead(_X2,3,1,X2);
		cvMatMul(&_Ra, &_X2, &_X2);
		cvAdd(&_X2, &_u3, &_X2);

		if (X[2] > 0)	c1_pos++;
		else	c1_neg++;

		if (X2[2] > 0)	c2_pos++;
		else	c2_neg++;
	}
	//cout<<"Test Ra"<<endl;
	//cout<<"+c1="<<c1_pos<<"\t-c1="<<c1_neg<<endl;
	//cout<<"+c2="<<c2_pos<<"\t-c2="<<c2_neg<<endl;

	if (c1_pos > c1_neg && c2_pos > c2_neg) {
		std::copy(Ra,Ra+9,ijinfo.Rj);
		ijinfo.tj[0] = u3[0];
		ijinfo.tj[1] = u3[1];
		ijinfo.tj[2] = u3[2];
	} else if (c1_pos < c1_neg && c2_pos < c2_neg) {
		std::copy(Ra,Ra+9,ijinfo.Rj);
		ijinfo.tj[0] = -u3[0];
		ijinfo.tj[1] = -u3[1];
		ijinfo.tj[2] = -u3[2];
	} else {
		//test Rb
		Pj[0]=Rb[0]; Pj[1]=Rb[1]; Pj[2]=Rb[2]; Pj[3]=u3[0];
		Pj[4]=Rb[3]; Pj[5]=Rb[4]; Pj[6]=Rb[5]; Pj[7]=u3[1];
		Pj[8]=Rb[6]; Pj[9]=Rb[7]; Pj[10]=Rb[8]; Pj[11]=u3[2];
		cvMatMul(&_Kj,&_Pj,&_Pj); //Pj = Kj*[Rb,u3];
		c1_pos = c1_neg = c2_pos = c2_neg = 0;
		for(int k=0; k<(int)ijinfo.matches.size(); ++k) {
			if(!ijinfo.inliers[k]) continue;
			const DMatch& m = ijinfo.matches[k];
			Point2f& pj = picj.key[m.trainIdx].pt;
			Point2f& pi = pici.key[m.queryIdx].pt;

			double X[3];
			helper::triangulate(pi.x, pi.y,
				pj.x, pj.y, Pi, Pj, X);
			double X2[3]={X[0],X[1],X[2]};
			CreateCvMatHead(_X2,3,1,X2);
			cvMatMul(&_Rb, &_X2, &_X2);
			cvAdd(&_X2, &_u3, &_X2);

			if (X[2] > 0)	c1_pos++;
			else c1_neg++;

			if (X2[2] > 0) c2_pos++;
			else c2_neg++;
		}
		//cout<<"Test Rb"<<endl;
		//cout<<"+c1="<<c1_pos<<"\t-c1="<<c1_neg<<endl;
		//cout<<"+c2="<<c2_pos<<"\t-c2="<<c2_neg<<endl;

		if (c1_pos > c1_neg && c2_pos > c2_neg) {
			std::copy(Rb,Rb+9,ijinfo.Rj);
			ijinfo.tj[0] = u3[0];
			ijinfo.tj[1] = u3[1];
			ijinfo.tj[2] = u3[2];
		} else if (c1_pos < c1_neg && c2_pos < c2_neg) {
			std::copy(Rb,Rb+9,ijinfo.Rj);
			ijinfo.tj[0] = -u3[0];
			ijinfo.tj[1] = -u3[1];
			ijinfo.tj[2] = -u3[2];
		} else {
			TagE("no case was found!\n");
			return;
		}
	}
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
			const PairwiseInfo& info = this->getPairwiseInfo(i,j);
			dumpfmat << info << std::endl;
		}
	}
	dumpfmat.close();
	return true;
}
