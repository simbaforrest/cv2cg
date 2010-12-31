/* 
 *  Copyright (c) 2010  Chen Feng (cforrest (at) umich.edu)
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

/* SparseRec2View.cpp */

#include "SparseRec2View.h"
#include "Log.h"

static CvScalar colors[] = 
{
	{{0,0,255}},
	{{0,128,255}},
	{{0,255,255}},
	{{0,255,0}},
	{{255,128,0}},
	{{255,255,0}},
	{{255,0,0}},
	{{255,0,255}},
	{{255,255,255}}
};

SparseRec2View::SparseRec2View(string ipath1,
							 string ipath2, double k[9], double lamda_)
{
	memcpy(K, k, sizeof(double)*9);
	lamda = lamda_;
	imgpath1 = ipath1;
	imgpath2 = ipath2;
	
	dir = helper::getFileDir(imgpath1);
	LogI("main dir = ");	Log::i(dir.c_str());

	imgname1 = helper::getNameNoExtension(imgpath1);
	imgname2 = helper::getNameNoExtension(imgpath2);

	img1=img2=0;
	igrey1=igrey2=0;
	combined=0;
	key1=key2=0;
	des1=des2=0;
	storage=0;
}

#define SAFEREL(p) {if((p)) cvReleaseImage(&(p)); (p)=0;}
SparseRec2View::~SparseRec2View()
{
	SAFEREL(igrey1);
	SAFEREL(igrey2);
	SAFEREL(img1);
	SAFEREL(img2);
	SAFEREL(combined);
	if(key1)cvClearSeq(key1); key1=0;
	if(key2)cvClearSeq(key2); key2=0;
	if(des1)cvClearSeq(des1); des1=0;
	if(des2)cvClearSeq(des2); des2=0;
	if(storage) cvReleaseMemStorage(&storage); storage=0;
}

bool SparseRec2View::save()
{
	//save reconstructed points
	string path1(dir+imgname1+string("-")+imgname2+string(".X"));
	std::ofstream o1(path1.c_str());
	o1 << "VERTEX " << (int)result.size() << endl;
	for(int i=0; i<(int)result.size(); ++i) {
		o1<<result[i].x<<" "<<result[i].y<<" "<<result[i].z<<endl;
	}
	o1.close();
	LogI("save reconstructed points to"); Log::i(path1.c_str());

	CvFont font = cvFont( 0.5, 1 );
	CvPoint text_origin;
	//save matched point pairs
	string path2(dir+imgname1+string("-")+imgname2+string(".p1p2"));
	std::ofstream o2(path2.c_str());
	//out.setf(std::ios::scientific);
	int cnt = 0;
	for (int i = 0; i < correctPairsNum; ++i) {
		if(pairs[i*2] == -1) continue;

		int idx1 = pairs[i*2];
		int idx2 = pairs[i*2+1];

		CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( key1, idx1 );
		CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( key2, idx2 );

		o2 << r1->pt.x <<" "<< r1->pt.y <<" "<< r2->pt.x <<" "<< r2->pt.y <<endl;

		char tmp[100];
		sprintf(tmp, "%d", cnt++);
		text_origin.x = r1->pt.x+5;
		text_origin.y = r1->pt.y-5;
		cvPutText( img1, tmp, text_origin, &font, CV_RGB(0,0,0));
		text_origin.x = r2->pt.x+5;
		text_origin.y = r2->pt.y-5;
		cvPutText( img2, tmp, text_origin, &font, CV_RGB(0,0,0));
	}
	o2.close();
	LogI("save matched point pairs to"); Log::i(path2.c_str());

	//save images
	string path3(imgpath1+string("-surf.jpg"));
	string path4(imgpath2+string("-surf.jpg"));
	string path5(dir+imgname1+string("-")+imgname2+string(".jpg"));
	if(img1) cvSaveImage(path3.c_str(), img1);
	else {
		LogE("no valid image to save!");
		return false;
	}
	if(img2) cvSaveImage(path4.c_str(), img2);
	if(combined) cvSaveImage(path5.c_str(), combined);
	LogI("save surfed image 1 to"); Log::i(path3.c_str());
	LogI("save surfed image 2 to"); Log::i(path4.c_str());
	LogI("save combined image to"); Log::i(path5.c_str());

	//save F
	string path6(dir+imgname1+string("-")+imgname2+string(".fmatrix"));
	std::ofstream o6(path6.c_str());
	o6.setf(std::ios::scientific);
	for(int i=0; i<3; ++i) {
		for(int j=0; j<3; ++j) {
			o6 << F[i*3+j] << " ";
		}
		o6 << endl;
	}
	o6.close();
	LogI("save fundamental matrix to"); Log::i(path6.c_str());

	//save cam par
	string path7(imgpath1+string(".par"));
	std::ofstream o7(path7.c_str());
	o7.setf(std::ios::scientific);
	o7 << "K(alphaX alphaY u0 v0)=" <<endl;
	o7 << K[0] << " " << K[4] << " " << K[2] << " " << K[5] << endl;
	o7 << "R=" << endl;
	o7 << "1 0 0\n0 1 0\n0 0 1" <<endl;
	o7 << "T=" << endl;
	o7 << "0 0 0" << endl;
	o7.close();
	LogI("save camera 1's parameters to"); Log::i(path7.c_str());

	string path8(imgpath2+string(".par"));
	std::ofstream o8(path8.c_str());
	o8.setf(std::ios::scientific);
	o8 << "K(alphaX alphaY u0 v0)=" <<endl;
	o8 << K[0] <<" "<< K[4] <<" "<< K[2] <<" "<< K[5] << endl;
	o8 << "R=" << endl;
	for(int i=0; i<3; ++i) {
		for(int j=0; j<3; ++j) {
			o8 << R[i*3+j] << " ";
		}
		o8 << endl;
	}
	o8 << "T=" << endl;
	o8 << t[0] <<" "<< t[1] <<" "<< t[2] << endl;
	o8.close();
	LogI("save camera 2's parameters to"); Log::i(path8.c_str());

	return true;
}

bool SparseRec2View::run()
{
	if( !loadImage() ) {
		LogE("can not open image at");
		Log::i(imgpath1.c_str());
		Log::i("\tor");
		Log::i(imgpath2.c_str());
		return false;
	}
	if( !surf() ) {
		return false;
	}
	if( !match() ) {
		return false;
	}
	if( !estimateFmatrix() ) {
		return false;
	}
	if( !estimateRelativePose() ) {
		return false;
	}
	return true;
}

bool SparseRec2View::loadImage()
{
	img1 = cvLoadImage(imgpath1.c_str());
	if(!img1) return false;
	img2 = cvLoadImage(imgpath2.c_str());
	if(!img2) return false;
	igrey1 = cvCreateImage(cvGetSize(img1), 8, 1);
	igrey2 = cvCreateImage(cvGetSize(img2), 8, 1);
	cvCvtColor(img1, igrey1, CV_RGB2GRAY);
	cvCvtColor(img2, igrey2, CV_RGB2GRAY);
	CvSize newsize = cvSize(img1->width+img2->width, 
		max(img1->height, img2->height));
	combined = cvCreateImage(newsize, 8, 3);
	return true;
}

bool SparseRec2View::surf()
{
	storage = cvCreateMemStorage(0);
	CvSURFParams params = cvSURFParams(500, 1);

	double tt = (double)cvGetTickCount();
	cvExtractSURF( igrey1, 0, &key1, &des1, storage, params );
	cvExtractSURF( igrey2, 0, &key2, &des2, storage, params );
	LogI("key number for image 1 =");	LogA("\t%d\n",key1->total);
	LogI("key number for image 2 ="); LogA("\t%d\n",key2->total);
	tt = (double)cvGetTickCount() - tt;
	LogI("surf time = "); LogA("\t%lf ms\n",tt/(cvGetTickFrequency()*1000.));

	for(int i=0; i<key1->total; ++i) {
		CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( key1, i );
		CvPoint center;
		int radius;
		center.x = cvRound(r->pt.x);
		center.y = cvRound(r->pt.y);
		radius = cvRound(r->size*1.2/9.*2);
		cvCircle( img1, center, 2, colors[0], 1, 8, 0 );
	}
	for(int i=0; i<key2->total; ++i) {
		CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( key2, i );
		CvPoint center;
		int radius;
		center.x = cvRound(r->pt.x);
		center.y = cvRound(r->pt.y);
		radius = cvRound(r->size*1.2/9.*2);
		cvCircle( img2, center, 2, colors[0], 1, 8, 0 );
	}

	//draw correspondences lines
	cvSetImageROI(combined, cvRect(0,0,img1->width, img1->height));
	cvCopy(img1, combined);
	cvSetImageROI(combined, cvRect(img1->width,0, img2->width, img2->height));
	cvCopy(img2, combined);
	cvResetImageROI(combined);

	return true;
}

bool SparseRec2View::match()
{
	double tt = (double)cvGetTickCount();
	//---begin---
	int length = (int)(des1->elem_size/sizeof(float));

	cv::Mat m_image1(des1->total, length, CV_32F);
	cv::Mat m_image2(des2->total, length, CV_32F);

	// copy descriptors
	CvSeqReader reader1;
	float* ptr1 = m_image1.ptr<float>(0);
	cvStartReadSeq( des1, &reader1 );
	for(int i = 0; i < des1->total; ++i )
	{
		const float* descriptor = (const float*)reader1.ptr;
		CV_NEXT_SEQ_ELEM( reader1.seq->elem_size, reader1 );
		memcpy(ptr1, descriptor, length*sizeof(float));
		ptr1 += length;
	}
	CvSeqReader reader2;
	float* ptr2 = m_image2.ptr<float>(0);
	cvStartReadSeq( des2, &reader2 );
	for(int i = 0; i < des2->total; i++ )
	{
		const float* descriptor = (const float*)reader2.ptr;
		CV_NEXT_SEQ_ELEM( reader2.seq->elem_size, reader2 );
		memcpy(ptr2, descriptor, length*sizeof(float));
		ptr2 += length;
	}

	// find nearest neighbors using FLANN
	cv::Mat m_indices(des1->total, 2, CV_32S);
	cv::Mat m_dists(des1->total, 2, CV_32F);
	cv::flann::Index flann_index(m_image2, 
		cv::flann::KDTreeIndexParams(4) );  // using 4 randomized kdtrees
	flann_index.knnSearch(m_image1, m_indices, m_dists, 
		2, cv::flann::SearchParams(128) ); // maximum number of leafs checked

	int* indices_ptr = m_indices.ptr<int>(0);
	float* dists_ptr = m_dists.ptr<float>(0);
	for (int i=0;i<m_indices.rows;++i) {
		if (dists_ptr[2*i]<0.6*dists_ptr[2*i+1]) {
			pairs.push_back(i);
			pairs.push_back(indices_ptr[2*i]);
		}
	}
	//---end------
	tt = (double)cvGetTickCount() - tt;
	LogI("flann match time = "); LogA("\t%lf ms\n",tt/(cvGetTickFrequency()*1000.) );
	
	return true;
}

bool SparseRec2View::estimateFmatrix()
{
	//fmatrix
	int n = (int)pairs.size()/2;
	CvMat* points1;
	CvMat* points2;
	CvMat* status;
	CvMat _F = cvMat(3,3,CV_64FC1, F);
	CvMat* fundMatr = &_F;
	points1 = cvCreateMat(2,n,CV_32F);
	points2 = cvCreateMat(2,n,CV_32F);
	status = cvCreateMat(1,n,CV_8UC1);
	// initialize the points here ... */
	for( int i = 0; i < n; i++ )
	{
		CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( key1, pairs[i*2] );
		CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( key2, pairs[i*2+1] );
		cvSetReal2D(points1,0,i,r1->pt.x);
		cvSetReal2D(points1,1,i,r1->pt.y);
		cvSetReal2D(points2,0,i,r2->pt.x);
		cvSetReal2D(points2,1,i,r2->pt.y);
	}
	//see opencv manual for other options in computing the fundamental matrix
	int num = cvFindFundamentalMat(points1,points2,fundMatr,CV_FM_RANSAC,3,0.99,status);
	if( num == 1 )
		LogI("Fundamental matrix was foundn!");
	else	{
		LogE("Fundamental matrix was not foundn!");
		return false;
	}

	int cnt=0;
	for(int i = 0; i < (int)pairs.size(); i += 2 )
	{
		uchar correct=CV_MAT_ELEM(*status, uchar, 0, i/2);
		if(!correct) {
			pairs[i]=pairs[i+1]=-1;//label as false corresponds
			continue;
		}
		++cnt;
		CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( key1, pairs[i] );
		CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( key2, pairs[i+1] );
		cvLine( combined, cvPointFrom32f(r1->pt),
			cvPoint(cvRound(r2->pt.x+img1->width), cvRound(r2->pt.y)), colors[8] );
		//cvLine( img1, cvPointFrom32f(r1->pt), cvPointFrom32f(r2->pt), colors[3]);
		//cvLine( img2, cvPointFrom32f(r2->pt), cvPointFrom32f(r1->pt), colors[3]);
	}
	LogI("number of correct correspond points found by RANSAC = ");	LogA("\t%d\n",cnt);
	correctPairsNum = cnt;

	//memcpy(F, fundMatr->data.db, sizeof(double)*9);
	cvReleaseMat(&points1);
	cvReleaseMat(&points2);
	cvReleaseMat(&status);
	return true;
}

inline void Project(double const* P,
					double x, double y, double z,
					double& u, double& v)
{
	double w;
	u = P[0] * x + P[1] * y
		+ P[2] * z + P[3] * 1;
	v = P[4] * x + P[5] * y
		+ P[6] * z + P[7] * 1;
	w = P[8] * x + P[9] * y
		+ P[10] * z + P[11] * 1;
	if(fabs(w)<1e-10)
		u = v = 0; // ideal point
	else {
		u/=w;
		v/=w;
	}
}

bool SparseRec2View::estimateRelativePose()
{
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

#define CreateCVHead(h,r,c,p) CvMat h = cvMat(r,c,CV_64FC1, p)
	CreateCVHead(_U,3,3,U);
	CreateCVHead(_S,3,3,S);
	CreateCVHead(_VT,3,3,VT);
	CreateCVHead(_K,3,3,K);
	CreateCVHead(_E,3,3,E);
	CreateCVHead(_F,3,3,F);
	CreateCVHead(_tmp9,3,3,tmp9);
	CreateCVHead(_u3,3,1,u3);
	CreateCVHead(_Ra,3,3,Ra);
	CreateCVHead(_Rb,3,3,Rb);
	CreateCVHead(_W,3,3,W);
	CreateCVHead(_WT,3,3,WT);
	
	//get essential matrix
	cvGEMM(&_K,&_F,1,0,0,&_tmp9, CV_GEMM_A_T);
	cvMatMul(&_tmp9, &_K, &_E); //E = K2' * F * K1; % K2==K1 currently

	cvSVD(&_E, &_S, &_U, &_VT, CV_SVD_V_T);

	/* Now find R and t */
	u3[0] = U[2];  u3[1] = U[5];  u3[2] = U[8]; //u3 = U*[0;0;1];

	//two possible R UDVT, UDTVT
	//cvMatMul(&_U, &_S, &_tmp9);
	//cvMatMul(&_tmp9, &_VT, &_tmp9);
	//Print(3,3,tmp9,"USVT");
	cvMatMul(&_U, &_W, &_tmp9);
	cvMatMul(&_tmp9, &_VT, &_Ra); //Ra = U*W*V';
	cvMatMul(&_U, &_WT, &_tmp9);
	cvMatMul(&_tmp9, &_VT, &_Rb); //Rb = U*W'*V';

	if( cvDet(&_Ra) <0 )
		cvScale(&_Ra, &_Ra, -1);
	if( cvDet(&_Rb) <0 )
		cvScale(&_Rb, &_Rb, -1);

	double P1[12] = {
		K[0],K[1],K[2],0,
		K[3],K[4],K[5],0,
		K[6],K[7],K[8],0
	};
	double P2[12];
	CreateCVHead(_P2,3,4,P2);

	//Print(3,3,F,"F");
	//Print(3,3,E,"E");
	//Print(3,3,U,"U");
	//Print(3,3,S,"S");
	//Print(3,3,VT,"VT");
	//Print(3,1,u3,"tu");
	//Print(3,3,Ra,"Ra");
	//Print(3,3,Rb,"Rb");
	//Print(3,4,P1,"P1");

	int num_pts = correctPairsNum;
	// test Ra
	P2[0]=Ra[0]; P2[1]=Ra[1]; P2[2]=Ra[2]; P2[3]=u3[0];
	P2[4]=Ra[3]; P2[5]=Ra[4]; P2[6]=Ra[5]; P2[7]=u3[1];
	P2[8]=Ra[6]; P2[9]=Ra[7]; P2[10]=Ra[8]; P2[11]=u3[2];
	cvMatMul(&_K,&_P2,&_P2); //P2 = K*[Ra,u3];

	//Print(3,4,P2,"P2");

	int c1_pos = 0, c1_neg = 0;
	int c2_pos = 0, c2_neg = 0;
	for (int i = 0; i < num_pts; ++i) {
		if(pairs[i*2] == -1) continue;

		int idx1 = pairs[i*2];
		int idx2 = pairs[i*2+1];

		CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( key1, idx1 );
		CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( key2, idx2 );

		double X[3];
		Triangulate(r1->pt.x, r1->pt.y,
			r2->pt.x, r2->pt.y, P1, P2, X);
		double X2[3]={X[0],X[1],X[2]};
		CreateCVHead(_X2,3,1,X2);
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
		memcpy(R, Ra, 9 * sizeof(double));
		t[0] = u3[0]; t[1] = u3[1]; t[2] = u3[2];
	} else if (c1_pos < c1_neg && c2_pos < c2_neg) {
		memcpy(R, Ra, 9 * sizeof(double));
		t[0] = -u3[0]; t[1] = -u3[1]; t[2] = -u3[2];
	} else {
		//test Rb
		P2[0]=Rb[0]; P2[1]=Rb[1]; P2[2]=Rb[2]; P2[3]=u3[0];
		P2[4]=Rb[3]; P2[5]=Rb[4]; P2[6]=Rb[5]; P2[7]=u3[1];
		P2[8]=Rb[6]; P2[9]=Rb[7]; P2[10]=Rb[8]; P2[11]=u3[2];
		cvMatMul(&_K,&_P2,&_P2); //P2 = K2*[Rb,u3];
		c1_pos = c1_neg = c2_pos = c2_neg = 0;
		for (int i = 0; i < num_pts; ++i) {
			if(pairs[i*2] == -1) continue;

			int idx1 = pairs[i*2];
			int idx2 = pairs[i*2+1];

			CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( key1, idx1 );
			CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( key2, idx2 );

			double X[3];
			Triangulate(r1->pt.x, r1->pt.y,
				r2->pt.x, r2->pt.y, P1, P2, X);
			double X2[3]={X[0],X[1],X[2]};
			CreateCVHead(_X2,3,1,X2);
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
			memcpy(R, Rb, 9 * sizeof(double));
			t[0] = u3[0]; t[1] = u3[1]; t[2] = u3[2];
		} else if (c1_pos < c1_neg && c2_pos < c2_neg) {
			memcpy(R, Rb, 9 * sizeof(double));
			t[0] = -u3[0]; t[1] = -u3[1]; t[2] = -u3[2];
		} else {
			LogE("no case was found!");
			return false;
		}
	};

	//final triangulate
	double tulen = u3[0]*u3[0]+u3[1]*u3[1]+u3[2]*u3[2];
	tulen = sqrt(tulen);
	u3[0]*=lamda/tulen; u3[1]*=lamda/tulen; u3[2]*=lamda/tulen;

	P2[0]=R[0]; P2[1]=R[1]; P2[2]=R[2]; P2[3]=t[0];
	P2[4]=R[3]; P2[5]=R[4]; P2[6]=R[5]; P2[7]=t[1];
	P2[8]=R[6]; P2[9]=R[7]; P2[10]=R[8]; P2[11]=t[2];
	cvMatMul(&_K,&_P2,&_P2);
	helper::print(3,4,P2,"Final P2");
	for (int i = 0; i < num_pts; ++i) {
		if(pairs[i*2] == -1) continue;

		int idx1 = pairs[i*2];
		int idx2 = pairs[i*2+1];

		CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( key1, idx1 );
		CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( key2, idx2 );

		double X[3];
		Triangulate(r1->pt.x, r1->pt.y,
			r2->pt.x, r2->pt.y, P1, P2, X);
		if(X[2]>0)
			result.push_back(Pt3(X[0],X[1],X[2]));

		double u,v;
		Project(P1,X[0],X[1],X[2], u,v);
		cout<<">P1\t"<<u-r1->pt.x<<"\t"<<v-r1->pt.y<<endl;
		Project(P2,X[0],X[1],X[2], u,v);
		cout<<">P2\t"<<u-r2->pt.x<<"\t"<<v-r2->pt.y<<endl;
	}

	return true;
}

void Triangulate(const double x1, const double y1,
		const double x2, const double y2,
		const double P1[12], const double P2[12], double X[3])
{
	double A[12] = {
		P1[0]-x1*P1[8], P1[1]-x1*P1[9], P1[2]-x1*P1[10],
		P1[4]-y1*P1[8], P1[5]-y1*P1[9], P1[6]-y1*P1[10],
		P2[0]-x2*P2[8], P2[1]-x2*P2[9], P2[2]-x2*P2[10],
		P2[4]-y2*P2[8], P2[5]-y2*P2[9], P2[6]-y2*P2[10]
	};
	double b[4] = {
		x1*P1[11]-P1[3],
		y1*P1[11]-P1[7],
		x2*P2[11]-P2[3],
		y2*P2[11]-P2[7]
	};

	CreateCVHead(_A,4,3,A);
	CreateCVHead(_b,4,1,b);
	CreateCVHead(_X,3,1,X);
	cvSolve(&_A,&_b,&_X, CV_SVD);

	//double A[16] = {
	//	x1*P1[8]-P1[0], x1*P1[9]-P1[1], x1*P1[10]-P1[2], x1*P1[11]-P1[3],
	//	y1*P1[8]-P1[4], y1*P1[9]-P1[5], y1*P1[10]-P1[6], y1*P1[11]-P1[7],
	//	x2*P2[8]-P2[0], x2*P2[9]-P2[1], x2*P2[10]-P2[2], x2*P2[11]-P2[3],
	//	y2*P2[8]-P2[4], y2*P2[9]-P2[5], y2*P2[10]-P2[6], y2*P2[11]-P2[7]
	//};
	//double x[4] = {0};
	//double B[4] = {0};
	////double u[16],w[16],v[16];

	//CreateCVHead(_A,4,4,A);
	//CreateCVHead(_x,4,1,x);
	//CreateCVHead(_B,4,1,B);
	////CreateCVHead(_u,4,4,u);
	////CreateCVHead(_w,4,4,w);
	////CreateCVHead(_v,4,4,v);

	////cvSVD(&_A, &_w, &_u, &_v);
	//cvSolve(&_A, &_B, &_x, CV_SVD);
	//X[0] = x[0]/x[3];
	//X[1] = x[1]/x[3];
	//X[2] = x[2]/x[3];
}