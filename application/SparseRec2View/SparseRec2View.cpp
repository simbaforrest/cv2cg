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

/* SparseRec2View.cpp */

#include "SparseRec2View.h"
#include "Log.h"

#define CV_RED		Scalar(255,0,0)
#define CV_GREEN	Scalar(0,255,0)
#define CV_BLUE		Scalar(0,0,255)
#define CV_WHITE	Scalar(255,255,255)
#define CV_BLACK	Scalar(0,0,0)
#define CV_GRAY		Scalar(128,128,128)

SparseRec2View::SparseRec2View(
	string ipath1, //left image path
	string ipath2, //right image path
	double k[9], //calibration matrix
	double lamda_, //controls the reconstructed scene scale
	bool onlymatch //only perform match, no reconstruction
	)
{
	memcpy(K, k, sizeof(double)*9);
	lamda = lamda_;
	imgpath1 = ipath1;
	imgpath2 = ipath2;

	dir = helper::getFileDir(imgpath1);
	TagI("main dir = %s\n",dir.c_str());

	imgname1 = helper::getNameNoExtension(imgpath1);
	imgname2 = helper::getNameNoExtension(imgpath2);

	_onlymatch=onlymatch;

	detector = new cv::SurfFeatureDetector();
	TagI("detector = SURF\n");

	descriptor = new cv::SurfDescriptorExtractor();
	TagI("descriptor = SURF\n");

	matcher = new cv::FlannBasedMatcher();
	TagI("matcher = Flann\n");
}

SparseRec2View::~SparseRec2View()
{
	delete detector;
	detector = 0;
	delete descriptor;
	descriptor = 0;
	delete matcher;
	matcher = 0;
}

bool SparseRec2View::run()
{
	if( !detector || !descriptor || !matcher ) {
		TagE("No valid detector or descriptor or matcher\n");
		return false;
	}

	if( !loadImage() )	return false;
	if( !detect() )		return false;
	if( !match() )		return false;
	if( !fmatrix() )	return false;

	if(_onlymatch) return true;

	if( !estimateRelativePose() ) return false;
	return true;
}

bool SparseRec2View::loadImage()
{
	img1 = imread(imgpath1.c_str());
	if(img1.empty()) {
		TagE("can not open image at\n  %s\n",imgpath1.c_str());
		return false;
	}
	img2 = imread(imgpath2.c_str());
	if(img2.empty()) {
		TagE("can not open image at\n  %s\n",imgpath2.c_str());
		return false;
	}
	cvtColor(img1, igrey1, CV_RGB2GRAY);
	cvtColor(img2, igrey2, CV_RGB2GRAY);
	return true;
}

bool SparseRec2View::detect()
{
	double tt = (double)getTickCount();
	detector->detect(igrey1, key1);
	detector->detect(igrey2, key2);
	TagI("key number for image 1 = %d\n",(int)key1.size());
	TagI("key number for image 2 = %d\n",(int)key2.size());
	tt = (double)getTickCount() - tt;
	TagI("detect time = %lf ms\n", tt/getTickFrequency()*1000.0);

	//draw
	for(int i=0; i<(int)key1.size(); ++i) {
		int radius = cvRound(key1[i].size*1.2/9.*2);
		circle(img1, key1[i].pt, 2, CV_BLUE, 1);
		circle(img1, key1[i].pt, radius, CV_RED, 1);
	}
	for(int i=0; i<(int)key2.size(); ++i) {
		int radius = cvRound(key2[i].size*1.2/9.*2);
		circle(img2, key2[i].pt, 2, CV_BLUE, 1);
		circle(img2, key2[i].pt, radius, CV_RED, 1);
	}

	//combined
	int cw = img1.cols + img2.cols;
	int ch = std::max(img1.rows, img2.rows);
	combined.create(ch, cw, img1.type());

	IplImage imgl = IplImage(combined);
	cvSetImageROI(&imgl, cvRect(0,0,img1.cols,img1.rows));
	cvCopy(&IplImage(img1), &imgl);

	IplImage imgr = IplImage(combined);
	cvSetImageROI(&imgr, cvRect(img1.cols-1,0,img2.cols,img2.rows));
	cvCopy(&IplImage(img2), &imgr);

	return true;
}

bool SparseRec2View::match()
{
	//describe
	Mat des1, des2;
	double tt = (double)getTickCount();
	descriptor->compute(igrey1, key1, des1);
	descriptor->compute(igrey2, key2, des2);
	tt = (double)getTickCount() - tt;
	TagI("describe time = %lf ms\n", tt/getTickFrequency()*1000.0);

	//match
	tt = (double)getTickCount();
	matcher->clear();
	matcher->match(des2, des1, matches);//img1 as train image, img2 as query image
	tt = (double)getTickCount() - tt;
	TagI("match time = %lf ms\n", tt/getTickFrequency()*1000.0);

	p1.clear(); p2.clear();
	p1.reserve(matches.size());
	p2.reserve(matches.size());
	for(size_t i=0; i<matches.size(); ++i) {
		const DMatch& m = matches[i];
		p1.push_back(key1[m.trainIdx].pt);
		p2.push_back(key2[m.queryIdx].pt);
	}

	return true;
}

bool SparseRec2View::fmatrix()
{
	//fmatrix
	double tt = (double)getTickCount();
	Mat fmat = cv::findFundamentalMat(Mat(p1), Mat(p2), inliers);
	tt = (double)getTickCount() - tt;
	TagI("ransac time = %lf ms\n", tt/getTickFrequency()*1000.0);

	std::copy(fmat.begin<double>(), fmat.end<double>(), F);

	//draw
	int cnt=0;
	for(int i=0; i<(int)p1.size(); ++i) {
		if(!inliers[i]) continue;
		++cnt;
		line(combined, p1[i], Point(p2[i].x+img1.cols,p2[i].y), CV_WHITE);
	}
	TagI("number of inliers = %d\n",cnt);
	inliersNum = cnt;

	return true;
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

	CreateCvMatHead(_U,3,3,U);
	CreateCvMatHead(_S,3,3,S);
	CreateCvMatHead(_VT,3,3,VT);
	CreateCvMatHead(_K,3,3,K);
	CreateCvMatHead(_E,3,3,E);
	CreateCvMatHead(_F,3,3,F);
	CreateCvMatHead(_tmp9,3,3,tmp9);
	CreateCvMatHead(_u3,3,1,u3);
	CreateCvMatHead(_Ra,3,3,Ra);
	CreateCvMatHead(_Rb,3,3,Rb);
	CreateCvMatHead(_W,3,3,W);
	CreateCvMatHead(_WT,3,3,WT);

	//get essential matrix
	cvGEMM(&_K,&_F,1,0,0,&_tmp9, CV_GEMM_A_T);
	cvMatMul(&_tmp9, &_K, &_E); //E = K2' * F * K1; % K2==K1 currently

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

	double P1[12] = {
		K[0],K[1],K[2],0,
		K[3],K[4],K[5],0,
		K[6],K[7],K[8],0
	};
	double P2[12];
	CreateCvMatHead(_P2,3,4,P2);

	// test Ra
	P2[0]=Ra[0]; P2[1]=Ra[1]; P2[2]=Ra[2]; P2[3]=u3[0];
	P2[4]=Ra[3]; P2[5]=Ra[4]; P2[6]=Ra[5]; P2[7]=u3[1];
	P2[8]=Ra[6]; P2[9]=Ra[7]; P2[10]=Ra[8]; P2[11]=u3[2];
	cvMatMul(&_K,&_P2,&_P2); //P2 = K*[Ra,u3];

	int c1_pos = 0, c1_neg = 0;
	int c2_pos = 0, c2_neg = 0;
	for(int i=0; i<(int)p1.size(); ++i) {
		if(!inliers[i]) continue;

		double X[3];
		helper::triangulate(p1[i].x, p1[i].y,
			p2[i].x, p2[i].y, P1, P2, X);
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
		for(int i=0; i<(int)p1.size(); ++i) {
			if(!inliers[i]) continue;

			double X[3];
			helper::triangulate(p1[i].x, p1[i].y,
				p2[i].x, p2[i].y, P1, P2, X);
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
			memcpy(R, Rb, 9 * sizeof(double));
			t[0] = u3[0]; t[1] = u3[1]; t[2] = u3[2];
		} else if (c1_pos < c1_neg && c2_pos < c2_neg) {
			memcpy(R, Rb, 9 * sizeof(double));
			t[0] = -u3[0]; t[1] = -u3[1]; t[2] = -u3[2];
		} else {
			TagE("no case was found!\n");
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
	if(Log::level>=Log::LOG_DEBUG) helper::print(3,4,P2,"Final P2");
	double maxu1=-DBL_MAX,minu1=DBL_MAX,maxv1=-DBL_MAX,minv1=DBL_MAX;
	double maxu2=-DBL_MAX,minu2=DBL_MAX,maxv2=-DBL_MAX,minv2=DBL_MAX;
	for(int i=0; i<(int)p1.size(); ++i) {
		if(!inliers[i]) continue;

		double X[3];
		helper::triangulate(p1[i].x, p1[i].y,
			p2[i].x, p2[i].y, P1, P2, X);
		if(X[2]>0)
			results.push_back(cv::Point3d(X[0],X[1],X[2]));

		double u,v;
		helper::project(P1,X[0],X[1],X[2], u,v);
		double du1=u-p1[i].x, dv1=v-p1[i].y;
		LogD(">P1\t%lf\t%lf\n",du1,dv1);
		helper::project(P2,X[0],X[1],X[2], u,v);
		double du2=u-p2[i].x, dv2=v-p2[i].y;
		LogD(">P2\t%lf\t%lf\n",du2,dv2);
		maxu1 = std::max(maxu1,du1); minu1 = std::min(minu1,du1);
		maxv1 = std::max(maxv1,dv1); minv1 = std::min(minv1,dv1);
		maxu2 = std::max(maxu2,du2); minu2 = std::min(minu2,du2);
		maxv2 = std::max(maxv2,dv2); minv2 = std::min(minv2,dv2);
	}
	TagI("reproject error for img1 = (%g ~ %g, %g ~ %g)\n", minu1, maxu1, minv1, maxv1);
	TagI("reproject error for img2 = (%g ~ %g, %g ~ %g)\n", minu2, maxu2, minv2, maxv2);

	return true;
}

bool SparseRec2View::save()
{
	if(!_onlymatch) {
		//save reconstructed points
		string path1(dir+imgname1+string("-")+imgname2+string(".X"));
		std::ofstream o1(path1.c_str());
		o1 << "VERTEX " << (int)results.size() << endl;
		for(int i=0; i<(int)results.size(); ++i) {
			o1<<results[i].x<<" "<<results[i].y<<" "<<results[i].z<<endl;
		}
		o1.close();
		TagI("save reconstructed points to\n  %s\n",path1.c_str());

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
		TagI("save camera 1's parameters to\n  %s\n",path7.c_str());

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
		TagI("save camera 2's parameters to\n  %s\n",path8.c_str());
	}

	double fontScale=0.5;
	CvPoint text_origin;
	//save matched point pairs
	string path2(dir+imgname1+string("-")+imgname2+string(".p1p2"));
	std::ofstream o2(path2.c_str());
	//out.setf(std::ios::scientific);
	int cnt = 0;
	for(int i=0; i<(int)p1.size(); ++i) {
		if(!inliers[i]) continue;

		o2 << p1[i].x <<" "<< p1[i].y <<" "<< p2[i].x <<" "<< p2[i].y <<endl;

		char tmp[256];
		sprintf(tmp, "%d", cnt++);
		text_origin.x = (int)p1[i].x+5;
		text_origin.y = (int)p1[i].y-5;
		putText(img1, tmp, text_origin, CV_FONT_HERSHEY_PLAIN, fontScale, CV_BLACK);
		text_origin.x = (int)p2[i].x+5;
		text_origin.y = (int)p2[i].y-5;
		putText(img2, tmp, text_origin, CV_FONT_HERSHEY_PLAIN, fontScale, CV_BLACK);
	}
	o2.close();
	TagI("save matched point pairs to\n  %s\n",path2.c_str());

	//save images
	string path3(imgpath1+string("-detect.jpg"));
	string path4(imgpath2+string("-detect.jpg"));
	string path5(dir+imgname1+string("-")+imgname2+string(".jpg"));
	if(!img1.empty()) cv::imwrite(path3, img1);
	else {
		TagE("no valid image to save!\n");
		return false;
	}
	if(!img2.empty()) cv::imwrite(path4, img2);
	if(!combined.empty()) cv::imwrite(path5, combined);
	TagI("save surfed image 1 to\n  %s\n",path3.c_str());
	TagI("save surfed image 2 to\n  %s\n",path4.c_str());
	TagI("save combined image to\n  %s\n",path5.c_str());

	//save F
	string path6(dir+imgname1+string("-")+imgname2+string(".fmatrix"));
	std::ofstream o6(path6.c_str());
	o6 << helper::PrintMat<>(3,3,F);
	o6.close();
	TagI("save fundamental matrix to\n  %s\n",path6.c_str());

	return true;
}
