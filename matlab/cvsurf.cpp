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

/*[keys,matches]=cvsurf(imnames, DetectorType)
 *INPUT
 *  imnames <cell, string, Kx1>: file paths for each image
 *  [DetectorType <int, 1x1>]: specify detector type, default is SURF
 *OUTPUT
 *  [keys <cell, Kx1>]: optional, detected keypoints for each image, <Nx3>
 *  [matches <cell, KxK>]: optional, matched idx, <Mx2>
 *       matches{i,j}(k,:)=[m,n] means image i is training image, image j is
 *       query image, and keys{i}(m,:) is matched with keys{j}(n,:)
 *AUTHOR
 *  Chen Feng <simbaforrest@gmail.com>
 */

//standard include
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <time.h>
#include <math.h>
//opencv include
#include "opencv2/opencv.hpp"
//matlab include
#include "mex.h"
#include "matrix.h"

using namespace std;
using namespace cv;

#define IDXMATLAB(m,n,M) ((n)*(M)+(m)) //M rows

string DetectorType[] = {
    string("FAST"), //0
    string("STAR"), //1
    string("SIFT"), //2
    string("SURF"), //3
    string("MSER"), //4
    string("GFTT"), //5
    string("HARRIS") //6
};

void mexFunction(int nlhs, mxArray* plhs[], 
                 int nrhs, const mxArray* prhs[]) {
    if(nrhs<1)
		mexErrMsgTxt("[cvsurf] not enough input parameters!");
    if(nlhs>2)
        mexErrMsgTxt("[cvsurf] too many output parameters!");
    if(!mxIsCell(prhs[0]))
        mexErrMsgTxt("[cvsurf] input must be a cell array!");
    
    unsigned int NIMG = mxGetM(prhs[0]) * mxGetN(prhs[0]);
    
    vector<string> name(NIMG);
    vector<Mat> img(NIMG);
    for(int i=0; i<NIMG; ++i) {
        const mxArray* cell = mxGetCell(prhs[0],i);
        cout<<"[cvsurf] parsing input cell"<<i<<endl;
        if (mxIsChar(cell)!=1)
            mexErrMsgTxt("[cvsurf] this input must be string.");
        if (mxGetM(cell)!=1)
            mexErrMsgTxt("[cvsurf] this input must be row vector.");
        mwSize buflen = (mxGetM(cell) * mxGetN(cell)) + 1;
        char *input_buf = mxArrayToString(cell);
        if(!input_buf)
            mexErrMsgTxt("Could not convert this input to string.");
        for(int j=0; j<buflen-1; ++j)
            name[i].push_back(*(input_buf+j));
        
        cout<<"[cvsurf] reading "<<name[i]<<"->";
        Mat imgtmp = imread(name[i]);
        cout<<"channels="<<imgtmp.channels()<<" rows="
                <<imgtmp.rows<<" cols="<<imgtmp.cols<<"->";
        if(imgtmp.channels()==3) {
            cvtColor(imgtmp, img[i], CV_RGB2GRAY);
            cout<<"read to gray->";
        }
        else {
            img[i] = imgtmp.clone();
            cout<<"read as gray->";
        }
        if(img[i].empty()) {
            mexErrMsgTxt("[cvsurf] opencv can not read this image!");
        }
        cout<<"read done."<<endl;
    }
    
    int dtype = 3;
    if(nrhs>=2) {
        dtype = *(int*)mxGetData(prhs[1]);
        dtype = max(0,dtype);
        dtype = min(6,dtype);
    }
    cout<<"[cvsurf] detector type = "<<DetectorType[dtype]
            <<" ("<<dtype<<")"<<endl;
	Ptr<FeatureDetector> detector;
	if(dtype==0)
		detector = new FastFeatureDetector(20);
	else
		detector = FeatureDetector::create(DetectorType[dtype]);
    Ptr<DescriptorExtractor> descriptor=DescriptorExtractor::create("SURF");
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
    
    if(nlhs>0) { //detection
        vector< vector<KeyPoint> > key(NIMG);
        vector<unsigned int> M(NIMG);
        double tt = (double)getTickCount();
        for(int i=0; i<NIMG; ++i) {
            detector->detect(img[i], key[i]);
            M[i] = (unsigned int)key[i].size();
            cout<<"[cvsurf] "<<M[i]
                    <<" keypoints found in image"<<i<<"."<<endl;
        }
        tt = (double)getTickCount() - tt;
        cout<<"[cvsurf] detector time = "
                <<tt/getTickFrequency()*1000.0<<" ms."<<endl;
        unsigned int N = 3;
        
        plhs[0] = mxCreateCellMatrix(NIMG,1);
        for(int i=0; i<NIMG; ++i) {
            mxArray* tmparr = mxCreateDoubleMatrix(M[i], N, mxREAL);
            double *MP = mxGetPr(tmparr);
            for(int m=0; MP && m<(int)M[i]; ++m) {
                MP[IDXMATLAB(m, 0, M[i])]=key[i][m].pt.x;
                MP[IDXMATLAB(m, 1, M[i])]=key[i][m].pt.y;
                MP[IDXMATLAB(m, 2, M[i])]=key[i][m].size;
            }
            mxSetCell(plhs[0],i,tmparr);
        }
        
        if(nlhs>1) { //matching
            //descriptor
            vector<Mat> des(NIMG);
            double td = (double)getTickCount();
            for(int i=0; i<NIMG; ++i) {
                descriptor->compute(img[i], key[i], des[i]);
            }
            td = (double)getTickCount() - td;
            cout<<"[cvsurf] descriptor time = "
                    <<td/getTickFrequency()*1000.0<<" ms."<<endl;
            //matcher
            plhs[1] = mxCreateCellMatrix(NIMG,NIMG);
            vector<DMatch> matches;
            double tm = (double)getTickCount();
            for(int i=0; i<NIMG; ++i) {
                for(int j=i+1; j<NIMG; ++j) {
                    {//round 1, img[i] as train image, img[j] as query image
                        matcher->clear();
                        matches.clear();
                        matcher->match(des[j], des[i], matches);
                        
                        int S = (int)matches.size();
                        mxArray* tmparr = mxCreateDoubleMatrix(S, 2, mxREAL);
                        double *MP = mxGetPr(tmparr);
                        for(int k=0; k<S; ++k) {
                            const DMatch& m = matches[k];
                            MP[IDXMATLAB(k, 0, S)] = m.trainIdx;
                            MP[IDXMATLAB(k, 1, S)] = m.queryIdx;
                        }
                        mwIndex subs[2] = {i, j};
                        mwIndex idx = mxCalcSingleSubscript(plhs[1], 2, subs);
                        mxSetCell(plhs[1], idx, tmparr);
                    }
                    
                    {//round 2, reverse i,j
                        matcher->clear();
                        matches.clear();
                        matcher->match(des[i], des[j], matches);
                        
                        int S = (int)matches.size();
                        mxArray* tmparr = mxCreateDoubleMatrix(S, 2, mxREAL);
                        double *MP = mxGetPr(tmparr);
                        for(int k=0; k<S; ++k) {
                            const DMatch& m = matches[k];
                            MP[IDXMATLAB(k, 0, S)] = m.trainIdx;
                            MP[IDXMATLAB(k, 1, S)] = m.queryIdx;
                        }
                        mwIndex subs[2] = {j, i};
                        mwIndex idx = mxCalcSingleSubscript(plhs[1], 2, subs);
                        mxSetCell(plhs[1], idx, tmparr);
                    }
                }
                mxArray* tmparr = mxCreateDoubleMatrix(0, 0, mxREAL);
                mwIndex subs[2] = {i,i};
                mwIndex idx = mxCalcSingleSubscript(plhs[1],2,subs);
                mxSetCell(plhs[1],idx,tmparr);
            }
            tm = (double)getTickCount() - tm;
            cout<<"[cvsurf] matcher time = "
                    <<tm/getTickFrequency()*1000.0<<" ms."<<endl;
        }
    }
    
    return;
}
