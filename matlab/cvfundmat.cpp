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

/*[F12,inliers]=cvfundmat(x1,x2)
 *INPUT
 *  x1 <Nx2>: key points in image 1
 *  x2 <Nx2>: key points in image 2
 *OUTPUT
 *  F12 <3x3>: fundamental matrix, x2'*F12*x1 = 0
 *  [inliers <Kx1>]: optional, inlier matched pairs
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

//matlab include
#include "mex.h"
#include "matrix.h"
//opencv include
#include "opencv2/opencv.hpp"


using namespace std;
using namespace cv;

#define IDXMATLAB(m,n,M) ((n)*(M)+(m)) //M rows

void mexFunction(int nlhs, mxArray* plhs[], 
                 int nrhs, const mxArray* prhs[]) {
    if(nrhs<2)
		mexErrMsgTxt("[cvfundmat] not enough input parameters!");
    if(nlhs>2)
        mexErrMsgTxt("[cvfundmat] too many output parameters!");
    
    unsigned int Npts = mxGetM(prhs[0]);
    if(Npts<=7)
        mexErrMsgTxt("[cvfundmat] input point pairs size must be at least 7!");
    if( Npts != mxGetM(prhs[1]) )
        mexErrMsgTxt("[cvfundmat] input point pairs size must be the same!");
    if( mxGetN(prhs[0])!=2 || mxGetN(prhs[1])!=2 )
        mexErrMsgTxt("[cvfundmat] input point should be 2D point!");
    
    vector<Point2f> pi, pj;//matched image points
    pi.reserve(Npts);
    pj.reserve(Npts);
    double const *MP0 = mxGetPr(prhs[0]);
    double const *MP1 = mxGetPr(prhs[1]);
    for(int i=0; MP0 && MP1 && i<Npts; ++i) {
        double ui = MP0[IDXMATLAB(i,0,Npts)];
        double vi = MP0[IDXMATLAB(i,1,Npts)];
        pi.push_back(Point2f(ui,vi));
        double uj = MP1[IDXMATLAB(i,0,Npts)];
        double vj = MP1[IDXMATLAB(i,1,Npts)];
        pj.push_back(Point2f(uj,vj));
    }
    for(int i=0; MP0 && MP1 && i<Npts; ++i) {
        cout<<pi[i].x<<","<<pi[i].y<<"<->"<<pj[i].x<<","<<pj[i].y<<endl;
    }

    //find fundamental matrix
    //vector<uchar> inliers;
    //double tt = (double)getTickCount();
//     try {
    ///////////////////////////////////////////////////////////////code will crash in cvFindFundamentalMat, in cvSVD, I do not know why, leave it here...
    Mat F(3, 3, CV_64F);
    CvMat _pt1 = Mat(pi), _pt2 = Mat(pj);
    CvMat matF = F, _mask, *pmask = 0;
//     if( mask )
//     {
//         mask->resize(points1.cols*points1.rows*points1.channels()/2);
//         pmask = &(_mask = cvMat(1, (int)mask->size(), CV_8U, (void*)&(*mask)[0]));
//     }
    int n = cvFindFundamentalMat( &_pt1, &_pt2, &matF, CV_FM_RANSAC, 1, 0.99, pmask );
    if( n <= 0 )
        F = Scalar(0);
    cout<<F<<endl;
         //Mat F12 = findFundamentalMat(Mat(pi), Mat(pj));
//     } catch(...) {
//         cout<<"error!"<<endl;
//     }
    //tt = (double)getTickCount() - tt;
    //cout<<"[cvfundmat] RANSAC time = "
    //        <<tt/getTickFrequency()*1000.0<<" ms."<<endl;
    
//     {
//         plhs[0] = mxCreateDoubleMatrix(3, 3, mxREAL);
//         double *MP = mxGetPr(plhs[0]);
//         for(int i=0; i<3; ++i)
//             for(int j=0; j<3; ++j)
//                 MP[IDXMATLAB(i, j, 3)]=F12.at<double>(i, j);
//     }
//     if(nlhs>1) {
//                 unsigned int NI = inliers.size();
//                 plhs[1] = mxCreateDoubleMatrix(NI, 1, mxREAL);
//                 double *MP = mxGetPr(plhs[1]);
//                 for(int i=0; i<NI; ++i)
//                     MP[IDXMATLAB(i,0,NI)]=inliers[i];
//     }

    return;
}
