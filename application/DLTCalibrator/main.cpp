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

/* DLTCalibrator */

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "OpenCVHelper.h"
#include "Log.h"

/////////////////////////////////////////////
// Camear Calibration using Direct Linear Transform(DLT)
// p npoints*2, image points
// X npoints*3, world points
// P 3*4, projection matrix, sp = PX
// by normalization, meaning Y=SX => X=inv(S)Y => sp = (P*inv(S))Y
// so if sp = P'Y, then P = P'*S
namespace helper {
	bool DLT(int npoints, double const *p, double const *X,
		double *P)
	{
		if(npoints<6) {
			TagE("require at least 6 correspondences!\n");
			return false;
		}

		//solution matrix A, dim = (2*npoints)*12
		double *A = new double[2*npoints*12];
		helper::zeros(2*npoints,12,A);
		//normalized points
		double *Y = new double[npoints * 3];

		using helper::FixMat;
		FixMat<2, double const>::Type _p
			= FixMat<2, double const>::ConvertType(p);
		FixMat<3, double const>::Type _X
			= FixMat<3, double const>::ConvertType(X);
		FixMat<12, double>::Type _A
			= FixMat<12, double>::ConvertType(A);
		FixMat<3, double>::Type _Y
			= FixMat<3, double>::ConvertType(Y);

		// find mean, i.e. center point
		double c[3]={0,0,0};
		for(int i=0; i<npoints; ++i) {
			c[0]+=_X[i][0];
			c[1]+=_X[i][1];
			c[2]+=_X[i][2];
		}
		c[0]/=npoints; c[1]/=npoints; c[2]/=npoints;
		// find scale factor s
		double dist2 = 0;
		for(int i=0; i<npoints; ++i) {
			_Y[i][0]=_X[i][0]-c[0];
			_Y[i][1]=_X[i][1]-c[1];
			_Y[i][2]=_X[i][2]-c[2];
			dist2 += _Y[i][0]*_Y[i][0]+_Y[i][1]*_Y[i][1]+_Y[i][2]*_Y[i][2];
		}
		double s = sqrt(2*npoints/dist2);
		for(int i=0; i<npoints; ++i) {
			_Y[i][0]*=s; _Y[i][1]*=s; _Y[i][2]*=s;
		}

		// Form the solution matrix A
		for(int i=0; i<npoints; ++i) {
			_A[2*i][0] = -1 * _Y[i][0];
			_A[2*i][1] = -1 * _Y[i][1];
			_A[2*i][2] = -1 * _Y[i][2];
			_A[2*i][3] = -1 * 1;
			_A[2*i][8] = _p[i][0] * _Y[i][0];
			_A[2*i][9] = _p[i][0] * _Y[i][1];
			_A[2*i][10]= _p[i][0] * _Y[i][2];
			_A[2*i][11]= _p[i][0] * 1;

			_A[2*i+1][4] = -1 * _Y[i][0];
			_A[2*i+1][5] = -1 * _Y[i][1];
			_A[2*i+1][6] = -1 * _Y[i][2];
			_A[2*i+1][7] = -1 * 1;
			_A[2*i+1][8] = _p[i][1] * _Y[i][0];
			_A[2*i+1][9] = _p[i][1] * _Y[i][1];
			_A[2*i+1][10]= _p[i][1] * _Y[i][2];
			_A[2*i+1][11]= _p[i][1] * 1;
		}

		double Pp[12] ={0}; //P'
		helper::nullvector(2*npoints,12,A,Pp);

		double S[16]={
			s,0,0,-s*c[0],
			0,s,0,-s*c[1],
			0,0,s,-s*c[2],
			0,0,0,1
		};

		helper::mul(3,4,4,4,Pp,S,P); //P = P'*S

		delete [] A;
		delete [] Y;
		return true;
	}

	// Evaluate estimated projection matrix P by
	// ReprojectErr=ImagePoint-WorldPoint*P
	// meanErr, mean error of ReprojectErr
	// maxErr, max error of ReprojectErr
	// sumsqErr, sum of squared error of ReprojectErr
	// sd, standard deviation
	// detail error for each points will be reported
	// by Log
	void reprojectionError(int npoints, double const *p,
		double const *X, double const *P, double& meanErr,
		double& maxErr, double& sumsqErr, double& sd)
	{
		LogI("ReprojectionError :\n");
		double sumErr = 0;
		meanErr = maxErr = sumsqErr = sd = 0;
		for(int i=0; i<npoints; ++i) {
			double pp[2];
			helper::project(P, X+3*i, pp);
			double du,dv;
			du = p[2*i+0] - pp[0];
			dv = p[2*i+1] - pp[1];
			LogI("(% 10f, % 10f)\n", du, dv);

			double sqErr = du*du+dv*dv;
			double err = sqrt(sqErr);
			sumErr += err;
			sumsqErr += sqErr;
			maxErr = std::max(maxErr, err);
		}
		meanErr = sumErr/npoints;
		if(npoints==1)
			sd = 0;
		else
			sd=sqrt((sumsqErr - sumErr*sumErr/npoints) / (npoints-1));

		LogI("Mean Re-project Error=% 10f\n", meanErr);
		LogI("Max  Re-project Error=% 10f\n", maxErr);
		LogI("Sum of squared Errors=% 10f\n", sumsqErr);
		LogI("Standard Deviation=% 10f\n", sd);
		LogI("ReprojectionError done!\n");
	}

}


/////////////////////////////////////////////

#define MAX_NUM 1000
int g_num;
double ipa[MAX_NUM*2];
double wpa[MAX_NUM*3];
double K[4];//(fx,fy,u0,v0)

void readCP(std::ifstream& fin)
{
	g_num=0;
	while(fin) {
		std::string str;
		std::getline(fin,str,'\n');
		if(str.length()==0) continue;

		if(str.find_first_of("PnP")==0) {
			//usePnP = true;
			std::stringstream ss;
			ss << str;
			std::string tmpstr;
			ss >> tmpstr >> K[0] >> K[1] >> K[2] >> K[3];
			continue;
		}

		double x,y,z,u,v;
		sscanf(str.c_str(),
			"%lf%*[^0-9-+.eE]%lf%*[^0-9-+.eE]%lf%*[^0-9-+.eE]%lf%*[^0-9-+.eE]%lf",
			&x,&y,&z,&u,&v);
		ipa[IDX(g_num,0,2)]=u; ipa[IDX(g_num,1,2)]=v;
		//ipa[g_num][0]=u; ipa[g_num][1]=v;
		wpa[IDX(g_num,0,3)]=x; wpa[IDX(g_num,1,3)]=y; wpa[IDX(g_num,2,3)]=z;
		//wpa[g_num][0]=x; wpa[g_num][1]=y; wpa[g_num][2]=z;
		++g_num;
	}
	fin.close();
}

void calibrate(std::ofstream& fout)
{
	double P[3][4], KK[3][3], R[3][3], T[3], C[3];
	helper::DLT(g_num, ipa, wpa, P[0]);
	double M[3][3];//P<3x4>=[M<3x3>=K<3x3>*R<3x3>|K<3x3>*T<3x1>]
	for(int i=0; i<3; ++i) for(int j=0; j<3; ++j)
		M[i][j] = P[i][j];
	if(helper::det(3, M[0])<0)
		helper::scale(3,4,P[0],-1,P[0]); //make sure det(M) > 0
	helper::decompose(P[0], KK[0], R[0], T, C);
	fout<<"K(alphaX alphaY u0 v0)="<<std::endl;
	fout<<KK[0][0]/KK[2][2]<<" "<<KK[1][1]/KK[2][2]<<" "
		<<KK[0][2]/KK[2][2]<<" "<<KK[1][2]/KK[2][2]<<std::endl;
	fout<<"R=\n"<<helper::PrintMat<>(3,3,R[0]);
	fout<<"C=\n"<<helper::PrintMat<>(3,1,C);
	fout<<"T=\n"<<helper::PrintMat<>(3,1,T);
	fout<<"P=\n"<<helper::PrintMat<>(3,4,P[0]);
	fout.close();
	double meanErr,maxErr,sumsqErr,sd;
	helper::reprojectionError(g_num, ipa, wpa, P[0],
		meanErr, maxErr, sumsqErr, sd);
}

Log::Level Log::level = Log::LOG_INFO;

int main( int argc, char **argv )
{
	if(argc<=1) {
		LogI("Usage: \n\tDLTCalibrator <InputFileName>"
			" [<OutPutFileName>(Without extention!)]\n"
			"Example: \n\tDLTCalibrator test.cp\n"
			"\tDLTCalibrator test.cp D:/Out\n");
		return 1;
	}

	std::string ifname(argv[1]);
	std::string ofname;
	if(argc<3) {
		std::string fnameNoExt(ifname);
		int len = fnameNoExt.length();
		if(len>3 && fnameNoExt[len-1]=='p' &&
			fnameNoExt[len-2]=='c' &&
			fnameNoExt[len-3]=='.')
			fnameNoExt.resize(len-3);
		ofname = fnameNoExt + std::string(".par");
	} else {
		ofname = std::string(argv[2]);
		ofname += std::string(".par");
	}

	LogI("IN_FILE_NAME=%s\n",ifname.c_str());
	LogI("OUT_FILE_NAME=%s\n",ofname.c_str());

	std::ifstream ifile(ifname.c_str());
	std::ofstream ofile(ofname.c_str());

	readCP(ifile);
	LogI("READ_NUM_POINTS=%d\n",g_num);
	calibrate(ofile);
	return 0;
}
