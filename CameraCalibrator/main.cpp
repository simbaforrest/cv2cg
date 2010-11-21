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

/* main.cpp */

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include "CameraAlgebra.h"

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
	CameraAlgebra::DLT(g_num, ipa, wpa, P[0]);
	CameraAlgebra::Decompose(P, KK, R, C, T);
	fout<<"K(alphaX alphaY u0 v0)="<<std::endl;
	fout<<KK[0][0]<<" "<<KK[1][1]<<" "<<KK[0][2]<<" "<<KK[1][2]<<std::endl;
	MatrixManip::Print(
		fout<<"R="<<std::endl,
		R[0],3,3);
	MatrixManip::Print(
		fout<<"C="<<std::endl,
		C,3,1);
	MatrixManip::Print(
		fout<<"T="<<std::endl,
		T,3,1);
	MatrixManip::Print(
		fout<<"P="<<std::endl,
		P[0],3,4)<<std::endl;
	fout.close();
	double meanErr,maxErr,sumsqErr,sd;
	CameraAlgebra::ReprojectionError(g_num, ipa, wpa, P[0], 
		meanErr, maxErr, sumsqErr, sd);
}

int main( int argc, char **argv )
{
	if(argc<=1) {
		std::cout<<"Usage: \n\t"<<argv[0]<<" <InputFileName> [<OutPutFileName>(Without extention!)]"<<std::endl;
		std::cout<<"Example: \n\t"<<argv[0]<<" test.cp"<<std::endl;
		std::cout<<"Example: \n\t"<<argv[0]<<" test.cp D:\\Out"<<std::endl;
		return 1;
	}

	std::string ifname(argv[1]);
	std::string fnameNoExt(ifname);
	int len = fnameNoExt.length();
	if(len>3 && fnameNoExt[len-1]=='p' &&
		fnameNoExt[len-2]=='c' &&
		fnameNoExt[len-3]=='.') fnameNoExt.resize(len-3);
	std::string ofname = fnameNoExt + std::string(".par");

	std::ifstream ifile(ifname.c_str());
	std::ofstream ofile(ofname.c_str());

	readCP(ifile);
	calibrate(ofile);
	return 0;
}
