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

/* SparseRecMultiView
   main.cpp */

//standard include
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "Log.h"
#include "MultiViewSparseRec.h"

using namespace std;

Log::Level Log::level = Log::LOG_INFO;

int main(int argc, char** argv)
{
	if(argc<2) {
		std::cout<<
			"Usage: MultiViewSparseRec <input file>\n"
			"Example:\n"
			"\tMultiViewSparseRec mvsr.in"<<std::endl;
		return -1;
	}
	string ifname(argv[1]);
	ifstream in(ifname.c_str());
	vector<string> pathlist;
	vector< vector<double> > caliblist;
	string outdir, mainname;

	if( !helper::readValidLine(in,outdir) ) {
		TagE("input file invalid! exit...");
		return -1;
	} else {
		TagI("outdir=%s\n",outdir.c_str());
	}

	if( !helper::readValidLine(in,mainname) ) {
		TagE("input file invalid! exit...");
		return -1;
	} else {
		TagI("mainname=%s\n",mainname.c_str());
	}

	std::string line;
	double K[9] = {
		1, 0, 0,
		0, 1, 0,
		0, 0, 1
	};
	while(helper::readValidLine(in,line)) {
		if(line.find(">>K")!=line.npos) { //change K
			for(int i=0; i<9; ++i)
				in >> K[i];
			std::cout<<"[main] new calibration matrix K =\n"
				<<helper::PrintMat<>(3,3,K)<<std::endl;
		} else {
			std::cout<<"[main] add "<<line<<std::endl;
			pathlist.push_back(line);
			vector<double> calib(9,0);
			std::copy(K,K+9,calib.begin());
			caliblist.push_back(calib);
		}
	}

	MVSR mvsr;
	mvsr.run(pathlist,caliblist,outdir,mainname);
	return 0;
}
