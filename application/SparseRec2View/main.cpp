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

/* SparseRec2View 
   main.cpp */

//standard include
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "Log.hxx" // for log control
#include "Log.h"
#include "SparseRec2View.h"

using namespace std;

int main(int argc, char** argv)
{
	if(argc<4) {
		LogI("Usage:\n\tSparseRec2View.exe <name of config file>"
			" <name of image file 1> <name of image file 2>\n");
		LogI("config file is composed of K and lamda\n");
		LogI("Example:\n\tSparseRec2View.exe cam.txt left.jpg righ.jpg\n");
		return -1;
	}

	Log::debug = false;

	double K[9]={0}, lamda=1;
	std::ifstream in(argv[1]);
	for(int i=0; i<9; ++i) in >> K[i];
	in >> lamda;
	TagD("K=\n");
	for(int i=0; i<3; ++i) {
		for(int j=0; j<3; ++j)
			LogD("\t%lf",K[i*3+j]);
		LogD("\n");
	}
	TagD("lamda=%lf\n",lamda);

	bool onlymatch=false;
	if(argc>4) {
		if( argv[4] == std::string("-onlymatch") ) {
			TagI("onlymatch turned on, no reconstruction.\n");
			onlymatch=true;
		}
	}

	string ipath1(argv[2]);
	string ipath2(argv[3]);
	SparseRec2View tvs( ipath1,
			    ipath2,
			    K,
			    lamda,
			    onlymatch );

	TagI("Begin :\n");
	tvs.run();
	tvs.save();
	TagI("Done!\n");
	
	return 0;
}
