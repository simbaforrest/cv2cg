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

#include "Log.hxx" // for log control
#include "Log.h"
#include "MultiViewSparseRec.h"

using namespace std;

int main(int argc, char** argv)
{
	Log::debug = true;
	if(argc<2) {
		std::cout<<
			"Usage: MultiViewSparseRec <input file>\n"
			"Example:\n"
			"\tMultiViewSparseRec mvsr.in"<<std::endl;
		return -1;
	}
	std::string ifname(argv[1]);
	std::ifstream in(ifname.c_str());
	std::vector<std::string> imgnamelist;
	std::string outdir, mainname;

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
	while(helper::readValidLine(in,line)) {
		imgnamelist.push_back(line);
	}

	MVSR mvsr;
	mvsr.run(imgnamelist,outdir,mainname);
	return 0;
}
