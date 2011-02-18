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
#include "CameraSimulator.h"

#include "Log.inc"
#include "Log.h"

int main( int argc, char **argv )
{
#ifdef USE_IN_CHINA
	setlocale(LC_ALL,"chs");
#endif
	if(argc<=1) {
		LogI("Usage: \n\tCameraSimulator.exe <InputFileName>"
			" [<OutPutFileName>(Without extention!)]\n");
		LogI("Example: \n\tCameraSimulator.exe draw.txt D:\\Out\n");
		LogI("Example: \n\tCameraSimulator.exe draw.txt\n");
		return -1;
	}

	CameraSimulator cs;
	cs.inputFileName = std::string(argv[1]);
	std::string tmp = cs.inputFileName + std::string(".out");
	cs.outputFileName = argc>2?std::string(argv[2]):tmp;
	return cs.run();
}
