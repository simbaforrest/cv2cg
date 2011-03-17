#pragma once
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

/* IOHelper.h */

//standard include
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <time.h>

namespace IOHelper {
	//read one valid line, i.e. non-empty line
	//return false if no valid line, i.e. end-of-file
	inline bool readValidLine(std::istream& in,
		std::string& line)
	{
		line = "";//clear;
		bool haschar = false;
		while(in) {
			std::string str;
			std::getline(in, str);
			if(str.length()==0) continue;
			haschar = true;
			
			//take care of CR/LF, see details:
			// http://www.codeguru.com/forum/showthread.php?t=253826
			char lastchar = str[str.length()-1];
			if(lastchar=='\n' || lastchar=='\r') {
				str.resize(str.length()-1);
			}
			if(str.length()==0) continue;
			
			line = str;
			break;
		}
		return line.length()!=0 || haschar;
	}
}//end of IOHelper
