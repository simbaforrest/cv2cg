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

/* ImageHelper.h
   Image Processing related helper functions*/

//standard include
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <time.h>
//opencv include
#include "OpenCVHelper.h"

namespace ImageHelper {

	// generate pseudocolor look up table
	// maxcolors: required max number of colors
	inline std::vector<CvScalar> pseudocolor(int maxcolors)
	{
		//find proper number of bit_per_channle
		//maxcolors = std::min(1<<30,maxcolors);
		int bit_per_channle, bits_total, color_num;
		for(bit_per_channle=1; bit_per_channle<11; ++bit_per_channle) {
			bits_total = 3*bit_per_channle;
			color_num = 1 << bits_total;
			if(color_num>=maxcolors || bit_per_channle==10) {
				--bit_per_channle;
				bits_total = 3*bit_per_channle;
				color_num = 1 << bits_total;
				break;
			}
		}

		std::vector<CvScalar> lut(color_num);

		for(int c = 0; c < color_num; c++){
			int r = 0;
			int g = 0;
			int b = 0;
			for(int k = 0; k < bits_total; ){
				b = (b << 1) + ((c >> k++) & 1);
				g = (g << 1) + ((c >> k++) & 1);
				r = (r << 1) + ((c >> k++) & 1);
			}
			r = r << (8 - bit_per_channle);
			g = g << (8 - bit_per_channle);
			b = b << (8 - bit_per_channle);

			lut[c].val[0]=r; lut[c].val[1]=g; lut[c].val[2]=b;
		}

		return lut;
	}
}//ImageHelper
