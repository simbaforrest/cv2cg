#pragma once
/*
 *  Copyright (c) 2010  Chen Feng (cforrest (at) umich.edu)
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

/* FilterHelper.h
   some filter functions */

#include <math.h>

namespace FilterHelper
{

template<typename T>
inline void stablize(int dim,
                     T const *newObservation, T *filtered,
                     T bufferLen=1.0f, T amplification=500.0f)
{
	for(int i=0; i<dim; ++i) {
		filtered[i] += amplification;
		double tmp = newObservation[i] + amplification;
		filtered[i] = (T) (sqrt((filtered[i] * filtered[i] * bufferLen
		                         + tmp * tmp) / (1 + bufferLen)));
		filtered[i] -= amplification;
	}
}

}//end of namespace FilterHelper
