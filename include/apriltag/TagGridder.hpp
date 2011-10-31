#pragma once
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

/* TagGridder.hpp
	modified from april.util.Gridder.java
*/

//standard include
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include <set>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>

//opencv include
#include "OpenCVHelper.h"

namespace april
{
namespace tag
{

using std::vector;
using std::list;

/** A lookup table in 2D for implementing nearest neighbor **/
template<typename T>
struct Gridder {
	typedef T* Tptr;
	typedef list<Tptr> Cell;
	typedef typename list<Tptr>::iterator CellIterator;

	vector<vector<Cell> > cells;
	double x0, y0, x1, y1;
	int width, height;
	double metersPerCell;

	Gridder(double x0, double y0, double x1, double y1, double metersPerCell) {
		this->x0 = x0;
		this->y0 = y0;
		this->metersPerCell = metersPerCell;

		width = (int) ((x1 - x0)/metersPerCell + 1);
		height = (int) ((y1 - y0)/metersPerCell + 1);

		this->x1 = x0 + metersPerCell*width;
		this->y1 = y0 + metersPerCell*height;

		cells.resize(height);
		for(int i=0; i<height; ++i) {
			cells[i].resize(width);
		}
	}

	inline void add(double x, double y, Tptr o) {
		int ix = (int) ((x - x0)/metersPerCell);
		int iy = (int) ((y - y0)/metersPerCell);

		if (ix >=0 && iy >=0 && ix < width && iy < height) {
			cells[iy][ix].push_back(o);
		}
	}

	inline void remove(double x, double y, Tptr o) {
		int ix = (int) ((x - x0)/metersPerCell);
		int iy = (int) ((y - y0)/metersPerCell);

		if (ix >=0 && iy >=0 && ix < width && iy < height) {
			cells[iy][ix].remove(o);
		}
	}

	struct Iterator {
		int ix0, ix1, iy0, iy1;

		int ix, iy;
		Gridder *g;
		Cell *c;
		CellIterator citr;

		Iterator(int ix0=0, int ix1=0, int iy0=0, int iy1=0, Gridder *g=0) {
			this->ix0=ix0;
			this->ix1=ix1;
			this->iy0=iy0;
			this->iy1=iy1;
			this->g = g;
			ix = ix0;
			iy = iy0;
			c = 0;
			if(g) {
				c = &(g->cells[iy][ix]);
				citr = (*c).begin();
			}
		}

		inline bool valid() {
			return (g && c && !c->empty());
		}

		inline Tptr get() {
			return valid() ? (*citr) : 0;
		}

		void next() {
			if(!valid()) {
				return;
			}

			++citr;
			if (citr != c->end()) {
				return;
			}
			c = 0;

			//go to next cell
			ix++;
			while (true) {
				if (ix > ix1) {
					iy++;
					ix = ix0;
				}
				if (iy > iy1) {
					return;
				}

				c = &(g->cells[iy][ix]);
				if (!c->empty()) {
					citr = c->begin();
					return;
				}
				c = 0;
				ix++;
			}
		}
	};

	inline Iterator find(double x, double y, double range) {
		int ix0 = (int) ((x - range - x0)/metersPerCell);
		int iy0 = (int) ((y - range - y0)/metersPerCell);

		int ix1 = (int) ((x + range - x0)/metersPerCell);
		int iy1 = (int) ((y + range - y0)/metersPerCell);

		ix0 = std::max(0, ix0);
		ix0 = std::min(width-1, ix0);

		ix1 = std::max(0, ix1);
		ix1 = std::min(width-1, ix1);

		iy0 = std::max(0, iy0);
		iy0 = std::min(height-1, iy0);

		iy1 = std::max(0, iy1);
		iy1 = std::min(height-1, iy1);

		return Iterator(ix0,ix1,iy0,iy1,this);
	}
};

}//end of tag
}//end of april
