#pragma once
/************************************************************************\

  Copyright 2011 The University of Michigan.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software
  and its documentation for educational, research and non-profit
  purposes, without fee, and without a written agreement is
  hereby granted, provided that the above copyright notice and
  the following paragraph appear in all copies.

  THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY OF MICHIGAN "AS IS" AND 
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF MICHIGAN
  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
\************************************************************************/

/* SearchHelper.h
	modified from april.util.Gridder.java
	git://april.eecs.umich.edu/home/git/april.git
*/

#include <vector>

namespace SearchHelper {

/** A lookup table in 2D for implementing nearest neighbor **/
template<typename T>
struct Gridder {
	typedef T* Tptr;
	typedef std::vector<Tptr> Cell;
	typedef int CellIterator;

	std::vector<std::vector<Cell> > cells;
	double x0, y0, x1, y1;
	int width, height;
	double metersPerCell;
	int count;

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
		count = 0;
	}

	inline void add(double x, double y, Tptr o) {
		int ix = (int) ((x - x0)/metersPerCell);
		int iy = (int) ((y - y0)/metersPerCell);

		if (ix >=0 && iy >=0 && ix < width && iy < height) {
			cells[iy][ix].push_back(o);
			++count;
		}
	}

	inline int size() { return count; }

	struct Iterator {
		int ix0, ix1, iy0, iy1;

		int ix, iy;
		Gridder *g;
		CellIterator citr;

		Iterator(int ix0=0, int ix1=0, int iy0=0, int iy1=0, Gridder *g=0) {
			this->ix0=ix0;
			this->ix1=ix1;
			this->iy0=iy0;
			this->iy1=iy1;
			this->g = g;
			ix = ix0;
			iy = iy0;
			if(g) {
				Cell& c = g->cells[iy][ix];
				citr = 0;
				if (c.empty()) next(); //first cell empty, so move to next non-empty cell
			}
		}

		inline bool done() {
			return !g;
		}

		inline Tptr get() {
			return g ? g->cells[iy][ix][citr] : 0;
		}

		//after next() finished, either citr moved to next valid position,
		//or no more valid position and g is set to be 0
		inline void next() {
			if(!g) {
				return;
			}
			const Cell& c = g->cells[iy][ix];

			++citr;
			if(citr < (int)c.size()) {//current cell not finished yet, ok to move on list
				return;
			}
			//current cell finished, find next non-empty cell
			do {
				++ix;
				if (ix > ix1) {
					iy++;
					ix = ix0;
				}
				if (iy > iy1) { //no more valid position
					g = 0;
					return;
				}

				const Cell& nc = g->cells[iy][ix];
				if (!nc.empty()) {
					citr = 0;
					return;
				}
			} while(true);
		}
	};//end of struct Gridder<T>::Iterator

	inline Iterator find(double x, double y, double range) {
		int ix0 = (int) ((x - range - x0)/metersPerCell);
		int iy0 = (int) ((y - range - y0)/metersPerCell);

		int ix1 = (int) ((x + range - x0)/metersPerCell);
		int iy1 = (int) ((y + range - y0)/metersPerCell);

		ix0 = (std::max)(0, ix0);
		ix0 = (std::min)(width-1, ix0);

		ix1 = (std::max)(0, ix1);
		ix1 = (std::min)(width-1, ix1);

		iy0 = (std::max)(0, iy0);
		iy0 = (std::min)(height-1, iy0);

		iy1 = (std::max)(0, iy1);
		iy1 = (std::min)(height-1, iy1);

		return Iterator(ix0,ix1,iy0,iy1,this);
	}

	/**
	find all elements within range
	
	@param x,y[in] search center
	@param range[in] search range
	@param dst[out] output list of elements' pointers
	@return number of elements in output list
	*/
	inline int findAll(double x, double y, double range, std::vector<Tptr>& dst) {
		dst.clear();
		Iterator itr = find(x,y,range);
		for (; !itr.done(); itr.next()) {
			Tptr p = itr.get();
			if(p) dst.push_back(p);
		}
		return (int)dst.size();
	}
};//end of struct Gridder<T>

}//end of namespace SearchHelper
