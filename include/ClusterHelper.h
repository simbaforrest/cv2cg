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

/* ClusterHelper.h
   helper functions related to clusterring */

#include <vector>
#include <set>

namespace ClusterHelper
{

//modified from april.util.UnionFindSimple
struct UnionFind {
	std::vector<int> parent;
	std::vector<int> rank; //store class size
//	set<int> root; //store all remaining roots
//	int nclasses;

	//id range [0, maxid-1]
	UnionFind(int maxid) {
//		nclasses = maxid;
		parent.resize(maxid);
		rank.resize(maxid);
		for(int i=0; i<maxid; ++i) {
			parent[i]=i;
			rank[i]=1;
//			root.insert(i);
		}
	}

	inline int ClassSize(int id) {
		return rank[Find(id)];
	}

	inline int Find(int id) {
		if( parent[id]==id ) {
			return id;
		}
		return parent[id] = Find(parent[id]); //path compression
	}

	inline int Union(int xid, int yid) {
		int xroot = Find(xid);
		int yroot = Find(yid);
		if(xroot==yroot) {
			return xroot;
		}

		//merge by rank
//		--nclasses;
		int xrank = rank[xroot];
		int yrank = rank[yroot];
		if(xrank>yrank) {
			parent[yroot] = xroot;
			rank[xroot] += yrank;
			return xroot;
//			root.erase(xroot);
		}
		parent[xroot] = yroot;
		rank[yroot] += xrank;
		return yroot;
//		root.erase(yroot);
	}
};

}//end of ClusterHelper
