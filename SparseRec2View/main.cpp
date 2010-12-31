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

#include "SparseRec2View.h"

using namespace std;

int main(int argc, char** argv)
{
	if(argc<4) {
		cout<<"Usage:\n\t"<<argv[0]<<
			" <name of config file> <name of image file 1> <name of image file 2>"<<endl;
		cout<<"config file is composed of K and lamda"<<endl;
		cout<<"Example:\n\t"<<argv[0]<<" cam.txt left.jpg righ.jpg"<<endl;
		return -1;
	}

	double K[9]={0}, lamda=1;
	std::ifstream in(argv[1]);
	for(int i=0; i<9; ++i) in >> K[i];
	in >> lamda;
	cout<<"[main] K="<<endl;
	for(int i=0; i<3; ++i) {
		for(int j=0; j<3; ++j) {
			cout << K[i*3+j] << " ";
		}
		cout << endl;
	}
	cout<<"[main] lamda="<<lamda<<endl;

	SparseRec2View tvs(string(argv[2]), string(argv[3]), K, lamda);

	cout<<"[main] Begin:"<<endl;
	tvs.run();
	tvs.save();
	cout<<"[main] Done!"<<endl;
	
	return 0;
}