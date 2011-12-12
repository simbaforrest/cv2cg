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
  
  Authors:

			Chen Feng
            Laboratory for Interactive Visualization in Engineering (LIVE)
			Department of Civil and Environmental Engineering
            2350 Hayward Street, Suite 2340 GG Brown Building
            University of Michigan
            Ann Arbor, MI 48109-2125
			Phone:    (734)764-8495
			EMail:    simbaforrest@gmail.com
			WWW site: http://www.umich.edu/~cforrest
            
			Vineet R. Kamat
            Laboratory for Interactive Visualization in Engineering (LIVE)
			Department of Civil and Environmental Engineering
            2350 Hayward Street, Suite 2340 GG Brown Building
            University of Michigan
            Ann Arbor, MI 48109-2125
            Phone:    (734)764-4325
			EMail:    vkamat@umich.edu
			WWW site: http://pathfinder.engin.umich.edu

\************************************************************************/

/* SparseRec2View
   main.cpp */

//standard include
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "Log.h"
#include "SparseRec2View.h"

using namespace std;

Log::Level Log::level = Log::LOG_INFO;

int main(int argc, char** argv)
{
	
	if(argc<4) {
		LogI("Usage:\n\tSparseRec2View <name of config file>"
			" <name of image file 1> <name of image file 2>\n");
		LogI("config file is composed of K and lamda\n");
		LogI("Example:\n\tSparseRec2View cam.txt left.jpg righ.jpg\n");
		return -1;
	}

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
