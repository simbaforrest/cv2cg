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

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "OpenCVHelper.h"
#include "Log.h"

#include "apriltag/apriltag.hpp"
#include "apriltag/TagFamilyFactory.hpp"

using namespace std;
using namespace cv;
using april::tag::INT64;
using april::tag::TagFamily;
using april::tag::TagFamilyFactory;
using april::tag::TagDetector;
using april::tag::TagDetection;

void usage( int argc, char **argv ) {
	cout<< "[usage] " <<argv[0]<<" <output dir> [TagFamily ID] [Tag Scale]"<<endl;
	cout<< "Supported TagFamily ID List:\n";
	for(int i=0; i<(int)TagFamilyFactory::TAGTOTAL; ++i) {
		cout<<"\t"<<TagFamilyFactory::SUPPORT_NAME[i]<<" id="<<i<<endl;
	}
	cout<<"default ID:    0"<<endl;
	cout<<"default Scale: 1.0"<<endl;
}

Log::Level Log::level = Log::LOG_INFO;

int main( int argc, char **argv )
{
	if(argc<2) {
		usage(argc,argv);
		return -1;
	}

	//// create tagFamily
	int tagid = 0; //default tag16h5
	if(argc>2) tagid = atoi(argv[2]);
	cv::Ptr<TagFamily> tagFamily = TagFamilyFactory::create(tagid);
	if(tagFamily.empty()) {
		loglne("[main] create TagFamily fail!");
		return -1;
	}

	string dir(argv[1]);
	double tagscale = 1.0;
	if(argc>3) tagscale = atof(argv[3]);
	tagFamily->writeAllImages(helper::legalDir(dir),tagscale);
	tagFamily->writeAllImagesMosaic(dir+"mosaic.png");
	tagFamily->writeAllImagesPostScript(dir+"all.ps");

	loglni("[main] DONE!");
	return 0;
}
