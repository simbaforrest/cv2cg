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
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "Log.h"
#include "OpenCVHelper.h"

#ifndef KEG_DEBUG
	#define KEG_DEBUG 0
#endif

namespace keg {

using namespace cv;
using namespace std;

/**
\class keg::KeyFrame "KeyFrame.hpp"
\brief contains keyframe id, image, rotation, translation, and quality measure
*/
struct KeyFrame {
	int id; //store the apriltag id
	Mat frame;
	double R[3][3];
	double T[3];
	double rms,ncc;
#if KEG_DEBUG
	double duration; //time to track
#endif
};

inline void CapKeyFrame(vector<KeyFrame>& keyframes,
		int id, Mat const& frame,
		double const R[3][3], double const T[3],
		double const rms, double const ncc
#if KEG_DEBUG
		, double const dur
#endif
) {
	KeyFrame newkf;
	newkf.id = id;
	if(!frame.empty()) newkf.frame = frame.clone();
#if KEG_DEBUG
	newkf.duration = dur;
#endif
	for(int i=0; i<3; ++i) {
		newkf.T[i] = T[i];
		for(int j=0; j<3; ++j) {
			newkf.R[i][j]=R[i][j];
		}
	}
	newkf.rms = rms;
	newkf.ncc = ncc;
	keyframes.push_back(newkf);
}

//	rmsncc <<"#format: frame_id rms ncc"<<endl;
//	framesR <<"#format: frame_id R[0][0] R[0][1] ... R[2][2]"<<endl;
//	framesT <<"#format: frame_id T[0] T[1] T[2]"<<endl;
//	framesDUR <<"#format: frame_id dur"<<endl;
inline bool SaveKeyFrames(const vector<KeyFrame>& keyframes, string name,
		double K[9], bool saveimg=true)
{
	cout<<"[SaveKeyFrames] "<<(int)keyframes.size()<<" frame(s) total!"<<endl;
	if(keyframes.size()<=0) return true;
	name = DirHelper::getFileDir(name);
	string mainname = helper::legalDir(name) + string("frames.main");
	string rmsnccname = name + string("frames.rmsncc");
	string framesRname = name + string("frames.R");
	string framesTname = name + string("frames.T");
#if KEG_DEBUG
	string framesDURname = name + string("frames.DUR");
#endif
	std::ofstream mainout(mainname.c_str());
	mainout << "CAMERAFRUSTUM "<<keyframes.size()<<" 1"<< endl;
	mainout << name << endl;
	std::ofstream rmsncc(rmsnccname.c_str());
	std::ofstream framesR(framesRname.c_str());
	std::ofstream framesT(framesTname.c_str());
#if KEG_DEBUG
	std::ofstream framesDUR(framesDURname.c_str());;
#endif
	for(int i=0; i<(int)keyframes.size(); ++i) {
		const KeyFrame& kf = keyframes[i];
		rmsncc <<kf.id<<" "<<kf.rms<<" "<<kf.ncc<<endl;
		framesR <<kf.id<<" "<<helper::PrintMat<>(1,9,kf.R[0]);
		framesT <<kf.id<<" "<<helper::PrintMat<>(1,3,kf.T);
#if KEG_DEBUG
		framesDUR <<kf.id<<" "<<kf.duration<<endl;
#endif

		if(kf.frame.empty() || !saveimg)
			continue;

		string num;
		helper::num2str(i,num);
		string prefix = name + string("frame") + num;
		string relativePrefix = string("frame") + num;

		string parname = prefix + string(".par");
		std::ofstream par(parname.c_str());
		par << "n=0.1\nf=10000\n" << endl;
		par << "K=\n" << helper::PrintMat<>(3,3,K) << endl;
		par << "R=\n" << helper::PrintMat<>(3,3,kf.R[0]) << endl;
		par << "T=\n" << helper::PrintMat<>(3,1,kf.T) << endl;
		par.close();
		cout<<"[SaveKeyFrames] "<<parname<<" saved."<<endl;

		string imgname = prefix + string(".jpg");
		imwrite(imgname, kf.frame);
		cout<<"[SaveKeyFrames] "<<imgname<<" saved."<<endl;

		string relativeParname = relativePrefix + string(".par");
		string relativeImgname = relativePrefix + string(".jpg");
		mainout << relativeImgname << endl;
		mainout << relativeParname << endl;
	}
	rmsncc.close();
	framesR.close();
	framesT.close();
	mainout.close();
#if KEG_DEBUG
	framesDUR.close();
#endif
	cout<<"[SaveKeyFrames] DONE!"<<endl;
	return true;
}

}; //end of namespace keg
