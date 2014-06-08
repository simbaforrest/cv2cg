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
//opencv include
#include "opencv2/opencv.hpp"

namespace esm {

using namespace std;
using namespace cv;

struct Interface {
	int maxIter;
	double mprec;

	Interface() { maxIter=5; mprec=2; }

	/**
	init the esm tracker
	
	@param[in] refimg reference image, i.e. template image
	@return true if sucess
	*/
	virtual bool init(const Mat& refimg) = 0;

	/**
	main tracking function
	
	@param[in] curimg current image, i.e. image to be tracked
	@param[in,out] H input homography, should be last frame's H or
	                 close to current H
	@param[out] zncc zero mean normalized cross correlation
	@param[out] rms root-mean-square error
	@return true if sucess
	*/
	virtual bool operator()(Mat& curimg, Mat& H, double& zncc, double& rms) = 0;

	/**
	set termination criterion, smaller maxIter and smaller mprec means less computations
	
	@param maxIter max number of iterations
	@param mprec terminate when abs(RMS(new)-RMS(old))<0.01/mprec
	*/
	virtual void setTermCrit(int maxIter=5, double mprec=2) = 0;
};

};

#ifdef ESM_ORIGINAL
#include "ESMHelper.hpp" //use original implementation of ESM author
#else
#include "esm.hpp" //use our own implementation
#endif
