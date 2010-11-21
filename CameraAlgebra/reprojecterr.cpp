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

/* reprojecterr.cpp */

#include "CameraAlgebra.h"

namespace CameraAlgebra {
	// Evaluate estimated projection matrix P by
	// ReprojectErr=ImagePoint-WorldPoint*P
	// meanErr, mean error of ReprojectErr
	// maxErr, max error of ReprojectErr
	// sumsqErr, sum of squared error of ReprojectErr
	// sd, standard deviation
	// detail error for each points will be reported
	// by MatrixManip::Log::i
	void ReprojectionError(int npoints, double const *p,
		double const *X, double const *P, double& meanErr,
		double& maxErr, double& sumsqErr, double& sd)
	{
		MatrixManip::Log::i(
			"ReprojectionError :",
		__FUNCTION__);
#define ReportErr(msg) MatrixManip::Log::i(msg)
		double sumErr = 0;
		meanErr = maxErr = sumsqErr = sd = 0;
		for(int i=0; i<npoints; ++i) {
			double pp[2];
			Project(P, X+3*i, pp);
			double du,dv;
			du = p[2*i+0] - pp[0];
			dv = p[2*i+1] - pp[1];
			char msg[100];
			sprintf_s<100>(msg, "(% 10f, % 10f)", du, dv);
			ReportErr(msg);

			double sqErr = du*du+dv*dv;
			double err = sqrt(sqErr);
			sumErr += err;
			sumsqErr += sqErr;
			maxErr = max(maxErr, err);
		}
		meanErr = sumErr/npoints;
		if(npoints==1)
			sd = 0;
		else
			sd=sqrt((sumsqErr - sumErr*sumErr/npoints) / (npoints-1));

		char msg[100];
		sprintf_s<100>(msg, "Mean Re-project Error=% 10f", meanErr);
		ReportErr(msg);
		sprintf_s<100>(msg, "Max  Re-project Error=% 10f", maxErr);
		ReportErr(msg);
		sprintf_s<100>(msg, "Sum of squared Errors=% 10f", sumsqErr);
		ReportErr(msg);
		sprintf_s<100>(msg, "Standard Deviation=% 10f", sd);
		ReportErr(msg);
#undef ReportErr
		MatrixManip::Log::i(
			"ReprojectionError done!",
		__FUNCTION__);
	}
}//namespace
