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

/* TagUtils.hpp
	modified from git://april.eecs.umich.edu/home/git/april.git
*/

//standard include
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <set>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>

//opencv include
#include "AllHelpers.h"

namespace april
{
namespace tag
{

inline double distance(double x1, double y1, double x2, double y2)
{
	return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

inline double distance(const double p1[2], const double p2[2])
{
	return distance(p1[0],p1[1], p2[0],p2[1]);
}

inline void normalizeLine(double &dx, double &dy, double &x0, double &y0)
{
	double dl = sqrt(dx*dx+dy*dy); //make [dx,dy] unit vector
	dx/=dl;
	dy/=dl;
	double dotprod=-dy*x0+dx*y0; //make [x0,y0] closest to origin
	x0 = -dy*dotprod;
	y0 = dx*dotprod;
}

//make sure [dx,dy] is unit vector before call this
inline double lineCoordinate(double x, double y, double dx, double dy)
{
	return x*dx+y*dy;
}

//weighted least square fit of 2D line
//modified from april.jmat.geom.GLine2D
//return {vx,vy,x0,y0}
//where [vx,vy] is unit direction vector
//and [x0,y0] is closest to origin
inline cv::Vec4d wlsq2D(const std::vector<cv::Vec3d>& xyw)
{
	double Cxx=0, Cyy=0, Cxy=0, Ex=0, Ey=0, mXX=0, mYY=0, mXY=0, mX=0, mY=0;
	double n=0;

	for (int i=0; i<(int)xyw.size(); ++i) {
		const cv::Vec3d &tp = xyw[i];
		double x = tp[0];
		double y = tp[1];
		double alpha = tp[2];

		mY  += y*alpha;
		mX  += x*alpha;
		mYY += y*y*alpha;
		mXX += x*x*alpha;
		mXY += x*y*alpha;
		n   += alpha;
	}

	Ex  = mX/n;
	Ey  = mY/n;
	Cxx = mXX/n - (mX/n)*(mX/n);
	Cyy = mYY/n - (mY/n)*(mY/n);
	Cxy = mXY/n - (mX/n)*(mY/n);

	// find dominant direction via SVD
	double phi = 0.5*atan2(-2*Cxy,(Cyy-Cxx));
//	double rho = Ex*cos(phi) + Ey*sin(phi); //simbaforrest: no use, skip calc it

	// compute line parameters, normalize x0,y0 so it's closest to origin
	double dx=-sin(phi), dy=cos(phi), x0=Ex, y0=Ey;
	normalizeLine(dx,dy,x0,y0);
	return cv::Vec4d(dx, dy, x0, y0);
}

/** Represents a line fit to a set of pixels whose gradients are
 * similar.
 **/
struct Segment {
	double x0, y0, x1, y1;
	double theta;  // gradient direction (points towards white)
	double length; // length of line segment in pixels
	std::vector<Segment *> children;

	inline void swap() {
		std::swap(x0,x1);
		std::swap(y0,y1);
	}

	//weighted least square fit of 2D line segments
	//modified from april.jmat.geom.GLineSegment2D
	//return {x1,y1,x2,y2}
	//so line segment is [x1,y1]<-->[x2,y2]
	inline void fitBy(const std::vector<cv::Vec3d>& xyw) {
		cv::Vec4d line = wlsq2D(xyw);

		double dx=line[0], dy=line[1], xp=line[2], yp=line[3];
		double maxcoord = -DBL_MAX, mincoord = DBL_MAX;
		for (int i=0; i<(int)xyw.size(); ++i) {
			const cv::Vec3d &tp = xyw[i];
			double lcoord = lineCoordinate(tp[0],tp[1],dx,dy);
			maxcoord = (std::max)(maxcoord, lcoord);
			mincoord = (std::min)(mincoord, lcoord);
		}
		this->x0 = xp+mincoord*dx;
		this->y0 = yp+mincoord*dy;
		this->x1 = xp+maxcoord*dx;
		this->y1 = yp+maxcoord*dy;
		length = maxcoord - mincoord;
	}

	inline bool intersectionWith(const Segment &s, double &xo, double &yo) const {
		// this implementation is many times faster than the original,
		// mostly due to avoiding a general-purpose LU decomposition in
		// Matrix.inverse().
		double m00, m01, m10, m11;
		double i00, i01;//, i10, i11;
		double b00, b10;

		m00=this->x1 - this->x0;
		m01=s.x0 - s.x1;
		m10=this->y1 - this->y0;
		m11=s.y0 - s.y1;

		// determinant of m
		double det=m00*m11-m01*m10;

		// parallel lines?
		if (std::abs(det)<0.0000000001) {
			return false;
		}

		double idet = 1.0/det;
		// inverse of m
		i00=m11*idet;
		//i11=m00*idet;
		i01=-m01*idet;
		//i10=-m10*idet;

		b00=s.x0 - this->x0;
		b10=s.y0 - this->y0;

		double x00; //, x10;
		x00=i00*b00+i01*b10;
		//	x10=i10*b00+i11*b10;

		xo = m00*x00+this->x0;
		yo = m10*x00+this->y0;
		return true;
	}
};

/** For a set of 2D points, compute a 3x3 transform that
 * will make the points have mean zero and unit variance.
 **/
struct Normalize2D {
    double mX, mY, mXX, mYY;
    int N;

	//!! do not forget default constructor for any struct !!
	Normalize2D() : mX(0), mY(0), mXX(0), mYY(0), N(0) {}

    inline void add(double x, double y)
    {
        mX += x;
        mXX += x*x;
        mY += y;
        mYY += y*y;
        N++;
    }

    inline void getTransform(double T[3][3]) const
    {
        double eX = mX / N;
        double eY = mY / N;
        double stddevX = sqrt(mXX / N + eX*eX); //sf: this has to be done!
        double stddevY = sqrt(mYY / N + eY*eY);

		double scaleX = stddevX==0 ? 1 : 1.0 / stddevX; //sf: be careful if std==0
		double scaleY = stddevY==0 ? 1 : 1.0 / stddevY;

        T[0][0]=scaleX; T[0][1]=0; T[0][2]=-eX*scaleX;
		T[1][0]=0; T[1][1]=scaleY; T[1][2]=-eY*scaleY;
		T[2][0]=0; T[2][1]=0; T[2][2]=1;
    }
	
	inline void getInverseTransform(double T[3][3]) const
	{
		double eX = mX / N;
        double eY = mY / N;
		double stddevX = sqrt(mXX / N + eX*eX); //sf: this has to be done!
		double stddevY = sqrt(mYY / N + eY*eY);

		double scaleX = stddevX==0 ? 1 : stddevX; //sf: be careful if std==0
		double scaleY = stddevY==0 ? 1 : stddevY;

        T[0][0]=scaleX; T[0][1]=0; T[0][2]=eX;
		T[1][0]=0; T[1][1]=scaleY; T[1][2]=eY;
		T[2][0]=0; T[2][1]=0; T[2][2]=1;
	}
};

/** Compute 3x3 homography using Direct Linear Transform

    y = Hx (y = image coordinates in homogeneous coordinates, H = 3x3
    homography matrix, x = homogeneous 2D world coordinates)

    For each point correspondence, constrain y x Hx = 0 (where x is
    cross product). This means that they have the same direction, and
    ignores scale factor.

    We rewrite this as Ah = 0, where h is the 9x1 column vector of the
    elements of H. Each point correspondence gives us 3 equations of
    rank 2. The solution h is the minimum right eigenvector of A,
    which we can obtain via SVD, USV' = A. h is the right-most column
    of V'.

    We will actually maintain A'A internally, which is 9x9 regardless
    of how many correspondences we're given, and has the same
    eigenvectors as A.
	
	//simbaforrest
	this Homography33 class does not implement correspondence point's normalization
**/
struct Homography33 {
	double A[9][9];
	double H[3][3]; // homography, once computed.
	bool computed;

	double cx, cy;

	Homography33(double cx=0, double cy=0) {
		this->cx = cx;
		this->cy = cy;
		helper::zeros(9,9,A[0]);
		helper::zeros(3,3,H[0]);
		computed = false;
	}

	inline void reset(double cx, double cy) {
		this->cx = cx;
		this->cy = cy;
		helper::zeros(9,9,A[0]);
		helper::zeros(3,3,H[0]);
		computed = false;
	}

	/** Note that the returned H matrix does not reflect cx, cy. **/
	inline void getH(double h[3][3]) {
		compute();
		std::copy((double*)H[0], (double*)H[0]+9, (double*)h[0]);
	}

	inline void getCXY(double cxy[2]) const {
		cxy[0]=cx;
		cxy[1]=cy;
	}

	inline void addCorrespondence(double worldx, double worldy, double imagex, double imagey) {
		imagex -= cx;
		imagey -= cy;

		/** Here are the rows of matrix A.  We will compute A'*A
		    A[3*i+0][3] = -worldxyh[i][0]*imagexy[i][2];
		    A[3*i+0][4] = -worldxyh[i][1]*imagexy[i][2];
		    A[3*i+0][5] = -worldxyh[i][2]*imagexy[i][2];
		    A[3*i+0][6] =  worldxyh[i][0]*imagexy[i][1];
		    A[3*i+0][7] =  worldxyh[i][1]*imagexy[i][1];
		    A[3*i+0][8] =  worldxyh[i][2]*imagexy[i][1];

		    A[3*i+1][0] =  worldxyh[i][0]*imagexy[i][2];
		    A[3*i+1][1] =  worldxyh[i][1]*imagexy[i][2];
		    A[3*i+1][2] =  worldxyh[i][2]*imagexy[i][2];
		    A[3*i+1][6] = -worldxyh[i][0]*imagexy[i][0];
		    A[3*i+1][7] = -worldxyh[i][1]*imagexy[i][0];
		    A[3*i+1][8] = -worldxyh[i][2]*imagexy[i][0];

		    A[3*i+2][0] = -worldxyh[i][0]*imagexy[i][1];
		    A[3*i+2][1] = -worldxyh[i][1]*imagexy[i][1];
		    A[3*i+2][2] = -worldxyh[i][2]*imagexy[i][1];
		    A[3*i+2][3] =  worldxyh[i][0]*imagexy[i][0];
		    A[3*i+2][4] =  worldxyh[i][1]*imagexy[i][0];
		    A[3*i+2][5] =  worldxyh[i][2]*imagexy[i][0];
		**/

		// only update upper-right. A'A is symmetric, we'll finish the lower left later.
		double a03 = -worldx;
		double a04 = -worldy;
		double a05 = -1;
		double a06 = worldx*imagey;
		double a07 = worldy*imagey;
		double a08 = imagey;

		A[3][3] += a03*a03;
		A[3][4] += a03*a04;
		A[3][5] += a03*a05;
		A[3][6] += a03*a06;
		A[3][7] += a03*a07;
		A[3][8] += a03*a08;
		A[4][4] += a04*a04;
		A[4][5] += a04*a05;
		A[4][6] += a04*a06;
		A[4][7] += a04*a07;
		A[4][8] += a04*a08;
		A[5][5] += a05*a05;
		A[5][6] += a05*a06;
		A[5][7] += a05*a07;
		A[5][8] += a05*a08;
		A[6][6] += a06*a06;
		A[6][7] += a06*a07;
		A[6][8] += a06*a08;
		A[7][7] += a07*a07;
		A[7][8] += a07*a08;
		A[8][8] += a08*a08;

		double a10 = worldx;
		double a11 = worldy;
		double a12 = 1;
		double a16 = -worldx*imagex;
		double a17 = -worldy*imagex;
		double a18 = -imagex;

		A[0][0] += a10*a10;
		A[0][1] += a10*a11;
		A[0][2] += a10*a12;
		A[0][6] += a10*a16;
		A[0][7] += a10*a17;
		A[0][8] += a10*a18;
		A[1][1] += a11*a11;
		A[1][2] += a11*a12;
		A[1][6] += a11*a16;
		A[1][7] += a11*a17;
		A[1][8] += a11*a18;
		A[2][2] += a12*a12;
		A[2][6] += a12*a16;
		A[2][7] += a12*a17;
		A[2][8] += a12*a18;
		A[6][6] += a16*a16;
		A[6][7] += a16*a17;
		A[6][8] += a16*a18;
		A[7][7] += a17*a17;
		A[7][8] += a17*a18;
		A[8][8] += a18*a18;

		double a20 = -worldx*imagey;
		double a21 = -worldy*imagey;
		double a22 = -imagey;
		double a23 = worldx*imagex;
		double a24 = worldy*imagex;
		double a25 = imagex;

		A[0][0] += a20*a20;
		A[0][1] += a20*a21;
		A[0][2] += a20*a22;
		A[0][3] += a20*a23;
		A[0][4] += a20*a24;
		A[0][5] += a20*a25;
		A[1][1] += a21*a21;
		A[1][2] += a21*a22;
		A[1][3] += a21*a23;
		A[1][4] += a21*a24;
		A[1][5] += a21*a25;
		A[2][2] += a22*a22;
		A[2][3] += a22*a23;
		A[2][4] += a22*a24;
		A[2][5] += a22*a25;
		A[3][3] += a23*a23;
		A[3][4] += a23*a24;
		A[3][5] += a23*a25;
		A[4][4] += a24*a24;
		A[4][5] += a24*a25;
		A[5][5] += a25*a25;

		computed = false;
	}

	inline void compute() {
		if (computed) {
			return;
		}

		// make symmetric
		for (int i = 0; i < 9; ++i)
			for (int j = i+1; j < 9; ++j) {
				A[j][i] = A[i][j];
			}

		helper::nullvector(9,9,A[0],H[0]);
		computed = true;
	}

	inline void project(double worldx, double worldy, double &ix, double &iy) {
		compute();

		ix = H[0][0]*worldx + H[0][1]*worldy + H[0][2];
		iy = H[1][0]*worldx + H[1][1]*worldy + H[1][2];
		double z = H[2][0]*worldx + H[2][1]*worldy + H[2][2];
		ix = ix/z + cx;
		iy = iy/z + cy;
	}
};//end of Homography33

struct Homography33b
{
    double A[9][9];
    double H[3][3]; // homography, once computed.
	bool computed;

    // worldx, worldy, imagex, imagey
	std::vector<cv::Vec4d> correspondences;

    Normalize2D normWorld;
    Normalize2D normImage;

    Homography33b() : computed(false) {
		helper::zeros(9,9,A[0]);
		helper::zeros(3,3,H[0]);
	}
	
	inline void reset() {
		helper::zeros(9,9,A[0]);
		helper::zeros(3,3,H[0]);
		computed = false;
	}

    inline void getH(double h[3][3]) {
		compute();
		std::copy((double*)H[0], (double*)H[0]+9, (double*)h[0]);
	}

    inline void addCorrespondence(double worldx, double worldy, double imagex, double imagey)
    {
        correspondences.push_back(cv::Vec4d(worldx, worldy, imagex, imagey));
        normWorld.add(worldx, worldy);
        normImage.add(imagex, imagey);
        computed=false;
    }

    inline void compute()
    {
        if (computed) // already computed?
            return;

		//make sure last row is [0 0 1]
        double normWorldT[3][3];
		normWorld.getTransform(normWorldT);
        double normImageT[3][3];
		normImage.getTransform(normImageT);
		double iNormImageT[3][3];
		normImage.getInverseTransform(iNormImageT);

        // Would it be better to compute the Nx9 matrix and compute
        // the economy SVD of that? We'd have a smaller condition
        // number, but a slower SVD.
        for (int i=0; i<(int)correspondences.size(); ++i) {
			const cv::Vec4d& corr=correspondences[i];

			//assume last row is [0 0 1] in normWorldT and normImageT
            double worldx = corr[0]*normWorldT[0][0]+normWorldT[0][2];
            double worldy = corr[1]*normWorldT[1][1]+normWorldT[1][2];
            double imagex = corr[2]*normImageT[0][0]+normImageT[0][2];
            double imagey = corr[3]*normImageT[1][1]+normImageT[1][2];

            // only update upper-right. A'A is symmetric, we'll finish the lower left later.
            double a03 = -worldx;
            double a04 = -worldy;
            double a05 = -1;
            double a06 = worldx*imagey;
            double a07 = worldy*imagey;
            double a08 = imagey;

            A[3][3] += a03*a03;
            A[3][4] += a03*a04;
            A[3][5] += a03*a05;
            A[3][6] += a03*a06;
            A[3][7] += a03*a07;
            A[3][8] += a03*a08;
            A[4][4] += a04*a04;
            A[4][5] += a04*a05;
            A[4][6] += a04*a06;
            A[4][7] += a04*a07;
            A[4][8] += a04*a08;
            A[5][5] += a05*a05;
            A[5][6] += a05*a06;
            A[5][7] += a05*a07;
            A[5][8] += a05*a08;
            A[6][6] += a06*a06;
            A[6][7] += a06*a07;
            A[6][8] += a06*a08;
            A[7][7] += a07*a07;
            A[7][8] += a07*a08;
            A[8][8] += a08*a08;

            double a10 = worldx;
            double a11 = worldy;
            double a12 = 1;
            double a16 = -worldx*imagex;
            double a17 = -worldy*imagex;
            double a18 = -imagex;

            A[0][0] += a10*a10;
            A[0][1] += a10*a11;
            A[0][2] += a10*a12;
            A[0][6] += a10*a16;
            A[0][7] += a10*a17;
            A[0][8] += a10*a18;
            A[1][1] += a11*a11;
            A[1][2] += a11*a12;
            A[1][6] += a11*a16;
            A[1][7] += a11*a17;
            A[1][8] += a11*a18;
            A[2][2] += a12*a12;
            A[2][6] += a12*a16;
            A[2][7] += a12*a17;
            A[2][8] += a12*a18;
            A[6][6] += a16*a16;
            A[6][7] += a16*a17;
            A[6][8] += a16*a18;
            A[7][7] += a17*a17;
            A[7][8] += a17*a18;
            A[8][8] += a18*a18;

            double a20 = -worldx*imagey;
            double a21 = -worldy*imagey;
            double a22 = -imagey;
            double a23 = worldx*imagex;
            double a24 = worldy*imagex;
            double a25 = imagex;

            A[0][0] += a20*a20;
            A[0][1] += a20*a21;
            A[0][2] += a20*a22;
            A[0][3] += a20*a23;
            A[0][4] += a20*a24;
            A[0][5] += a20*a25;
            A[1][1] += a21*a21;
            A[1][2] += a21*a22;
            A[1][3] += a21*a23;
            A[1][4] += a21*a24;
            A[1][5] += a21*a25;
            A[2][2] += a22*a22;
            A[2][3] += a22*a23;
            A[2][4] += a22*a24;
            A[2][5] += a22*a25;
            A[3][3] += a23*a23;
            A[3][4] += a23*a24;
            A[3][5] += a23*a25;
            A[4][4] += a24*a24;
            A[4][5] += a24*a25;
            A[5][5] += a25*a25;
        }

        // make symmetric
		for (int i = 0; i < 9; ++i)
			for (int j = i+1; j < 9; ++j) {
				A[j][i] = A[i][j];
			}

		helper::nullvector(9,9,A[0],H[0]);
		double tmp[3][3];
		helper::mul(3,3,3,3,iNormImageT[0],H[0],tmp[0]);
		helper::mul(3,3,3,3,tmp[0],normWorldT[0],H[0]);
		computed = true;
    }

    void project(double worldx, double worldy, double& ix, double& iy)
    {
        compute();

		double zi = 1.0/(H[2][0]*worldx + H[2][1]*worldy + H[2][2]);
        ix = (H[0][0]*worldx + H[0][1]*worldy + H[0][2])*zi;
        iy = (H[1][0]*worldx + H[1][1]*worldy + H[1][2])*zi;        
    }
};//end of Homography33b

/** Represents four segments that form a loop, and might be a tag. **/
struct Quad {
	// points for the quad (in pixel coordinates), in counter
	// clockwise order. These points are the intersections of
	// segments.
	double p[4][2];

	// The total length (in pixels) of the actual perimeter
	// observed for the quad. This is in contrast to the geometric
	// perimeter, some of which may not have been directly
	// observed but rather inferred by intersecting
	// segments. Quads with more observed perimeter are preferred
	// over others.
	double observedPerimeter;

	// Given that the whole quad spans from (0,0) to (1,1) in
	// "quad space", compute the pixel coordinates for a given
	// point within that quad. Note that for most of the Quad's
	// existence, we will not know the correct orientation of the
	// tag.
	Homography33b homography;

	Quad() {}

	/** (x,y) are the optical center of the camera, which is
	 * needed to correctly compute the homography.
	 **/
	Quad(const double p[4][2]) {
		std::copy((double*)p[0], (double*)p[0]+8, (double*)this->p[0]);

		homography.addCorrespondence(-1, -1, p[0][0], p[0][1]);
		homography.addCorrespondence( 1, -1, p[1][0], p[1][1]);
		homography.addCorrespondence( 1,  1, p[2][0], p[2][1]);
		homography.addCorrespondence(-1,  1, p[3][0], p[3][1]);
	}

	inline void reset(const double p[4][2]) {
		std::copy(p[0], p[0]+8, this->p[0]);
		homography.reset();
		homography.addCorrespondence(-1, -1, p[0][0], p[0][1]);
		homography.addCorrespondence( 1, -1, p[1][0], p[1][1]);
		homography.addCorrespondence( 1,  1, p[2][0], p[2][1]);
		homography.addCorrespondence(-1,  1, p[3][0], p[3][1]);
	}

	// Same as interpolate, except that the coordinates are
	// interpreted between 0 and 1, instead of -1 and 1.
	inline void interpolate01(double x, double y, double &xo, double &yo) {
		interpolate(2*x - 1, 2*y - 1, xo, yo);
	}

	// interpolate given that the lower left corner of the lower
	// left cell is at (-1,-1) and the upper right corner of
	// the upper right cell is at (1,1)
	inline void interpolate(double x, double y, double &xo, double &yo) {
		homography.project(x, y, xo, yo);
	}
};//end of Quad

/** Fits a grayscale model over an area of the form:
    Ax + By + Cxy + D = value

    We use this model to compute spatially-varying thresholds for
    reading bits.
**/
struct GrayModel {
	// we're solving Ax = b. For each observation, we add a row to
	// A of the form [x y xy 1] and to be of the form [gray]. x is
	// the vector [A B C D].
	//
	// The least-squares solution to the system is x = inv(A'A)A'b
	double A[4][4]; // The A'A matrix
	double b[4];    // The A'b matrix
	double X[4]; // our solution, [A B C D]
	bool computed;

	int nobs; // how many observations?

	GrayModel() {
		computed=false;
		helper::zeros(4,4,A[0]);
		helper::zeros(4,1,b);
		helper::zeros(4,1,X);
		nobs = 0; // !!!!!!! need to init!
	}

	inline void addObservation(double x, double y, double gray) {
		double xy = x*y;

		// update only upper-right elements. A'A is symmetric,
		// we'll fill the other elements in later.
		A[0][0] += x*x;
		A[0][1] += x*y;
		A[0][2] += x*xy;
		A[0][3] += x;
		A[1][1] += y*y;
		A[1][2] += y*xy;
		A[1][3] += y;
		A[2][2] += xy*xy;
		A[2][3] += xy;
		A[3][3] += 1;

		b[0] += x*gray;
		b[1] += y*gray;
		b[2] += xy*gray;
		b[3] += gray;

		nobs++;
		computed=false; // force a new solution to be computed.
	}

	inline int getNumObservations() const {
		return nobs;
	}

	inline void compute() {
		if (computed) { // already computed?
			return;
		}

		bool solved = false;
		if (nobs >= 6) {
			// we really only need 4 linearly independent
			// observations to fit our answer, but we'll be very
			// sensitive to noise if we don't have an
			// over-determined system. Thus, require at least 6
			// observations (or we'll use a constant model below).

			// make symmetric
			for (int i = 0; i < 4; i++)
				for (int j = i+1; j < 4; j++) {
					A[j][i] = A[i][j];
				}

			solved = helper::solve(4,4, A[0], b, X);
		}

		if (!solved) {
			// not enough samples to fit a good model. Use a flat model.
			X[3] = b[3] / nobs;
		}
		computed = true;
	}

	inline double interpolate(double x, double y) {
		compute();
		return X[0]*x + X[1]*y + X[2]*x*y + X[3];
	}
};//end of GrayModel

}//end of tag
}//end of april
