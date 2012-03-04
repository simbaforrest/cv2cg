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

/* TagDetector.hpp
	modified from april/tag/TagDetector.java
	git://april.eecs.umich.edu/home/git/april.git
*/

#include <iostream>
#include <string>
#include <map>
//opencv include
#include "OpenCVHelper.h"
#include "Log.h"

#include "TagUtils.hpp"

#ifndef TAG_DEBUG_DRAW
	#define TAG_DEBUG_DRAW 0
#endif

#ifndef TAG_DEBUG_PERFORMANCE
	#define TAG_DEBUG_PERFORMANCE 0
#endif

namespace april
{
namespace tag
{

using cv::Ptr;
using cv::Mat;
using std::vector;
using std::map;
using SearchHelper::Gridder;

struct TagDetector {
	Ptr<TagFamily> tagFamily;

	/** Gaussian smoothing kernel applied to image (0 == no filter)
	 * used when sampling bits. Filtering is a good idea in cases
	 * where A) a cheap camera is introducing artifical sharpening, B)
	 * the bayer pattern is creating artifcats, C) the sensor is very
	 * noisy and/or has hot/cold pixels. However, filtering makes it
	 * harder to decode very small tags. Reasonable values are 0, or
	 * [0.8, 1.5].**/
	double sigma;

	/** Gaussian smoothing kernel applied to image (0 == no filter)
	 * used when detecting the outline of the box. It is almost always
	 * useful to have some filtering, since the loss of small details
	 * won't hurt. Recommended value = 0.8. The case where sigma ==
	 * segsigma has been optimized to avoid a redundant filter
	 * operation. **/
	double segSigma;

	/** Instead of blurring the input image before segmentation, we
	 * can achieve similar effects by decimating the image by a factor
	 * of two. When enabled, this option applies a block LPF filter of
	 * width 2, then decimates the image. With this option, not only
	 * can we safely set segSigma = 0, but the slowest part of the
	 * algorithm (the segmentation) runs about 4 times faster. The
	 * downside is that the position of the targets is determined
	 * based on the segmentation: lower resolution will result in more
	 * localization error. However, the effect on quality is quite
	 * modest, and this optimization is generally recommended (along
	 * with segSigma = 0). If segSigma is non-zero, the filtering by
	 * segSigma occurs first, followed by the block LPF, and the
	 * decimation. **/
	bool segDecimate;

	/** Do not consider pixels whose gradient magnitude is less than
	 * minMag. Small values make the detector more sensitive, but also
	 * force us to consider many more edges resulting in slower
	 * computation time. A value of 0.001 is very sensitive. A value
	 * of 0.01 is quite fast. **/
	double minMag;

	/** When connecting edges, what is the maximum range allowed for
	 * the gradient directions? in radian**/
	double maxEdgeCost;

	/** When growing components, the intra component variation is
	 * allowed to grow when the component is small in size. These two
	 * thresholds determine how much. **/
	double thetaThresh;
	double magThresh;

	// in pixels. Set based on minimum plausible decoding size for Tag9 family.
	double minimumLineLength;

	// minimum number of pixels in a segment before we'll fit a line to it.
	double minimumSegmentSize;

	// minimum size of tag (in pixels) as measured along edges and diagonals.
	double minimumTagSize;

	/** Early pruning of quads which have insane aspect ratios. **/
	double maxQuadAspectRatio;

#if TAG_DEBUG_DRAW
	/** Produce debugging output. If the debugging code annoys you (or
	 * makes porting harder) you can delete all of the code in an
	 * if(debug) block.
	 **/
	Mat debugSegmentation;  // segmented image
	Mat debugTheta, debugMag;
#endif
#if TAG_DEBUG_PERFORMANCE
	double steptime[9];
#endif

	/** The optical center of the current frame, which is needed to correctly compute the homography. **/
	double opticalCenter[2];

	/** During segmentation, the weight of an edge is related to the
	 * change in theta between the two pixels. This change is
	 * normalized via maxEdgeCost, resulting in a number[0,1]. We then
	 * convert this number to fixed-point by multiplying it by
	 * WEIGHT_SCALE. The resulting number must fit in the number of
	 * bits allocated to it, currently 16 (see Step 3). Large values
	 * of WEIGHT_SCALE are good because they lead to better
	 * fixed-point approximations. However, small numbers are better,
	 * because there are fewer discrete values of edge costs, which
	 * means that there is more spatial/temporal coherence when
	 * processing the sorted edges. This results in faster
	 * processing. Given that these orientations are pretty noisy to
	 * begin with, some quantization error is acceptable.
	 **/
	int WEIGHT_SCALE;

	TagDetector(Ptr<TagFamily> tagFamily) {
		this->tagFamily = tagFamily;
		sigma = 0;
		segSigma = 0.8;
		segDecimate = false;
		minMag = 0.004;
		maxEdgeCost = helper::deg2rad(30);
		thetaThresh = 100;
		magThresh = 1200;
		minimumLineLength = 4;
		minimumSegmentSize = 4;
		minimumTagSize = 6;
		maxQuadAspectRatio = 32;
		WEIGHT_SCALE = 100; //10000;
	}

	int edgeCost(double theta0, double mag0, double theta1, double mag1) {
		if(mag0 < minMag || mag1 < minMag) {
			return -1;
		}

		double thetaErr = std::abs(helper::mod2pi(theta1 - theta0));
		if (thetaErr > maxEdgeCost) {
			return -1;
		}

		double normErr = thetaErr / maxEdgeCost;

		return (int) (normErr * WEIGHT_SCALE);
	}

	// lousy approximation of arctan function, but good enough for our purposes (about 4 degrees)
	static double fast_atan2(double y, double x) {
		double coeff_1 = CV_PI/4;
		double coeff_2 = 3*coeff_1;
		double abs_y = std::abs(y)+1e-10;      // kludge to prevent 0/0 condition

		double angle;

		if (x >= 0) {
			double r = (x - abs_y) / (x + abs_y);
			angle = coeff_1 - coeff_1 * r;
		} else {
			double r = (x + abs_y) / (abs_y - x);
			angle = coeff_2 - coeff_1 * r;
		}

		if (y < 0) {
			return -angle;    // negate if in quad III or IV
		} else {
			return angle;
		}
	}

	/** Sort and return the first vlength values in v[] by the value
	 * of v[i]&mask. The maximum value in the array 'v' is maxv
	 * (if maxv is negative, maxv will be found). These weights must
	 * be small enough to fit in an integer. This implementation is
	 * stable.
	 **/
	static void countingSortLongArray(vector<INT64>& v, int vlength, int maxv, INT64 mask) {
		if (maxv < 0) {
			for (int i = 0; i < vlength; i++) {
				maxv = std::max(maxv, (int) (v[i]&mask));
			}
		}

		// For weight 'w', counts[w] will give the output position for
		// the next sample with weight w.  To build this, we begin by
		// counting how many samples there are with each weight. (Note
		// that the initial position for weight w is only affected by
		// the number of weights less than w, hence the +1 in
		// counts[w+1].
		vector<int> counts(maxv+2, 0);

		for (int i = 0; i < vlength; i++) {
			int w = (int) (v[i]&mask);
			counts[w+1]++;
		}

		// accumulate.
		for (int i = 1; i < (int)counts.size(); i++) {
			counts[i] += counts[i-1];
		}

		vector<INT64> newv(vlength, 0);
		for (int i = 0; i < vlength; i++) {
			int w = (int) (v[i]&mask);
			newv[counts[w]] = v[i];
			counts[w]++;
		}

		v.swap(newv);
	}

	/** Detect the features in the specified image. We need the
	 * optical center, but it is usually fine to pass in (width/2,
	 * height/2).
	 **/
	void process(Mat im, double opticalCenter[2], vector<TagDetection>& goodDetections) {
		this->opticalCenter[0] = opticalCenter[0];
		this->opticalCenter[1] = opticalCenter[1];

		// This is a very long function, but it can't really be
		// factored any more simply: it's just a long sequence of
		// sequential operations.
#if TAG_DEBUG_PERFORMANCE
		helper::PerformanceMeasurer PM;
		PM.scale = 1000;
		PM.tic();
#endif
		///////////////////////////////////////////////////////////
		// Step one. Preprocess image (convert to float (grayscale) [0,1]
		// and low pass if necessary.)
		Mat fimOrig;
		helper::green2float(im, fimOrig);

		Mat fim = fimOrig;
		if (sigma > 0) {
			int filtsz = ((int) std::max(3.0, 3*sigma)) | 1;
			GaussianBlur(fimOrig, fim, cv::Size(filtsz,filtsz), sigma);
		}
#if TAG_DEBUG_PERFORMANCE
		steptime[0] = PM.toctic();
#endif
#if TAG_DEBUG_DRAW
		{
			std::string win = "fim";
			cv::namedWindow(win);
			cv::imshow(win, fim);
		}
#endif
		///////////////////////////////////////////////////////////
		// Step two. For each pixel, compute the local gradient. We
		// store the direction and magnitude.

		// This step is quite sensitive to noise, since a few bad
		// theta estimates will break up segments, causing us to miss
		// quads. It is helpful to do a Gaussian low-pass on this step
		// even if we don't want it for decoding.
		Mat fimseg = fimOrig; // default
		if (segSigma > 0) {
			if (segSigma == sigma) {
				// reuse the already-filtered image...
				fimseg = fim;
			} else {
				// blur anew.
				int filtsz = ((int) std::max(3.0, 3*segSigma)) | 1;
				GaussianBlur(fimOrig, fimseg, cv::Size(filtsz,filtsz), segSigma);
			}
		}
		if (segDecimate) {
			Mat tmp;
			pyrDown(fimseg, tmp);
			fimseg = tmp;
		}

		Mat fimTheta(fimseg.size(), fimseg.type());
		Mat fimMag(fimseg.size(), fimseg.type());

		for (int y = 1; y+1 < fimseg.rows; y++) {
			for (int x = 1; x+1 < fimseg.cols; x++) {

				float Ix = fimseg.at<float>(y, x+1) - fimseg.at<float>(y, x-1);
				float Iy = fimseg.at<float>(y+1, x) - fimseg.at<float>(y-1, x);

				fimMag.at<float>(y,x) = Ix*Ix + Iy*Iy;
				fimTheta.at<float>(y,x) = fast_atan2(Iy, Ix);
			}
		}
#if TAG_DEBUG_PERFORMANCE
		steptime[1] = PM.toctic();
#endif
#if TAG_DEBUG_DRAW
		{
			debugTheta = Mat::zeros(fimseg.size(), CV_32FC1);
			debugMag = Mat::zeros(fimseg.size(), CV_32FC1);
			cv::normalize(fimTheta, debugTheta, 0, 1, cv::NORM_MINMAX);
			cv::normalize(fimMag, debugMag, 0, 1, cv::NORM_MINMAX);
			std::string win = "fimTheta";
			cv::namedWindow(win);
			cv::imshow(win, debugTheta);
			std::string win2 = "fimMag";
			cv::namedWindow(win2);
			cv::imshow(win2, debugMag);
		}
#endif
		///////////////////////////////////////////////////////////
		// Step three. Segment the edges, grouping pixels with similar
		// thetas together. This is a greedy algorithm: we start with
		// the most similar pixels.  We use 4-connectivity.
		helper::UnionFind uf(fimseg.cols*fimseg.rows);
		{
			int width = fimseg.cols;
			int height = fimseg.rows;

			vector<INT64> edges(width*height*4, 0);
			int nedges = 0;

			// for efficiency, each edge is encoded as a single
			// long. The constants below are used to pack/unpack the
			// long.
			INT64 IDA_SHIFT = 40, IDB_SHIFT = 16, INDEX_MASK = (((INT64)1)<<24) - 1, WEIGHT_MASK=(((INT64)1)<<16)-1;

			// bounds on the thetas assigned to this group. Note that
			// because theta is periodic, these are defined such that the
			// average value is contained *within* the interval.
			vector<double> tmin(width*height, 0);
			vector<double> tmax(width*height, 0);

			vector<double> mmin(width*height, 0);
			vector<double> mmax(width*height, 0);

			for (int y = 0; y+1 < fimseg.rows; y++) {
				for (int x = 0; x+1 < fimseg.cols; x++) {

					double mag0 = fimMag.at<float>(y,x);
					if (mag0 < minMag) {
						continue;
					}
					mmax[y *width+x] = mag0;
					mmin[y *width+x] = mag0;

					double theta0 = fimTheta.at<float>(y,x);
					tmin[y *width+x] = theta0;
					tmax[y *width+x] = theta0;

					int edgecost;

					edgecost = edgeCost(theta0, mag0, fimTheta.at<float>(y,x+1), fimMag.at<float>(y,x+1));
					if (edgecost >= 0) {
						edges[nedges++] = (((INT64) y*width+x)<<IDA_SHIFT) + (((INT64) y*width+x+1)<<IDB_SHIFT) + edgecost;
					}

					edgecost = edgeCost(theta0, mag0, fimTheta.at<float>(y+1,x), fimMag.at<float>(y+1,x));
					if (edgecost >= 0) {
						edges[nedges++] = (((INT64) y*width+x)<<IDA_SHIFT) + (((INT64) (y+1)*width+x)<<IDB_SHIFT) + edgecost;
					}

					edgecost = edgeCost(theta0, mag0, fimTheta.at<float>(y+1,x+1), fimMag.at<float>(y+1,x+1));
					if (edgecost >= 0) {
						edges[nedges++] = (((INT64) y*width+x)<<IDA_SHIFT) + (((INT64) (y+1)*width+x+1)<<IDB_SHIFT) + edgecost;
					}

					edgecost = (x == 0) ? -1 : edgeCost(theta0, mag0, fimTheta.at<float>(y+1,x-1), fimMag.at<float>(y+1,x-1));
					if (edgecost >= 0) {
						edges[nedges++] = (((INT64) y*width+x)<<IDA_SHIFT) + (((INT64) (y+1)*width+x-1)<<IDB_SHIFT) + edgecost;
					}

					// XXX Would 8 connectivity help for rotated tags?
					// (Probably not much, so long as input filtering
					// hasn't been disabled.)
				}
			}
			// sort those edges by weight (lowest weight first).
			countingSortLongArray(edges, nedges, -1, WEIGHT_MASK);
			// process edges in order of increasing weight, merging
			// clusters if we can do so without exceeding the
			// thetaThresh.
			for (int i = 0; i < nedges; i++) {
				int ida = (int) ((edges[i]>>IDA_SHIFT)&INDEX_MASK);
				int idb = (int) ((edges[i]>>IDB_SHIFT)&INDEX_MASK);

				ida = uf.Find(ida);
				idb = uf.Find(idb);

				if (ida == idb) {
					continue;
				}

				int sza = uf.ClassSize(ida);
				int szb = uf.ClassSize(idb);

				double tmina = tmin[ida], tmaxa = tmax[ida];
				double tminb = tmin[idb], tmaxb = tmax[idb];

				double costa = (tmaxa-tmina);
				double costb = (tmaxb-tminb);

				// bshift will be a multiple of 2pi that aligns the spans
				// of b with a so that we can properly take the union of
				// them.
				double bshift = helper::mod2pi((tmina+tmaxa)/2, (tminb+tmaxb)/2) - (tminb+tmaxb)/2;

				double tminab = std::min(tmina, tminb + bshift);
				double tmaxab = std::max(tmaxa, tmaxb + bshift);

				if (tmaxab - tminab > 2*CV_PI) { // corner case that's probably not useful to handle correctly. oh well.
					tmaxab = tminab + 2*CV_PI;
				}

				double mmaxab = std::max(mmax[ida], mmax[idb]);
				double mminab = std::min(mmin[ida], mmin[idb]);

				// merge these two clusters?
				double costab = (tmaxab - tminab);
				if (costab <= (std::min(costa, costb) + thetaThresh/(sza+szb)) &&
				        (mmaxab-mminab) <= std::min(mmax[ida]-mmin[ida], mmax[idb]-mmin[idb]) + magThresh/(sza+szb)) {

					int idab = uf.Union(ida, idb);

					tmin[idab] = tminab;
					tmax[idab] = tmaxab;

					mmin[idab] = mminab;
					mmax[idab] = mmaxab;
				}
			}
		}
#if TAG_DEBUG_PERFORMANCE
		steptime[2] = PM.toctic();
#endif
		///////////////////////////////////////////////////////////
		// Step four. Loop over the pixels again, collecting
		// statistics for each cluster. We will soon fit lines to
		// these points.
#if TAG_DEBUG_DRAW
		debugSegmentation = Mat::zeros(fimseg.size(), CV_8UC3);
#endif
		typedef cv::Vec3d Pixel;
		typedef vector<Pixel> PixelList;
		typedef map<int, PixelList> PixelCluster;
		PixelCluster clusters;
		for (int y = 0; y+1 < fimseg.rows; y++) {
			for (int x = 0; x+1 < fimseg.cols; x++) {
				int pid = y*fimseg.cols+x;
				if (uf.ClassSize(pid) < minimumSegmentSize) {
					continue;
				}

				int rep = uf.Find(pid);
#if TAG_DEBUG_DRAW
				cv::Vec3b& pix = debugSegmentation.at<cv::Vec3b>(y, x);
				pix[2]=(char)(rep&0xFF0000)>>16;
				pix[1]=(char)(rep&0x00FF00)>>8;
				pix[0]=(char)rep&0x0000FF;
#endif
				PixelCluster::iterator itr = clusters.find(rep);
				if (itr == clusters.end()) {
					std::pair<PixelCluster::iterator,bool> tmp =
					    clusters.insert(PixelCluster::value_type(rep,PixelList()));
					itr = tmp.first;
				}

				itr->second.push_back(Pixel(x,y,fimMag.at<float>(y,x)));
			}
		}
#if TAG_DEBUG_PERFORMANCE
		steptime[3] = PM.toctic();
		loglnd(">>> clusters.size()="<<clusters.size());
#endif
		///////////////////////////////////////////////////////////
		// Step five. Loop over the clusters, fitting lines (which we
		// call Segments).
		vector<Segment> segments;
		segments.reserve(300);

		PixelCluster::iterator itr = clusters.begin();
		for (; itr!=clusters.end(); ++itr) {
			PixelList &points = itr->second;
			Segment seg;
			seg.fitBy(points);

			// filter short lines
			if (seg.length < minimumLineLength) {
				continue;
			}

			double dy = seg.y1-seg.y0;
			double dx = seg.x1-seg.x0;
			seg.theta = atan2(dy,dx);

			// We add an extra semantic to segments: the vector
			// p1->p2 will have dark on the left, white on the right.
			// To do this, we'll look at every gradient and each one
			// will vote for which way they think the gradient should
			// go. (This is way more retentive than necessary: we
			// could probably sample just one point!)
			double flip = 0, noflip = 0;
			for (int i=0; i<(int)points.size(); ++i) {
				Pixel &xyw = points[i];
				double theta = fimTheta.at<float>((int) xyw[1], (int) xyw[0]);
				double mag = fimMag.at<float>((int) xyw[1], (int) xyw[0]);

				// err *should* be +Math.PI/2 for the correct winding,
				// but if we've got the wrong winding, it'll be around
				// -Math.PI/2.
				double err = helper::mod2pi(theta - seg.theta);

				if (err < 0) {
					noflip += mag;
				} else {
					flip += mag;
				}
			}

			if (flip > noflip) {
				seg.theta += CV_PI;
			}

			double dot = dx*cos(seg.theta) + dy*sin(seg.theta);
			if (dot > 0) {
				seg.swap();
			}

			if (segDecimate) {
				seg.x0 = 2*seg.x0 + .5;
				seg.y0 = 2*seg.y0 + .5;
				seg.x1 = 2*seg.x1 + .5;
				seg.y1 = 2*seg.y1 + .5;
				seg.length *= 2;
			}

#if TAG_DEBUG_DRAW
			double cx = (seg.x0 + seg.x1)/2, cy = (seg.y0 + seg.y1)/2;
			double notch = std::max(2.0, 0.1*seg.length);
			cv::Scalar co(rand()%255, rand()%255, rand()%255);
			line(debugSegmentation, cv::Point(seg.x0,seg.y0), cv::Point(seg.x1,seg.y1), co);
			line(debugSegmentation, cv::Point(cx,cy), cv::Point(cx+notch*sin(seg.theta),cy-notch*cos(seg.theta)), co);
			circle(debugSegmentation, cv::Point(seg.x0,seg.y0), 2, co, -1);
#endif

			segments.push_back(seg);
		}
		segments.resize(segments.size());

		int width = fim.cols, height = fim.rows;

#if TAG_DEBUG_PERFORMANCE
		loglnd(">>> segments.size()="<<segments.size());
		steptime[4] = PM.toctic();
#endif
		////////////////////////////////////////////////////////////////
		// Step six. For each segment, find segments that begin where
		// this segment ends. (We will chain segments together
		// next...) The gridder accelerates the search by building
		// (essentially) a 2D hash table.
		Gridder<Segment> gridder(0, 0, width, height, 10);

		// add every segment to the hash table according to the
		// position of the segment's first point. (Remember that the
		// first point has a specific meaning due to our left-hand
		// rule above.)
		for(int i=0; i<(int)segments.size(); ++i) {
			Segment &seg = segments[i];
			gridder.add(seg.x0,seg.y0,&seg);
		}

		// Now, find child segments that begin where each parent
		// segments ends.
		for(int i=0; i<(int)segments.size(); ++i) {
			Segment &parent = segments[i];

			Gridder<Segment>::Iterator itr = gridder.find(parent.x1, parent.y1, 0.5*parent.length);
			for (; !itr.done(); itr.next()) {
				Segment &child = *(itr.get());
				// require child to have the right handedness...
				if (helper::mod2pi(child.theta - parent.theta) > 0) {
					continue;
				}

				// compute intersection of points.
				double px, py;
				if( !parent.intersectionWith(child,px,py) ) {
					continue;
				}

				if( std::max(distance(px,py, parent.x1, parent.y1),
				             distance(px,py, child.x0 , child.y0 ) ) > parent.length ) {
					continue;
				}

				// everything's okay, this child is a reasonable successor.
				parent.children.push_back(&child);
			}
		}
#if TAG_DEBUG_PERFORMANCE
		steptime[5] = PM.toctic();
#endif
		////////////////////////////////////////////////////////////////
		// Step seven. Search all connected segments to see if any
		// form a loop of length 4. Add those to the quads list.
		vector<Quad> quads;
		{
			Segment *tmp[5];
			for(int i=0; i<(int)segments.size(); ++i) {
				Segment *segptr = &(segments[i]);
				tmp[0] = segptr;
				search(quads, tmp, segptr, 0);
			}
		}

#if TAG_DEBUG_DRAW
		for(int i=0; i<(int)quads.size(); ++i) {
			Quad& q = quads[i];
			cv::Point p0(q.p[0][0], q.p[0][1]);
			cv::Point p1(q.p[1][0], q.p[1][1]);
			cv::Point p2(q.p[2][0], q.p[2][1]);
			cv::Point p3(q.p[3][0], q.p[3][1]);
			cv::Scalar co = helper::CV_RG;
			line(debugSegmentation, p0, p1, co, 2);
			line(debugSegmentation, p1, p2, co, 2);
			line(debugSegmentation, p2, p3, co, 2);
			line(debugSegmentation, p3, p0, co, 2);
		}
#endif
#if TAG_DEBUG_PERFORMANCE
		loglnd(">>> quads.size()="<<quads.size());
		steptime[6] = PM.toctic();
#endif
		////////////////////////////////////////////////////////////////
		// Step eight. Decode the quads. For each quad, we first
		// estimate a threshold color to decided between 0 and
		// 1. Then, we read off the bits and see if they make sense.
		vector<TagDetection> detections;

		for(int i=0; i<(int)quads.size(); ++i) {
			Quad &quad = quads[i];
			// Find a threshold
			GrayModel blackModel;
			GrayModel whiteModel;

			// sample points around the black and white border in
			// order to calibrate our gray threshold. This code is
			// simpler if we loop over the whole rectangle and discard
			// the points we don't want.
			int dd = 2*tagFamily->blackBorder + tagFamily->d;

			for (int iy = -1; iy <= dd; iy++) {
				for (int ix = -1; ix <= dd; ix++) {
					double y = (iy + .5) / dd;
					double x = (ix + .5) / dd;

					double px, py;
					quad.interpolate01(x, y, px, py);
					int irx = (int) (px+.5);
					int iry = (int) (py+.5);

					if (irx < 0 || irx >= width || iry < 0 || iry >= height) {
						continue;
					}

					float v = fim.at<float>(iry, irx);

					if ((iy == -1 || iy == dd) || (ix == -1 || ix == dd)) {
						// part of the outer white border.
						whiteModel.addObservation(x, y, v);
#if TAG_DEBUG_DRAW
						circle(debugSegmentation, cv::Point(px,py), 2, helper::CV_BLUE, 2, -1);
#endif
					} else if ((iy == 0 || iy == (dd-1)) || (ix == 0 || ix == (dd-1))) {
						// part of the outer black border.
						blackModel.addObservation(x, y, v);
#if TAG_DEBUG_DRAW
						circle(debugSegmentation, cv::Point(px,py), 2, helper::CV_BLUE, 2, -1);
#endif
					}
				}
			}

			bool bad = false;
			INT64 tagCode = 0;

			// Try reading off the bits.
			// XXX: todo: multiple samples within each cell and vote?
			for (int iy = tagFamily->d-1; iy >= 0; iy--) {
				for (int ix = 0; ix < tagFamily->d; ix++) {
					double y = (tagFamily->blackBorder + iy + .5) / dd;
					double x = (tagFamily->blackBorder + ix + .5) / dd;

					double px, py;
					quad.interpolate01(x, y, px, py);
					int irx = (int) (px+.5);
					int iry = (int) (py+.5);

					if (irx < 0 || irx >= width || iry < 0 || iry >= height) {
						bad = true;
						continue;
					}

					double threshold = (blackModel.interpolate(x, y) + whiteModel.interpolate(x,y))*.5;

#if TAG_DEBUG_DRAW
					circle(debugSegmentation, cv::Point(px,py), 2, helper::CV_RG, 2, -1);
#endif

					float v = fim.at<float>(iry, irx);

					tagCode = tagCode << 1;
					if (v > threshold) {
						tagCode |= 1;
					}
				}
			}

			if (!bad) {
				TagDetection d;
				tagFamily->decode(d, tagCode);

				// rotate points in detection according to decoded
				// orientation. Thus the order of the points in the
				// detection object can be used to determine the
				// orientation of the target.
				for (int i4 = 0; i4 < 4; i4++) {
					int id = (4+i4-d.rotation)%4;
					d.p[id][0] = quad.p[i4][0];
					d.p[id][1] = quad.p[i4][1];
				}

				// compute the homography (and rotate it appropriately)
				quad.homography.getH(d.homography);
				quad.homography.getCXY(d.hxy);

				{
					double c = cos(d.rotation*CV_PI/2.0);
					double s = sin(d.rotation*CV_PI/2.0);
					double R[3][3] = {{ c, -s, 0},
						{ s,  c, 0},
						{ 0,  0, 1}
					};
					double homo[3][3];
					helper::mul(3,3,3,3,d.homography[0],R[0],homo[0]);
					std::copy((double*)homo[0], (double*)homo[0]+9, (double*)d.homography[0]);
				}

				if (d.good) {
					quad.interpolate01(.5, .5, d.cxy[0], d.cxy[1]);
					d.observedPerimeter = quad.observedPerimeter;
					detections.push_back(d);
				}
			}
		}//end of quads[]

#if TAG_DEBUG_PERFORMANCE
		loglnd(">>> detections.size()="<<detections.size());
		steptime[7] = PM.toctic();
#endif
#if TAG_DEBUG_DRAW
		std::string win = "debugSegmentation";
		cv::namedWindow(win);
		cv::imshow(win, debugSegmentation);
#endif
		////////////////////////////////////////////////////////////////
		// Step nine. Some quads may be detected more than once, due
		// to partial occlusion and our aggressive attempts to recover
		// from broken lines. When two quads (with the same id)
		// overlap, we will keep the one with the lowest error, and if
		// the error is the same, the one with the greatest observed
		// perimeter.

		// NOTE: allow multiple (non-overlapping) detections of the same target.
		for(int i=0; i<(int)detections.size(); ++i) {
			TagDetection &d = detections[i];

			bool newFeature = true;

			for (int odidx = 0; odidx < (int)goodDetections.size(); odidx++) {
				TagDetection &od = goodDetections[odidx];

				if (d.id != od.id || !detectionsOverlapTooMuch(d, od)) {
					continue;
				}

				// there's a conflict. we must pick one to keep.
				newFeature = false;

				// this detection is worse than the previous one... just don't use it.
				if (d.hammingDistance > od.hammingDistance) {
					continue;
				}

				// otherwise, keep the new one if it either has
				// *lower* error, or has greater perimeter
				if (d.hammingDistance < od.hammingDistance || d.observedPerimeter > od.observedPerimeter) {
					od = d;
				}
			}

			if (newFeature) {
				goodDetections.push_back(d);
			}
		}
		////////////////////////////////////////////////////////////////
		// I thought it would never end. //simbaforrest: me too! ^_^
#if TAG_DEBUG_PERFORMANCE
		steptime[8] = PM.toctic();
		for(int i=0; i<9; ++i) {
			loglnd("[process] step "<<(i+1)<<" takes "<<steptime[i]<<" ms.");
		}
#endif
	}

	bool detectionsOverlapTooMuch(TagDetection &a, TagDetection &b) {
		// Compute a sort of "radius" of the two targets. We'll do
		// this by computing the average length of the edges of the
		// quads (in pixels).
		double radius = 0.0625*(distance(a.p[0], a.p[1]) +
		                        distance(a.p[1], a.p[2]) +
		                        distance(a.p[2], a.p[3]) +
		                        distance(a.p[3], a.p[0]) +
		                        distance(b.p[0], b.p[1]) +
		                        distance(b.p[1], b.p[2]) +
		                        distance(b.p[2], b.p[3]) +
		                        distance(b.p[3], b.p[0]));

		// distance (in pixels) between two tag centers.
		double d = distance(a.cxy, b.cxy);

		// reject pairs where the distance between centroids is
		// smaller than the "radius" of one of the tags.
		return (d < radius);
	}

	/** quads: any discovered quads will be added to this list.
	    path: The segments currently part of the search.
	    parent: The first segment in the quad.
	    depth: how deep in the search are we?
	**/
	void search(vector<Quad>& quads, Segment *path[5], Segment *parent, int depth) {
		// terminal depth occurs when we've found four segments.
		if (depth == 4) {
			// Is the first segment the same as the last segment (i.e., a loop?)
			if (path[4] == path[0]) {

				// the 4 corners of the quad as computed by the intersection of segments.
				double p[4][2]= {{0}};
				double observedPerimeter = 0;

				bool bad = false;
				for (int i = 0; i < 4; i++) {
					// compute intersections between all the
					// lines. This will give us sub-pixel accuracy for
					// the corners of the quad.
					if( !path[i]->intersectionWith(*path[i+1], p[i][0], p[i][1]) ) {
						bad = true;    // no intersection? Occurs when the lines are almost parallel.
					}

					observedPerimeter += path[i]->length;
				}

				// eliminate quads that don't form a simply connected
				// loop (i.e., those that form an hour glass, or wind
				// the wrong way.)
				if (!bad) {
					double t0 = atan2(p[1][1] - p[0][1], p[1][0] - p[0][0]);
					double t1 = atan2(p[2][1] - p[1][1], p[2][0] - p[1][0]);
					double t2 = atan2(p[3][1] - p[2][1], p[3][0] - p[2][0]);
					double t3 = atan2(p[0][1] - p[3][1], p[0][0] - p[3][0]);

					double ttheta = helper::mod2pi(t1-t0) + helper::mod2pi(t2-t1) +
					                helper::mod2pi(t3-t2) + helper::mod2pi(t0-t3);

					// the magic value is -2*PI. It should be exact,
					// but we allow for (lots of) numeric imprecision.
					if (ttheta < -7 || ttheta > -5) {
						bad = true;
					}
				}

				if (!bad) {
					double d0 = distance(p[0], p[1]);
					double d1 = distance(p[1], p[2]);
					double d2 = distance(p[2], p[3]);
					double d3 = distance(p[3], p[0]);
					double d4 = distance(p[0], p[2]);
					double d5 = distance(p[1], p[3]);

					// check sizes
					if (d0 < minimumTagSize || d1 < minimumTagSize || d2 < minimumTagSize ||
					        d3 < minimumTagSize || d4 < minimumTagSize || d5 < minimumTagSize) {
						bad = true;
					}

					// check aspect ratio
					double dmax = std::max(std::max(d0, d1), std::max(d2, d3));
					double dmin = std::min(std::min(d0, d1), std::min(d2, d3));

					if (dmax > dmin * maxQuadAspectRatio) {
						bad = true;
					}
				}

				if (!bad) {
					Quad q(p, opticalCenter[0], opticalCenter[1]);
					q.observedPerimeter = observedPerimeter;
					quads.push_back(q);
				}
			}
			return;
		}//end of depth==4

		// Not terminal depth. Recurse on any children that obey the correct handedness.
		for(int i=0; i<(int)parent->children.size(); ++i) {
			Segment *child = parent->children[i];
			// (handedness was checked when we created the children)

			// we could rediscover each quad 4 times (starting from
			// each corner). If we had an arbitrary ordering over
			// points, we can eliminate the redundant detections by
			// requiring that the first corner have the lowest
			// value. We're arbitrarily going to use theta...
			if (child->theta > path[0]->theta) {
				continue;
			}

			path[depth+1] = child;
			search(quads, path, child, depth + 1);
		}
	}//end of search()
};//end of TagDetector

}//end of tag
}//end of april
