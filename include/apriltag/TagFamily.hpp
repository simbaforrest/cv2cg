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

/* TagFamily.hpp
	modified from april/tag/TagFamily.java
	git://april.eecs.umich.edu/home/git/april.git
*/

#include <iostream>
#include <string>
#include <vector>
//opencv include
#include "OpenCVHelper.h"

#include "TagDetection.hpp"
#include "TagTypes.hpp"

namespace april
{
namespace tag
{

using std::vector;
using helper::hammingDistance;
using cv::Ptr;

/** Generic class for all tag encoding families **/
struct TagFamily {
	/** How many pixels wide is the outer-most white border? This is
	 * only used when rendering a tag, not for decoding a tag (since
	 * we look for the white/black edge). **/
	int whiteBorder;

	/** How many pixels wide is the inner black border? **/
	int blackBorder;

	/** number of bits in the tag. Must be a square (n^2). **/
	const int bits;

	/** dimension of tag. e.g. for 16 bits, d=4. Must be sqrt(bits). **/
	const int d;

	/** What is the minimum hamming distance between any two codes
	 * (accounting for rotational ambiguity? The code can recover
	 * (minHammingDistance-1)/2 bit errors.
	 **/
	const int minimumHammingDistance;

	/** The error recovery value determines our position on the ROC
	 * curve. We will report codes that are within errorRecoveryBits
	 * of a valid code. Small values mean greater rejection of bogus
	 * tags (but false negatives). Large values mean aggressive
	 * reporting of bad tags (but with a corresponding increase in
	 * false positives).
	 **/
	int errorRecoveryBits;

	/** The array of the codes. The id for a code is its index. **/
	const vector<INT64> codes;

	/** The codes array is not copied internally and so must not be
	 * modified externally. **/
	TagFamily(int bits_, int minimumHammingDistance_, vector<INT64> codes_):
		whiteBorder(1), blackBorder(1), errorRecoveryBits(1),
		bits(bits_), d((int)sqrt((float)bits)), codes(codes_),
		minimumHammingDistance(minimumHammingDistance_) {
		assert(d *d == bits);
	}

	void setErrorRecoveryBits(int b) {
		errorRecoveryBits = b;
	}

	void setErrorRecoveryFraction(double v) {
		errorRecoveryBits = (int) (((int) (minimumHammingDistance-1)/2)*v);
	}

	/** if the bits in w were arranged in a d*d grid and that grid was
	 * rotated, what would the new bits in w be?
	 * The bits are organized like this (for d = 3):
	 *
	 *  8 7 6       2 5 8      0 1 2
	 *  5 4 3  ==>  1 4 7 ==>  3 4 5    (rotate90 applied twice)
	 *  2 1 0       0 3 6      6 7 8
	 **/
	static INT64 rotate90(INT64 w, int dd) {
		INT64 wr = 0;

		for (int r = dd-1; r >=0; r--) {
			for (int c = 0; c < dd; c++) {
				int b = r + dd*c;

				wr = wr << 1;

				if ((w & (1L << b))!=0) {
					wr |= 1;
				}

			}
		}
		return wr;
	}

	/** Given an observed tag with code 'rcode', try to recover the
	 * id. The corresponding fields of TagDetection will be filled
	 * in. **/
	void decode(TagDetection& det, INT64 rcode) {
		int  bestid = -1;
		int  besthamming = INT_MAX;
		int  bestrotation = 0;
		INT64 bestcode = 0;

		vector<INT64> rcodes(4);

		rcodes[0] = rcode;
		rcodes[1] = rotate90(rcodes[0], d);
		rcodes[2] = rotate90(rcodes[1], d);
		rcodes[3] = rotate90(rcodes[2], d);

		for (int id = 0; id < (int)codes.size(); id++) {

			for (int rot = 0; rot < (int)rcodes.size(); rot++) {
				int thishamming = helper::hammingDistance(rcodes[rot], codes[id]);
				if (thishamming < besthamming) {
					besthamming = thishamming;
					bestrotation = rot;
					bestid = id;
					bestcode = codes[id];
				}
			}
		}

		det.id = bestid;
		det.hammingDistance = besthamming;
		det.rotation = bestrotation;
		det.good = (det.hammingDistance <= errorRecoveryBits);
		det.obsCode = rcode;
		det.code = bestcode;
	}

	/** Return the dimension of the tag including borders when we render it.**/
	int getTagRenderDimension() {
		return whiteBorder*2 + blackBorder*2 + d;
	}

	cv::Mat makeImage(int id) {
		INT64 v = codes[id];

		int width = getTagRenderDimension();
		int height = width;

		cv::Mat im(width, height, CV_8UC3);

		// Draw the borders.  It's easier to do this by iterating over
		// the whole tag than just drawing the borders.
		for (int y = 0; y < width; y++) {
			for (int x = 0; x < height; x++) {
				if (y < whiteBorder || y+whiteBorder >= height ||
				        x < whiteBorder || x+whiteBorder >= width) {
					cv::Vec3b &pix = im.at<cv::Vec3b>(y,x);
					pix[0]=pix[1]=pix[2]=255;
				} else {
					cv::Vec3b &pix = im.at<cv::Vec3b>(y,x);
					pix[0]=pix[1]=pix[2]=0;
				}
			}
		}

		// Now, draw the payload.
		for (int y = 0; y < d; y++) {
			for (int x = 0; x < d; x++) {
				if ((v&(1L<<(bits-1)))!=0) {
					cv::Vec3b &pix = im.at<cv::Vec3b>(y + whiteBorder + blackBorder,x + whiteBorder + blackBorder);
					pix[0]=pix[1]=pix[2]=255;
				} else {
					cv::Vec3b &pix = im.at<cv::Vec3b>(y + whiteBorder + blackBorder,x + whiteBorder + blackBorder);
					pix[0]=pix[1]=pix[2]=0;
				}

				v = v<<1;
			}
		}

		return im;
	}

	/** Generate all valid tags, writing them as PNGs in the specified
	 * directory.  The files will be named tag##_##_####.png, where
	 * the first block is nbits, the second block is hamming distance,
	 * and the final block is the id.
	 **/
	void writeAllImages(std::string dirpath, int scale=1) {
		for (int i = 0; i < (int)codes.size(); i++) {
			cv::Mat im = makeImage(i);
			if(scale>1) {
				static int textH=30;
				cv::resize(im,im,cv::Size(im.rows*scale+textH+10,im.cols*scale),0,0,cv::INTER_NEAREST);
				cv::putText(im, cv::format("ID=%03d",i),
					cv::Point(im.cols/2,im.rows-textH-5), CV_FONT_NORMAL, 1, cv::Scalar(0,0,0), 2);
			}
			std::string fname = cv::format("tag%02d_%02d_%05d.png",
			                               bits,
			                               minimumHammingDistance,
			                               i);
			cv::imwrite(dirpath+fname,im);
		}
	}

	void writeAllImagesMosaic(std::string filepath) {
		int width = (int) sqrt((float)codes.size());
		int height = codes.size()/width+1;
		int dim = getTagRenderDimension();

		cv::Mat im(dim*height, dim*width, CV_8UC3, cv::Scalar(255,255,255));
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				int id = y*width+x;
				if(id >= (int)codes.size()) {
					continue;
				}
				cv::Mat roi(im, cv::Rect(x*dim, y*dim, dim, dim));
				makeImage(id).copyTo(roi);
			}
		}
		cv::imwrite(filepath, im);
	}

	void printHammingDistances() {
		vector<int> hammings(d*d+1);

		for (int i = 0; i < (int)codes.size(); i++) {
			INT64 r0 = codes[i];
			INT64 r1 = rotate90(r0, d);
			INT64 r2 = rotate90(r1, d);
			INT64 r3 = rotate90(r2, d);

			for (int j = i+1; j < (int)codes.size(); j++) {

				int dd = std::min(std::min(hammingDistance(r0, codes[j]),
				                           hammingDistance(r1, codes[j])),
				                  std::min(hammingDistance(r2, codes[j]),
				                           hammingDistance(r3, codes[j])));

				hammings[dd]++;
			}
		}

		for (int i = 0; i < (int)hammings.size(); i++) {
			printf("%10d  %10d\n", i, hammings[i]);
		}
	}

	void writeAllImagesPostScript(std::string filepath) {
		int sz = d + 2*whiteBorder + 2*blackBorder;

		std::ofstream outs(filepath.c_str());

		outs<<"/pagewidth 8.5 72 mul def                      \n"
		    "/pageheight 11 72 mul def                      \n"

		    "/maketag                                       \n"
		    "{                                              \n"
		    "  /img exch def                                \n"
		    "  /name exch def                               \n"
		    "  gsave                                        \n"
		    "  pagewidth 2 div pageheight 2 div translate   \n"
		    "  0 0 moveto                                   \n"
		    "  1.0 pagewidth mul dup scale                  \n"
		    "  1 -1 scale                                   \n"
		    "  -.5 -.5 translate                            \n"
		    "  "<<sz<<" "<<sz<<" 1 [ "<<sz<<" 0 0 "<<sz<<" 0 0 ] { img } image \n"
		    "  0 setlinewidth .5 setgray [0.002 0.01] 0 setdash \n"
		    "  0 0 moveto 1 0 lineto 1 1 lineto 0 1 lineto  \n"
		    "  closepath stroke                             \n"
		    "  grestore                                     \n"
		    "  gsave                                        \n"
		    "  pagewidth 2 div 72 translate                 \n"
		    "  /Helvetica-Bold findfont 20 scalefont setfont \n"
		    "  name                                         \n"
		    "  dup stringwidth pop -.5 mul 0 moveto         \n"
		    "  show                                         \n"
		    "  grestore                                     \n"
		    "  showpage                                     \n"
		    "} def                                          \n";

		for (int id = 0; id < (int)codes.size(); id++) {
			cv::Mat im = makeImage(id);

			// convert image into a postscript string
			int width = im.cols, height = im.rows;

			std::string imgdata = "";

			for (int y = 0; y < height; y++) {
				INT64 v = 0;
				int vlen = 0;

				for (int x = 0; x < width; x++) {
					cv::Vec3b &pix = im.at<cv::Vec3b>(y,x);
					int b = pix[0]||pix[1]||pix[2]?1:0;
					v = (v<<1) | b;
					vlen++;
				}

				// pad to a byte boundary.
				while ((vlen%8) != 0) {
					v = (v<<1) | 0;
					vlen++;
				}

				std::string formatstr = cv::format("%%0%dx",vlen/4);
				imgdata += cv::format(formatstr.c_str(), v);
			}

			outs<<"(april.tag.Tag"<<bits<<"h"<<minimumHammingDistance
			    <<", id = "<<id<<") <"<<imgdata<<"> maketag\n";
		}

		outs.close();
	}
};

}//end of tag
}//end of april
