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

#include "AllHelpers.h"

#include "apriltag/apriltag.hpp"

#include "TemplateData.hpp"

namespace keg {

using april::tag::TagFamily;
using april::tag::TagDetector;
using april::tag::TagDetection;

/**
interface for keg recognizer
*/
struct Recognizer {
	virtual void operator()(cv::Mat &nframe,
		std::vector<cv::Mat>& retH,
		std::vector<int>& retId,
		int errorThresh=0) = 0;

	virtual bool init(std::vector<TemplateData> &tds) = 0;

	virtual std::string name() const { return ""; }
};

struct AprilTagRecognizer : public Recognizer {
	Ptr<TagFamily> tagFamily;
	Ptr<TagDetector> detector;

	inline void operator()(cv::Mat &nframe,
		std::vector<cv::Mat>& retH,
		std::vector<int>& retId,
		int errorThresh=0)
	{
		retH.clear(); retId.clear();
		vector<TagDetection> detections;
		detector->process(nframe, detections);

		for(int id=0; id<(int)detections.size(); ++id) {
			TagDetection &dd = detections[id];
			if(dd.hammingDistance>errorThresh) continue;

			retId.push_back(dd.id);
			retH.push_back( cv::Mat(3,3,CV_64FC1,dd.homography[0]) );
		}
	}

	inline bool init(std::vector<TemplateData> &tds) {
		for(int i=0; i<(int)tds.size(); ++i) {
			keg::TemplateData& td = tds[i];
			vector<int> ids;
			vector<Mat> HIs;
			(*this)(td.img, HIs, ids);
			if((int)ids.size() != 1) {
				logle("[AprilTagRecognizer.init error] found none/multiple apriltag"
					" on template image #"<<i<<", exit.");
				return false;
			} else {
				td.iHI = HIs.front().inv();
				td.id = ids.front();
				logli("[AprilTagRecognizer.init] found tag "<<td.id);
			}
		}
		return true;
	}

	inline std::string name() const { return "apriltag"; }
};

struct BriefRecognizer : public Recognizer {
	cv::BriefDescriptorExtractor brief;
	cv::BFMatcher desc_matcher;
	cv::GridAdaptedFeatureDetector detector;

	std::vector<cv::KeyPoint> tpltKeys;
	cv::Mat tpltDesc;

	BriefRecognizer() : brief(32), desc_matcher(NORM_HAMMING),
		detector(new FastFeatureDetector(10, true), 500, 4, 4)
	{}

	inline bool init(std::vector<TemplateData> &tds) {
		if(tds.size()>1) {
			logle("[BriefRecognizer error] only accept one template; for multiple templates, use AprilTagRecognizer!");
			return false;
		}
		TemplateData& tpltData = tds.front();
		detector.detect(tpltData.img, tpltKeys);

		vector<int> ids;
		vector<Mat> HIs;
		brief.compute(tpltData.img, tpltKeys, tpltDesc);
		tpltData.iHI = cv::Mat::eye(3,3,CV_64FC1);
		tpltData.id = 0;
		logli("[BriefRecognizer.init] inited tag "<<tpltData.id<<" with #keys="<<tpltKeys.size());
		return true;
	}

	inline std::string name() const { return "brief"; }

	//Converts matching indices to xy points
	static inline void matches2points(
		const std::vector<cv::KeyPoint>& train,
		const std::vector<cv::KeyPoint>& query,
		const std::vector<cv::DMatch>& matches,
		std::vector<cv::Point2f>& pts_train,
		std::vector<cv::Point2f>& pts_query)
	{
		pts_train.clear();
		pts_query.clear();
		pts_train.reserve(matches.size());
		pts_query.reserve(matches.size());

		for (size_t i = 0; i < matches.size(); ++i)
		{
			const cv::DMatch & dmatch = matches[i];

			pts_query.push_back(query[dmatch.queryIdx].pt);
			pts_train.push_back(train[dmatch.trainIdx].pt);
		}
	}

	inline void operator()(
		cv::Mat &nframe,
		std::vector<cv::Mat>& retH,
		std::vector<int>& retId,
		int errorThresh=0)
	{
		if(tpltKeys.empty()) {
			logli("[BriefRecognizer warn] template data not inited yet in this recognizer!");
			return;
		}
		std::vector<cv::KeyPoint> nkeys;
		detector.detect(nframe, nkeys);
		logli("[BriefRecognizer] find #nkeys="<<nkeys.size());

		cv::Mat ndesc;
		brief.compute(nframe, nkeys, ndesc);

		std::vector<cv::DMatch> matches;
		desc_matcher.match(ndesc, tpltDesc, matches);

		std::vector<cv::Point2f> tpltX, nX;
		matches2points(tpltKeys, nkeys, matches, tpltX, nX);

		retId.clear(); retH.clear();
		if (matches.size() > 5)
		{
			std::vector<unsigned char> match_mask;
			cv::Mat H = cv::findHomography(tpltX, nX, cv::RANSAC, 4, match_mask);
			if(cv::countNonZero(match_mask)>errorThresh) {
				retId.push_back(0);
				retH.push_back(H);
			} else {
				logli("[BriefRecognizer] no enough RANSAC inliers!");
			}
		} else {
			logli("[BriefRecognizer] no enough match inliers!");
		}
	}
};

}//end of namespace keg
