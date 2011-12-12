#pragma once
/*
 *  Copyright (c) 2010  Chen Feng (cforrest (at) umich.edu)
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

/* OpenCV2OSG.h
   a set of helper functions for conversion between opencv and openscenegraph */

#include "OpenCVHelper.h"
#include "OSGHelper.h"

namespace helper
{

using namespace cv;
using namespace std;

inline bool cvmat2osgimage(Mat &cvImg, osg::Image *osgImg, bool bflip=false)
{
	if(!osgImg) {
		cout<<"[cvmat2osgimage] no valid osg::Image!"<<endl;
		return false;
	}
	// Flip image from top-left to bottom-left origin
	if(bflip) {
		flip(cvImg,cvImg,0);
	}
	osgImg->setWriteHint(osg::Image::NO_PREFERENCE);

	if(cvImg.channels() == 3) {
		// Convert from BGR to RGB color format
		// cvtColor( cvImg, cvImg, CV_BGR2RGB );

		osgImg->setImage(
		    cvImg.cols, //s
		    cvImg.rows, //t
		    3, //r
		    GL_LINE_STRIP, //GLint internalTextureformat, (, 0x0003)
		    GL_BGR, // GLenum pixelFormat, (GL_RGB, 0x1907)
		    GL_UNSIGNED_BYTE, // GLenum type, (GL_UNSIGNED_BYTE, 0x1401)
		    cvImg.data, // unsigned char* data
		    osg::Image::NO_DELETE // AllocationMode mode (shallow copy)
		);//int packing=1); (???)

		return true;
	} else if(cvImg.channels() == 1) {
		osgImg->setImage(
		    cvImg.cols, //s
		    cvImg.rows, //t
		    1, //r
		    GL_LINE_STRIP, //GLint internalTextureformat, (GL_LINE_STRIP, 0x0003)
		    GL_LUMINANCE, // GLenum pixelFormat
		    GL_UNSIGNED_BYTE, // GLenum type
		    cvImg.data, // unsigned char* data
		    osg::Image::NO_DELETE // AllocationMode mode (shallow copy)
		);//int packing=1); (???)
		return true;
	}

	cout<<"[cvmat2osgimage] unrecognized image type!"<<endl;
	return false;
}

}
