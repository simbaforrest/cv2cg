#pragma once
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

/* SimulatorAPI.h */

#include <osgDB/ReadFile>

#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>

#include <osgGA/TrackballManipulator>

#include <osg/Material>
#include <osg/Geode>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/PolygonOffset>
#include <osg/MatrixTransform>
#include <osg/Camera>
#include <osg/RenderInfo>
#include <osg/Point>

#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
#include <osgText/Text>

#include <iostream>
#include <fstream>
#include <sstream>

inline void printUsage(bool chinese=true)
{
	system("cls");
	if(!chinese) {
		std::cout<<
"Usage:\n\
   Mouse:\n\tLEFT_BUTTON,RIGHT_BUTTON can be used to change the \
position and posture of the camera, i.e. change the \
R and C in computer vision camera model(KR[I|-C]), \
or change the ViewMatrix in computer graphics camera model.\n\
\tSCROLLWHEEL can be used to change the focal lenght, i.e. change \
the alpha_X and alpha_Y in computer vision, or change the near plane \
in computer graphics.\n\
   Key:\n\tUP,DOWN,LEFT,RIGHT can be used to change the \
principle point position, i.e. (u0,v0) in computer vision, or \
the l,r,b,t of the Frustum in computer graphics.\n\
\t'p' can be used to capture a photo and the application will \
write out an image, a parameter file, a control points file and \
also an osg model file.\n\
By the Way, by changing the size of the window, you can change the \
size of the image you get, i.e., the size of the window is just that of the output \
image."
		<<std::endl;
	} else {
		std::cout<<
"使用方法:\n\
   鼠标:\n\
\t左键，右键 用于改变相机位置和姿态, \
即改变计算机视觉相机模型(KR[I|-C])中的R和C, 亦即\
改变计算机图形学相机模型中的ViewMatrix.\n\
\t滚轮 用于改变焦距, 即改变计算机视觉中\
的alpha_X和alpha_Y, 亦即改变计算机图形学中\
的近剪裁面.\n\
   键盘:\n\
\t上下左右 用于改变像主点位置, 即计算机视\
觉中的(u0,v0), 亦即计算机图形学中视景体的l,r,b,t.\n\
\t'p' 用于拍摄一幅照片，同时程序会写出一幅图\
片，一个参数文件（.par），一个控制点文件（.cp）\
以及一个osg模型文件.\n\
\t'h' 打印使用方法.\n\
\t'g' 打印当前相机的计算机图形学参数.\n\
\t'v' 打印当前相机的计算机视觉参数.\n\
\t同时，改变窗口大小可以改变照片的像幅，即：输\
出影像的大小亦为窗口大小."
		<<std::endl;
	}
}

inline void Modify_PrinciplePoint(osg::Camera& cam, double dx, double dy)
{
	double l,r,t,b,n,f;
	cam.getProjectionMatrixAsFrustum(l,r,b,t,n,f);
	double w=dx*fabs(r-l), h=dy*fabs(t-b);
	l+=w; r+=w;
	t+=h; b+=h;
	cam.setProjectionMatrixAsFrustum(l,r,b,t,n,f);
}

inline void Distort_Image(osg::Image* _image, 
	double k1, double c0, double r0, int w, int h)
{
	static osg::ref_ptr<osg::Image> newImg = new osg::Image;
	newImg->allocateImage(_image->s(),_image->t(),
		_image->r(),_image->getPixelFormat(),
		_image->getDataType());

	if (_image && _image->getPixelFormat()==GL_RGBA &&
		_image->getDataType()==GL_UNSIGNED_BYTE)
	{
		int column_start = 0;//_image->s()/4;
		int column_end = _image->s();//3*column_start;

		int row_start = 0;//_image->t()/4;
		int row_end = _image->t();//3*row_start;

		for(int nr=row_start; nr<row_end; ++nr)
		{
			unsigned char* ndata = newImg->data(column_start, nr);
			for(int nc=column_start; nc<column_end; ++nc)
			{
				double dr = nr-r0, dc = nc-c0;
				double r2 = dr*dr+dc*dc;
				int r = int(nr - dr * k1 * r2 + 0.5);
				int c = int(nc - dc * k1 * r2 + 0.5);
				r = std::max(0,r); c = std::max(0,c);
				r = std::min(row_end-1, r); c = std::min(column_end-1, c);

				const unsigned char* data = _image->data(c, r);
				(*ndata++) = (*data++);
				(*ndata++) = (*data++);
				(*ndata++) = (*data++);
				(*ndata++) = (*data++);
			}
		}
	}
	else if (_image && _image->getPixelFormat()==GL_RGBA &&
		_image->getDataType()==GL_FLOAT)
	{
		int column_start = 0;//_image->s()/4;
		int column_end = _image->s();//3*column_start;

		int row_start = 0;//_image->t()/4;
		int row_end = _image->t();//3*row_start;

		for(int nr=row_start; nr<row_end; ++nr)
		{
			float* ndata = (float*)newImg->data(column_start, nr);
			for(int nc=column_start; nc<column_end; ++nc)
			{
				double dr = nr-r0, dc = nc-c0;
				double r2 = dr*dr+dc*dc;
				int r = int(nr - dr * k1 * r2 + 0.5);
				int c = int(nc - dc * k1 * r2 + 0.5);
				r = std::max(0,r); c = std::max(0,c);
				r = std::min(row_end-1, r); c = std::min(column_end-1, c);

				const float* data = (const float*)_image->data(c, r);
				(*ndata++) = (*data++);
				(*ndata++) = (*data++);
				(*ndata++) = (*data++);
				(*ndata++) = (*data++);
			}
		}
	}

	glDrawPixels(w,h,
		GL_RGBA, GL_UNSIGNED_BYTE, newImg->data());
}