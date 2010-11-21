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

/* SwitcherAPI.h */

#include <osg/ref_ptr>
#include <osg/Group>
#include <osg/Texture2D>
#include <osgViewer/Viewer>
#include <osg/NodeVisitor>
#include <osg/MatrixTransform>
#include <osg/geometry>
#include <osgText/Text>
#include <osg/AlphaFunc>
#include <osg/blendcolor>
#include <osg/BlendFunc>
#include <osg/BlendEquation>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "CV_CG.h"

template<class Type>
inline Type Interpolate(const Type&s, const Type& e, double t)
{
#if 1
	return s*(1-t)+e*t; //linear
#else
	t -= 1;
	t *= t;
	return e+(s-e)*t; //parabola
#endif
}

//t belongs to [0,1]
void Interpolate_Camera(const osg::Camera& s, const osg::Camera& e,
	const double t, osg::Camera& camera);

osg::ref_ptr<osg::MatrixTransform> 
Make_Photo(const osg::Camera& camera, osg::Image& image);

osg::ref_ptr<osg::Camera> 
readCameraFile(std::string str, double n=1, double f=1000);

osg::ref_ptr<osg::Node> 
createSceneFromFile(std::string filename);

osg::ref_ptr<osg::Node>
createSceneFromBundlerResult(std::string filename);

osg::ref_ptr<osg::Node>
readBundlerFile(std::string filename,
								std::vector<std::string> imgfilelist);
