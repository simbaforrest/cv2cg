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

/* CameraSwitcher.h */

#include <string>
#include <osg/ref_ptr>
#include <osg/Group>

struct CameraSwitcher
{
	std::string infilename;
	bool writeModel;
	int run();
};

osg::ref_ptr<osg::Node> 
createSceneFromFile(std::string filename);

osg::ref_ptr<osg::Camera> 
readCameraFile(std::string str, double n=1, double f=1000);

osg::ref_ptr<osg::MatrixTransform> 
Make_Photo(const osg::Camera& camera, osg::Image& image);

#if 0
osg::ref_ptr<osg::Node>
createSceneFromBundlerResult(std::string filename);

osg::ref_ptr<osg::Node>
readBundlerFile(std::string filename,
								std::vector<std::string> imgfilelist);
#endif