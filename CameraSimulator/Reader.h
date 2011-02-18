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

/* Reader.h */

#include <osg/Array>

inline osg::ref_ptr<osg::Vec3Array> ReadVertex(int num, std::istream& in)
{
	osg::ref_ptr<osg::Vec3Array> ret = new osg::Vec3Array;
	while(num--) {
		std::string str;
		std::getline(in,str,'\n');
		if(str.length()==0) continue;

		double x,y,z;
		sscanf(str.c_str(), 
			"%lf%*[^0-9-+.eE]%lf%*[^0-9-+.eE]%lf", &x,&y,&z);
		ret->push_back(osg::Vec3(x,y,z));
	}
	return ret;
}

inline osg::ref_ptr<osg::Geometry> ReadLine(int num, std::istream& in)
{
	osg::ref_ptr<osg::Geometry> ret = new osg::Geometry;
	osg::ref_ptr<osg::DrawElementsUInt> lines = 
		new osg::DrawElementsUInt(osg::PrimitiveSet::LINES,0);
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	while(num--) {
		std::string str;
		std::getline(in,str,'\n');
		if(str.length()==0) continue;

		int s,e,r,g,b;
		sscanf(str.c_str(), 
			"%d%*[^0-9-+.eE]%d%*[^0-9-+.eE]%d%*[^0-9-+.eE]%d%*[^0-9-+.eE]%d", 
			&s,&e,&r,&g,&b);
		lines->push_back(s); lines->push_back(e);
		colors->push_back(osg::Vec4(r,g,b,1));
	}
	ret->addPrimitiveSet(lines);
	ret->setColorArray(colors);
	ret->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
	return ret;
}

inline osg::ref_ptr<osg::Group> ReadModel(int num, std::istream& in)
{
	osg::ref_ptr<osg::Group> ret = new osg::Group;
	while(num--) {
		std::string str;
		std::getline(in,str,'\n');
		if(str.length()==0) continue;

		osg::Node* node = osgDB::readNodeFile(str);
		if(node)
			ret->addChild(node);
	}
	return ret;
}