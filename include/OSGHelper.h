#pragma once
/*
 *  Copyright (c) 2011  Chen Feng (cforrest (at) umich.edu)
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

/* OSGHelper.h
   a set of helper functions for easier access to openscenegraph */

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osgViewer/ViewerEventHandlers>

#include <osg/Material>
#include <osg/Geode>
#include <osg/Geometry>
//#include <osg/Array>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/MatrixTransform>
#include <osg/Camera>

#include <osgText/Text>
#include <osg/Texture2D>
#include <osg/Image>

namespace OSGHelper
{

inline osg::Geode* createGeodeFromImage(osg::Image* image, bool flip=false)
{
	if (image) {
		float s = image->s();
		float t = image->t();
		if (s>0 && t>0) {
			float y = t;
			float x = s;

			float texcoord_y_b = flip ? 0.0f : 1.0f;
			float texcoord_y_t = flip ? 1.0f : 0.0f;

			// set up the texture.
			osg::Texture2D* texture = new osg::Texture2D;
			texture->setResizeNonPowerOfTwoHint(false);
			texture->setImage(image);

			// set up the geoset.
			osg::Geometry* geom = new osg::Geometry;
			osg::StateSet* dstate = geom->getOrCreateStateSet();
			dstate->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
			dstate->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
			dstate->setTextureAttributeAndModes(0, 
				texture,osg::StateAttribute::ON);

			osg::Vec3Array* coords = new osg::Vec3Array(4);
			(*coords)[0].set(0,y,0);
			(*coords)[1].set(0,0,0);
			(*coords)[2].set(x,0,0);
			(*coords)[3].set(x,y,0);
			geom->setVertexArray(coords);

			osg::Vec2Array* tcoords = new osg::Vec2Array(4);
			(*tcoords)[0].set(0.0f,texcoord_y_t);
			(*tcoords)[1].set(0.0f,texcoord_y_b);
			(*tcoords)[2].set(1.0f,texcoord_y_b);
			(*tcoords)[3].set(1.0f,texcoord_y_t);
			geom->setTexCoordArray(0,tcoords);

			osg::Vec4Array* colours = new osg::Vec4Array(1);
			(*colours)[0].set(1.0f,1.0f,1.0,1.0f);
			geom->setColorArray(colours);
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);

			geom->addPrimitiveSet(new osg::DrawArrays(
				osg::PrimitiveSet::QUADS,0,4));

			// set up the geode.
			osg::Geode* geode = new osg::Geode;
			geode->addDrawable(geom);

			return geode;
		}
		return NULL;
	}
	return NULL;
}


struct ARVideoBackground : public osg::Camera {
	osg::ref_ptr<osg::Image> frame;
	int width,height; //image width and height

	ARVideoBackground(osg::Image* img, bool flip=false) {
		frame = img;
		width = img?img->s():1;
		height = img?img->t():1;

		//SET UP HUD
		// set the projection matrix
		setProjectionMatrix(osg::Matrix::ortho2D(0,width,0,height));
		// set the view matrix
		setReferenceFrame(osg::Transform::ABSOLUTE_RF);
		setViewMatrix(osg::Matrix::identity());
		// only clear the depth buffer
		setClearMask(GL_DEPTH_BUFFER_BIT);
		// draw subgraph after main camera view.
		setRenderOrder(osg::Camera::NESTED_RENDER);
		getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
		getOrCreateStateSet()->setRenderBinDetails(-1,"RenderBin");
		setAllowEventFocus(false);

		//add geometry
		osg::Geode* geode = createGeodeFromImage(img,flip);
		if(geode) {
			addChild(geode);
		} else {
			std::cout<<"[ARVideoNode] No Valid Image!"<<std::endl;
		}
	}
};

struct ARSceneRoot : public osg::Camera {
	ARSceneRoot() {
		setReferenceFrame(osg::Transform::ABSOLUTE_RF);
		// only clear the depth buffer
		setClearMask(GL_DEPTH_BUFFER_BIT);
		getOrCreateStateSet()->setRenderBinDetails(1,"RenderBin");
		setAllowEventFocus(false);
	}
};

}

namespace helper {
using namespace OSGHelper;
}
