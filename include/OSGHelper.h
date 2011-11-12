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

inline osg::Geode *createGeodeFromImage(osg::Image *image, bool flip=false)
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
			osg::Texture2D *texture = new osg::Texture2D;
			texture->setResizeNonPowerOfTwoHint(false);
			texture->setImage(image);

			// set up the geoset.
			osg::Geometry *geom = new osg::Geometry;
			osg::StateSet *dstate = geom->getOrCreateStateSet();
			dstate->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
			dstate->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
			dstate->setTextureAttributeAndModes(0,
			                                    texture,osg::StateAttribute::ON);

			osg::Vec3Array *coords = new osg::Vec3Array(4);
			(*coords)[0].set(0,y,0);
			(*coords)[1].set(0,0,0);
			(*coords)[2].set(x,0,0);
			(*coords)[3].set(x,y,0);
			geom->setVertexArray(coords);

			osg::Vec2Array *tcoords = new osg::Vec2Array(4);
			(*tcoords)[0].set(0.0f,texcoord_y_t);
			(*tcoords)[1].set(0.0f,texcoord_y_b);
			(*tcoords)[2].set(1.0f,texcoord_y_b);
			(*tcoords)[3].set(1.0f,texcoord_y_t);
			geom->setTexCoordArray(0,tcoords);

			osg::Vec4Array *colours = new osg::Vec4Array(1);
			(*colours)[0].set(1.0f,1.0f,1.0,1.0f);
			geom->setColorArray(colours);
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);

			geom->addPrimitiveSet(new osg::DrawArrays(
			                          osg::PrimitiveSet::QUADS,0,4));

			// set up the geode.
			osg::Geode *geode = new osg::Geode;
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

	ARVideoBackground(osg::Image *img, bool flip=false) {
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
		osg::Geode *geode = createGeodeFromImage(img,flip);
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

namespace helper
{
using namespace OSGHelper;
}
