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

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <osgDB/ReadFile>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Array>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osgText/Text>
#include <osg/Texture2D>
#include <osg/Image>
#include <osg/AlphaFunc>
#include <osg/BlendColor>
#include <osg/BlendFunc>
#include <osg/BlendEquation>
#include <osgViewer/Viewer>
#include <osg/AnimationPath>
#include <osgGA/AnimationPathManipulator>

#include "OpenCVHelper.h"
#include "CV2CG.h"
#include "OSGHelper.h"

// script processor for create scene from scripts
namespace CreateSceneHelper {

using namespace std;
using IOHelper::readValidLine;

/**
SYNTAX:
VERTEX <#points> [lightflag=0] [pointsize=3] [#lines=0] [labelsize=24.0]

<#points> lines of 3D or 2D (with default z=0) points
[pointsize] size of each point
[#lines] lines of line indices, indicating which two points are connected
 index start from 0
[labelsize] size of label

EXAMPLE script:
VERTEX 2 3.0 1 0
1.5 2 0.5 0 0 1
2 3.5 0.1 1 0 0
0 1 1 1 1
 **/
inline osg::ref_ptr<osg::Node> create_VERTEX(std::istream& in, int npoints,
	bool lightflag=false,
	double pointsize=3.0, int nlines=0, double labelsize=24.0) {
	if(npoints<=0) return 0;

	osg::Geode* ret = new osg::Geode;
	ret->setName("CreateSceneHelper.VERTEX");
	osg::Vec3Array* v3a = new osg::Vec3Array;
	{
		osg::Vec4Array* v4a = new osg::Vec4Array;
		std::string str;
		while(npoints--) {
			readValidLine(in,str);
			double x=0,y=0,z=0,r=1,g=0,b=0;
			sscanf(str.c_str(), 
				"%lf%*[^0-9-+.eE]%lf%*[^0-9-+.eE]%lf%*[^0-9-+.eE]"
				"%lf%*[^0-9-+.eE]%lf%*[^0-9-+.eE]%lf", &x,&y,&z,&r,&g,&b);
			v3a->push_back(osg::Vec3(x,y,z));
			v4a->push_back(osg::Vec4(r,g,b,1));
		}
		osg::Geometry* geom = new osg::Geometry;
		osg::DrawArrays* points =
			new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, v3a->size());
		geom->setVertexArray(v3a);
		geom->setColorArray(v4a);
		geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
		geom->addPrimitiveSet(points);
		geom->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(pointsize));
		ret->addDrawable(geom);
	}
	if(nlines>0) {
		osg::Geometry* geom = new osg::Geometry;
		osg::DrawElementsUInt* lines = 
			new osg::DrawElementsUInt(osg::PrimitiveSet::LINES,0);
		osg::Vec4Array* v4a = new osg::Vec4Array;
		std::string str;
		while(nlines--) {
			readValidLine(in, str);
			int s,e;
			double r=1,g=1,b=1;
			sscanf(str.c_str(), 
				"%d%*[^0-9-+.eE]%d%*[^0-9-+.eE]"
				"%lf%*[^0-9-+.eE]%lf%*[^0-9-+.eE]%lf",
				&s,&e,&r,&g,&b);
			lines->push_back(s); lines->push_back(e);
			v4a->push_back(osg::Vec4(r,g,b,1));
		}
		geom->setVertexArray(v3a);
		geom->setColorArray(v4a);
		geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
		geom->addPrimitiveSet(lines);
		ret->addDrawable(geom);
	}
	if(labelsize>0) {
		for(int i=0; i<(int)v3a->size(); ++i) {
			osgText::Text* t = new osgText::Text;
			std::stringstream ss;
			ss << i;
			std::string str;
			ss >> str;
			t->setText(str);
			t->setPosition(v3a->at(i));
			t->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			t->setCharacterSize(labelsize);
			t->setAxisAlignment(osgText::Text::SCREEN);
			ret->addDrawable(t);
		}
	}
	if(lightflag) {
		ret->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::ON);
	} else {
		ret->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	}
	return ret;
}

/**
SYNTAX:
MODEL <#models> [lightflag=0] [scaleflag=0] [translateflag=0] [rotateflag=0]

<#models> lines of model's path
[lightflag] indicates whether to be LIGHT ON or OFF
[scaleflag] indicates whether to be scaled or not
[translateflag] indicates whether to be translate or not
[rotateflag] indicates whether to to be rotate or not

EXAMPLE script:
MODEL 2 0 1 0 1
0.5 0.5 0.5 //scale by 0.5
1 0 0 3.14 //rotate around x-axis of 3.14 rad
cow.osg
 **/
inline osg::ref_ptr<osg::Node> create_MODEL(std::istream& in, int nmodels,
	bool lightflag=false,
	bool scaleflag=false, bool translateflag=false, bool rotateflag=false) {
	if(nmodels<=0) return 0;

	osg::MatrixTransform* ret = new osg::MatrixTransform(osg::Matrix::identity());
	ret->setName("CreateSceneHelper.MODEL");
	std::string str;
	if(scaleflag) {
		readValidLine(in, str);
		double x=1,y=1,z=1;
		sscanf(str.c_str(), "%lf%*[^0-9-+.eE]%lf%*[^0-9-+.eE]%lf", &x,&y,&z);
		if(x>0 && y>0 && z>0) {
			ret->getOrCreateStateSet()->setMode(GL_NORMALIZE,osg::StateAttribute::ON);
			ret->postMult(osg::Matrix::scale(x,y,z));
		}
	}
	if(translateflag) {
		readValidLine(in, str);
		double x=0,y=0,z=0;
		sscanf(str.c_str(), "%lf%*[^0-9-+.eE]%lf%*[^0-9-+.eE]%lf", &x,&y,&z);
		ret->postMult(osg::Matrix::translate(x,y,z));
	}
	if(rotateflag) {
		readValidLine(in, str);
		double x=1,y=0,z=0,w=0;
		sscanf(str.c_str(), 
			"%lf%*[^0-9-+.eE]%lf%*[^0-9-+.eE]"
			"%lf%*[^0-9-+.eE]%lf",&x,&y,&z,&w);
		cout<<"rotate: "<<x<<","<<y<<","<<z<<":"<<w<<endl;
		ret->postMult(osg::Matrix::rotate(w,x,y,z));
	}
	if(lightflag) {
		ret->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::ON);
	} else {
		ret->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	}
	while(nmodels--) {
		readValidLine(in, str);
		osg::Node* node = osgDB::readNodeFile(str);
		if(!node) {
			osg::ref_ptr<osg::Image> img = osgDB::readImageFile(str);
			if(img.valid()) node = OSGHelper::createGeodeFromImage(img);
		}
		if(node)
			ret->addChild(node);
	}
	return ret;
}

///////////////////////////////////////////////////////////////////
inline osg::ref_ptr<osg::Camera> create_CAMERA(std::istream& in,
	double imgs, double imgt, int id)
{
	double K[3][3]={{0}};
	double C[3]={0};
	double R[3][3]={{0}};
	double n=0.1, f=1000;
	bool CisT=false;
	osg::ref_ptr<osg::Camera> ret = new osg::Camera;
	bool readK,readC,readR;
	readK=readC=readR=false;
	std::string line;
	while( !(readK && readR && readC)
			&& helper::readValidLine(in, line) ) {

		if(line.find("n=")!=line.npos) {
			std::stringstream ss;
			ss << line;
			std::string tmp;
			while(true) {
				ss >> tmp;
				if(tmp.find("n=")!=tmp.npos) {
					sscanf(tmp.c_str(),"n=%lf", &n);
					break;
				}
			}
		}

		if(line.find("f=")!=line.npos) {
			std::stringstream ss;
			ss << line;
			std::string tmp;
			while(true) {
				ss >> tmp;
				if(tmp.find("f=")!=tmp.npos) {
					sscanf(tmp.c_str(),"f=%lf", &f);
					break;
				}
			}
		}

		if(line.find_first_of("x=")==0) {
			double x,y,w,h;
			sscanf(line.c_str(),
				"x=%lf y=%lf w=%lf h=%lf",
				&x,&y,&w,&h
				);
			osg::ref_ptr<osg::Viewport> vp = new osg::Viewport(x,y,w,h);
			ret->setViewport(vp);
		}

		if(line == std::string("K(alphaX alphaY u0 v0)=")) {
			in >> K[0][0] >> K[1][1] >> K[0][2] >> K[1][2];
			K[2][2] = 1;
			//helper::print(3,3,K[0],"K");
			readK=true;
			continue;
		}

		if(line == std::string("R=")) {
			for(int i=0; i<3; ++i)
				for(int j=0; j<3; ++j)
					in >> R[i][j];
			//helper::print(3,3,R[0],"R");
			readR=true;
			continue;
		}

		if(line == std::string("C=")) {
			in >> C[0] >> C[1] >> C[2];
			//helper::print(3,1,C,"C");
			readC=true;
			CisT = false;
			continue;
		}

		if(line == std::string("T=")) {
			in >> C[0] >> C[1] >> C[2];
			readC=true;
			CisT = true;			
			continue;
		}

		if(line == std::string("P=")) {
			double P[3][4], T[3];
			for(int i=0; i<3; ++i) for(int j=0; j<4; ++j)
				in >> P[i][j];
			//helper::print(3,4,P[0],"P");
			helper::decompose(P[0],K[0],R[0],T,C);
			readK = readR = readC = true;
			CisT = false;
		}
	}
	if( !(readK && readR && readC) ){
		cout<<"[readCameraFile error] Camera Parameter not enough!"<<endl;
		ret = 0;
	}

	using CV2CG::cv2cg;
	cv2cg(K,n,f,imgs,imgt, *ret);
	cv2cg(C,R,*ret,!CisT);

	std::string strid;
	helper::num2str(id,strid);
	ret->setName(std::string("cam")+strid);

	return ret;
}

inline osg::ref_ptr<osg::MatrixTransform>
create_FRUSTUM(const osg::Camera& camera,
					 osg::Image& image)
{
	osg::Matrix view = camera.getViewMatrix();
	double znear,zfar,nLeft,nRight,nTop,nBottom;
	camera.getProjectionMatrixAsFrustum(
		nLeft,nRight,nBottom,nTop,znear,zfar);
	const double constscale = 1.1;
	znear*=constscale;
	nLeft*=constscale;
	nRight*=constscale;
	nTop*=constscale;
	nBottom*=constscale;

	osg::Vec3Array* v = new osg::Vec3Array;
	v->push_back( osg::Vec3( nLeft, nBottom, -znear ) );
	v->push_back( osg::Vec3( nRight, nBottom, -znear ) );
	v->push_back( osg::Vec3( nRight, nTop, -znear ) );
	v->push_back( osg::Vec3( nLeft, nTop, -znear ) );
	v->push_back( osg::Vec3( 0., 0., 0. ) );

	osg::Vec2Array* vt = new osg::Vec2Array;
	vt->resize( 5 );
	(*vt)[0].set( 0,0 );
	(*vt)[1].set( 1,0 );
	(*vt)[2].set( 1,1 );
	(*vt)[3].set( 0,1 );
	(*vt)[4].set( 0,0 );

	osg::Geometry* geom = new osg::Geometry;
	geom->setVertexArray( v );
	geom->setTexCoordArray(0, vt);

	osg::Vec4Array* c = new osg::Vec4Array;
	c->push_back( osg::Vec4( 1., 1., 1., 1. ) );
	geom->setColorArray( c );
	geom->setColorBinding( osg::Geometry::BIND_OVERALL );

	GLushort idxLoops0[4] = {0, 1, 2, 3 };
	geom->addPrimitiveSet( new osg::DrawElementsUShort(
		osg::PrimitiveSet::QUADS, 4, idxLoops0 ) );

	osg::StateSet* stateset = geom->getOrCreateStateSet();
	osg::Texture2D* texture = new osg::Texture2D;
	texture->setImage(&image);
	stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
	//this enables the transparency of photo
	osg::BlendFunc* blend = new osg::BlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
	stateset->setAttributeAndModes(blend,osg::StateAttribute::ON);
	stateset->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

	osg::Geode* geode = new osg::Geode;
	geode->addDrawable( geom );

	osg::Geometry* lgeom = new osg::Geometry;
	lgeom->setVertexArray(v);
	lgeom->setColorArray( c );
	lgeom->setColorBinding( osg::Geometry::BIND_OVERALL );
	GLushort idxLines[8] = {0, 4, 1, 4, 2, 4, 3, 4};
	lgeom->addPrimitiveSet( new osg::DrawElementsUShort(
		osg::PrimitiveSet::LINES, 8, idxLines ));
	geode->addDrawable(lgeom);

	osgText::Text* t = new osgText::Text;
	t->setText(camera.getName());
	t->setPosition(v->at(4));
	t->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
	t->setCharacterSize(24.0f);
	t->setColor(osg::Vec4(1,0,0,1));
	t->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,
		osg::StateAttribute::OFF); // add this so can always see the name of the cam
	t->setAxisAlignment(osgText::Text::SCREEN);
	geode->addDrawable(t);

	geode->getOrCreateStateSet()->setMode(
		GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
	geode->getOrCreateStateSet()->setMode(
		GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );

	osg::MatrixTransform* mt = new osg::MatrixTransform;
	mt->setMatrix( osg::Matrixd::inverse( view ) );
	mt->addChild( geode );

	return mt;
}

/**
SYNTAX:
CAMERAFRUSTUM <#camerafrustum> [scale=1]

<#camerafrustum> lines of <imagepath> and <parameterpath>
	<imagepath> is the path to image file
	<parameterpath> is the path to parameter file
[scale] scale of camera frustum

EXAMPLE script:
CAMERAFRUSTUM 2
/home/simbaforrest/Data/tmp/
image1.jpg
iamge1.par
image2.jpg
image2.par
 **/
inline osg::ref_ptr<osg::Node> create_CAMERAFRUSTUM(std::istream& in,
	int ncameras, double scale=1) {
	if(ncameras<=0) return 0;

	osg::ref_ptr<osg::Group> ret = new osg::Group;
	ret->setName("CreateSceneHelper.CAMERAFRUSTUM");
	string dir;
	readValidLine(in, dir);
	helper::legalDir(dir);
	for(int id=0; id<ncameras; ++id) {
		osg::ref_ptr<osg::Image> img;
		osg::ref_ptr<osg::Camera> cam;

		string imgpath,parpath;
		readValidLine(in, imgpath);
		readValidLine(in, parpath);
		img = osgDB::readImageFile(dir+imgpath);
		if(!img.valid()) {
			loglni("[create_CAMERAFRUSTUM warn] skip invalid image.");
			continue;
		}

		std::ifstream parfile((dir+parpath).c_str());
		cam = create_CAMERA(parfile, img->s(), img->t(), id);
		if(!cam.valid()) {
			continue;
		}

		osg::ref_ptr<osg::MatrixTransform> mtf =
			create_FRUSTUM(*cam, *img);
		if(scale>0) mtf->preMult(osg::Matrix::scale(scale,scale,scale));
		mtf->setUserData(cam);
//		cam->setUserData(img);
		mtf->setName(std::string("frustum")+helper::num2str(id));
		ret->addChild(mtf);
	}
	return ret;
}

////////////////////////////////////////////////////////////////////////
inline osg::ref_ptr<osg::Node>
readBundlerFile(std::istream& in,
	std::vector<std::string> imgfilelist, double scale=1.0)
{
	osg::ref_ptr<osg::Group> ret = new osg::Group;
	ret->setName("CreateSceneHelper.BUNDLER");

	std::string str;
	while(helper::readValidLine(in, str)) {
		if(str[0]=='#') continue; //skip comment

		std::stringstream ss;
		ss << str;
		int nCam, nPt;
		ss >> nCam >> nPt;

		for(int i=0; i<nCam; ++i) {
			double f,k1,k2;
			double R[3][3],t[3],C[3],K[3][3]={{0}};
			in >> f >> k1 >> k2;
			for(int a=0; a<3; ++a)
				for(int b=0; b<3; ++b)
					in >> R[a][b];
			in >> t[0] >> t[1] >> t[2];
			if(f==0) continue;

			osg::ref_ptr<osg::Image> img = osgDB::readImageFile(imgfilelist[i]);
			if( !img.valid() ) continue;
			osg::ref_ptr<osg::Camera> cam = new osg::Camera;
			helper::zeros(3,3,K[0]);
			K[0][0]=K[1][1]=f;
			K[2][2]=1;
			K[0][2] = img->s()/2;
			K[1][2] = img->t()/2;
			helper::mulAtB(3,3,3,1,R[0],t,C);
			C[0]*=-1;C[1]*=-1;C[2]*=-1;
			helper::RotationMatrix_PH_CV(R[0]);
			cam->setViewport(0,0,500,500);
			CV2CG::cv2cg(K,C,R,0.1,1000,img->s(),img->t(),*cam);

			std::string strnum;
			helper::num2str(i, strnum);
			cam->setName(std::string("cam")+strnum);
			osg::ref_ptr<osg::MatrixTransform> mtf =
				create_FRUSTUM(*cam, *img);
			if(scale>0) mtf->preMult(osg::Matrix::scale(scale,scale,scale));
			mtf->setUserData(cam);
			cam->setUserData(img);
			mtf->setName(std::string("frustum")+strnum);
			ret->addChild(mtf);
		}

		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
		ret->addChild(geode);
		geode->addDrawable(geom);
		osg::ref_ptr<osg::Vec3Array> v3a = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
		geom->setVertexArray(v3a);
		geom->setColorArray(colors);
		geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
		osg::DrawArrays* da = new osg::DrawArrays(osg::PrimitiveSet::POINTS,
			0, nPt);
		geom->addPrimitiveSet(da);
		geom->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(3));
		geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		while(nPt--) {
			double x,y,z,r,g,b;
			in >> x >> y >> z >> r >> g >> b;
			v3a->push_back(osg::Vec3(x,y,z));
			colors->push_back(osg::Vec4(r/255.0,g/255.0,b/255.0,1));
			int nList, camId, key;
			double u, v;
			in >> nList;
			while(nList--)
				in >> camId >> key >> u >> v;
		}
	}
	return ret;
}

/**
SYNTAX:
BUNDLER <#images> [scale=1]

<#images> lines of <imagepath>
	<imagepath> is the path to image file
[scale] scale of camera frustum

EXAMPLE script:
BUNDLER 2
/home/simbaforrest/Data/tmp/
bundle.out
image1.jpg
image2.jpg
 **/
inline osg::ref_ptr<osg::Node> create_BUNDLER(std::istream& in,
	int nimages, double scale=1) {
	if(nimages<=0) return 0;

	string dir;
	readValidLine(in, dir);
	helper::legalDir(dir);
	string bundlefile;
	readValidLine(in, bundlefile);
	std::vector<std::string> imgfilelist;
	for(int id=0; id<nimages; ++id) {
		string imgpath;
		readValidLine(in, imgpath);
		imgfilelist.push_back(dir+imgpath);
	}

	std::ifstream fin((dir+bundlefile).c_str());
	return readBundlerFile(fin, imgfilelist, scale);
}

////////////////////////////////////////////////////////////////////////
/**
SYNTAX:
PHOTOBACKGROUND
<imagepath>

EXAMPLE script:
PHOTOBACKGROUND
/home/simbaforrest/Data/tmp/image1.jpg
 **/
inline osg::ref_ptr<osg::Camera> create_PHOTOBACKGROUND(std::istream& in) {
	std::string str;
	readValidLine(in, str);
	osg::ref_ptr<osg::Image> img = osgDB::readImageFile(str);
	if(!img.valid()) return 0;
	osg::ref_ptr<osg::Camera> hud = new helper::ARVideoBackground(img, true);
	hud->setName("CreateSceneHelper.PHOTOBACKGROUND");
	return hud;
}

////////////////////////////////////////////////////////////////////////
/**
SYNTAX:
ANIMATIONPATH <#keyframe>

<#keyframe> lines of keyframe, keyframe is a line of 1+9+3 numbers (time, R & T)

EXAMPLE script: (move to 10,10,10 at 10.5 second)
ANIMATIONPATH 2
1 1 0 0 0 1 0 0 0 1 0 0 0
10.5 1 0 0 0 1 0 0 0 1 10 10 10
 **/
inline osg::ref_ptr<osg::AnimationPath> create_ANIMATIONPATH(std::istream& in,
	int nkeyframes) {
	if(nkeyframes<=0) return 0;
	osg::ref_ptr<osg::AnimationPath> ret = new osg::AnimationPath;
	std::string str;
	for(int i=0; i<nkeyframes; ++i) {
		if(!readValidLine(in, str)) { std::cout<<"error!"<<std::endl; return 0; }
		double R[9], T[3], time;
		std::stringstream ss; ss<<str;
		ss >> time;
		for(int j=0; j<9; ++j) ss >> R[j];
		for(int j=0; j<3; ++j) ss >> T[j];
		osg::AnimationPath::ControlPoint cp(
			osg::Vec3d(T[0],T[1],T[2]),
			osg::Matrix(R[0],R[3],R[6],0,
						R[1],R[4],R[7],0,
						R[2],R[5],R[8],0,
						0,0,0,1).getRotate() );
		ret->insert(time,cp);
	}
	return ret;
}

////////////////////////////////////////////////////////////////////////
inline osg::ref_ptr<osg::Node> create_SCENE(string filename,
	osgViewer::Viewer& viewer) {
	std::ifstream fin(filename.c_str());
	osg::ref_ptr<osg::Group> ret = new osg::Group;
	ret->setName("CreateSceneHelper.SCENE");

	std::string str, cmd;
	while(readValidLine(fin,str)) {
		double arg[10]={-1};
		std::stringstream ss;
		ss << str; ss >> cmd;
		sscanf(str.c_str(), 
			"%*[^0-9-+.]%lf%*[^0-9-+.]%lf%*[^0-9-+.]%lf%*[^0-9-+.]%lf"
			"%*[^0-9-+.]%lf%*[^0-9-+.]%lf%*[^0-9-+.]%lf"
			"%*[^0-9-+.]%lf%*[^0-9-+.]%lf%*[^0-9-+.]%lf",arg,arg+1,
			arg+2,arg+3,arg+4,arg+5,arg+6,arg+7,arg+8,arg+9);
		if(cmd=="VERTEX") {
			int npoints = arg[0];
			bool lightflag = arg[1]>=0?arg[1]!=0:0;
			double pointsize = arg[2]>=0?arg[2]:3;
			int nlines = arg[3]>=0?arg[3]:0;
			double labelsize = arg[4]>=0?arg[4]:24;
			osg::ref_ptr<osg::Node> nd = create_VERTEX(fin,
				npoints,lightflag,pointsize,nlines,labelsize);
			if(nd.valid()) ret->addChild(nd);
		} else if(cmd=="MODEL") {
			int nmodels = arg[0];
			bool lightflag = arg[1]>0;
			bool scaleflag = arg[2]>0;
			bool translateflag = arg[3]>0;
			bool rotateflag = arg[4]>0;
			osg::ref_ptr<osg::Node> nd = create_MODEL(fin,
				nmodels,lightflag,scaleflag,translateflag,rotateflag);
			if(nd.valid()) ret->addChild(nd);
		} else if(cmd=="CAMERAFRUSTUM") {
			int ncameras = arg[0];
			double scale = arg[1]>0?arg[1]:1;
			osg::ref_ptr<osg::Node> nd = create_CAMERAFRUSTUM(fin,
				ncameras, scale);
			if(nd.valid()) ret->addChild(nd);
		} else if(cmd=="BUNDLER") {
			int nimages = arg[0];
			double scale = arg[1]>0?arg[1]:1;
			osg::ref_ptr<osg::Node> nd = create_BUNDLER(fin,
				nimages, scale);
			if(nd.valid()) ret->addChild(nd);
		} else if(cmd=="PHOTOBACKGROUND") {
			osg::ref_ptr<osg::Node> nd = create_PHOTOBACKGROUND(fin);
			if(nd.valid()) ret->addChild(nd);
		} else if(cmd=="ANIMATIONPATH") {
			int nkeyframes = arg[0];
			osg::ref_ptr<osg::AnimationPath> ap = 
				create_ANIMATIONPATH(fin,nkeyframes);
			if(ap.valid()) {
				osgGA::AnimationPathManipulator* manip = 
					new osgGA::AnimationPathManipulator(ap);
				viewer.setCameraManipulator(manip);
				cout<<"animationpath setted!"<<endl;
			}
		} else {
			cout<<"[create_SCENE] unknown cmd!"<<endl;
			return 0;
		}
	}
	return ret;
}

}//end of namespace CreateSceneHelper
