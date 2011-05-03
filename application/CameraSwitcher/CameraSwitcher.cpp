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

/* CameraSwitcher.cpp */

#include "CameraSwitcher.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <osg/NodeCallback>
#include <osgGA/GUIEventHandler>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Texture2D>
#include <osgViewer/Viewer>
#include <osg/NodeVisitor>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <osgText/Text>
#include <osg/AlphaFunc>
#include <osg/BlendColor>
#include <osg/BlendFunc>
#include <osg/BlendEquation>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
#include <osg/Point>

#include "Log.h"
#include "CV2CG.h"
#include "CameraUpdator.h"
#include "SwitchHandler.h"

int CameraSwitcher::run()
{
	osg::ref_ptr<osg::Node> scene =
		createSceneFromFile(infilename);
	if(writeModel)
		osgDB::writeNodeFile(*scene,
			infilename+std::string("_model.osg"));

	SwitchHandler::printHelp();

	osgViewer::Viewer viewer;
	viewer.getCamera()->setClearColor(osg::Vec4(0.1f,0.1f,0.3f,0.0f));
	viewer.setSceneData(scene);
	CameraUpdator* cu = new CameraUpdator(&viewer);
	SwitchHandler* sh = new SwitchHandler(cu);
	viewer.getCamera()->setUpdateCallback(cu);
	viewer.addEventHandler(sh);
	viewer.getCamera()->setComputeNearFarMode(
		osg::Camera::DO_NOT_COMPUTE_NEAR_FAR);

	//disable small feature culling
	osg::CullSettings::CullingMode cullingMode = viewer.getCamera()->getCullingMode();
	cullingMode &= ~(osg::CullStack::SMALL_FEATURE_CULLING);
	viewer.getCamera()->setCullingMode(cullingMode);

	viewer.addEventHandler( new osgGA::StateSetManipulator(
		viewer.getCamera()->getOrCreateStateSet()) );
	viewer.addEventHandler(new osgViewer::StatsHandler);

	viewer.setUpViewInWindow(50,50,500,500);

	return viewer.run();
}

/////////////////////////////////////////////////////
///   API
#define ENABLE_BUNDLER 1

osg::ref_ptr<osg::Node> createSceneFromFile(std::string filename)
{
#if ENABLE_BUNDLER // currently deprecated
	if(helper::getFileExtensionNoDot(filename)==std::string("bundler")) {
		return createSceneFromBundlerResult(filename);
	}
#endif
	using namespace std;

	std::string filedir = helper::getFileDir(filename);
	//LogI("file dir = "); LogA("\t%s\n",filedir.c_str());
	std::ifstream in(filename.c_str());

	osg::ref_ptr<osg::Group> ret = new osg::Group;
	ret->setName("Switcher.SceneRoot");

	std::string modelfile;
	helper::readValidLine(in,modelfile);
	TagI("parsed line {%s}\n",modelfile.c_str());
	modelfile = helper::getNameWithExtension(modelfile);

	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(filedir+modelfile);
	ret->addChild(model);

	osg::ref_ptr<osg::Camera> cam;
	osg::ref_ptr<osg::Image> img;
	double K[3][3]={{0}},C[3],R[3][3];
	double n=1,f=1000;

	bool atImageLine=true;
	int cnt=0;
	std::string line;
	while(helper::readValidLine(in,line)) {
		line = helper::getNameWithExtension(line);
		TagI("parsed line {%s}\n",line.c_str());

		std::string filename = filedir+line;

		if(atImageLine) {
			img = osgDB::readImageFile(filename);
		} else {
			if( !img.valid() ) {
				TagE("no valid image!\n"); continue;
			}
			if(	!(cam = readCameraFile(filename, K, C, R, n, f)) ) {
				continue;
			}

			cv2cg(K,C,R,n,f, img->s(), img->t(), *cam);

			std::string strnum;
			helper::num2str(cnt++,strnum);
			cam->setName(std::string("cam")+strnum);

			osg::ref_ptr<osg::MatrixTransform> mtf =
				Make_Photo(*cam, *img);
			mtf->setUserData(cam);
			cam->setUserData(img);
			mtf->setName(std::string("MatTrans")+strnum);
			ret->addChild(mtf);

			img = 0;
			cam = 0;
		}

		atImageLine = !atImageLine;
	}

	return ret;
}

osg::ref_ptr<osg::Camera> readCameraFile(std::string str,
	double K[3][3], double C[3], double R[3][3],
	double& n, double& f)
{
	using std::cout;
	using std::endl;
	std::ifstream in(str.c_str());
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
			continue;
		}

		if(line == std::string("T=")) {
			double T[3];
			in >> T[0] >> T[1] >> T[2];
			//helper::print(3,1,T,"T");
			helper::mulAtB(3,3,3,1,R[0],T,C);
			C[0]*=-1;C[1]*=-1;C[2]*=-1;
			readC=true;
			continue;
		}

		if(line == std::string("P=")) {
			double P[3][4], T[3];
			for(int i=0; i<3; ++i) for(int j=0; j<4; ++j)
				in >> P[i][j];
			//helper::print(3,4,P[0],"P");
			helper::decompose(P[0],K[0],R[0],T,C);
			readK = readR = readC = true;
		}
	}
	if( !(readK && readR && readC) ){
		//if(ret->getViewport()==0)
		//	ret->setViewport(0,0,500,500);
		TagE("Camera Parameter not enough!\n");
		ret = 0;
	}
	return ret;
}

osg::ref_ptr<osg::MatrixTransform>
Make_Photo(const osg::Camera& camera,
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

#if ENABLE_BUNDLER
osg::ref_ptr<osg::Node>
createSceneFromBundlerResult(std::string filename)
{
	std::string filedir = helper::getFileDir(filename);
	std::ifstream in(filename.c_str());

	std::string bundlerfile;
	in >> bundlerfile;
	bundlerfile = filedir + bundlerfile;

	std::vector<std::string> imgfilelist;
	std::string line;
	while(helper::readValidLine(in, line)) {
		imgfilelist.push_back(filedir+line);
	}

	return readBundlerFile(bundlerfile, imgfilelist);
}

osg::ref_ptr<osg::Node>
readBundlerFile(std::string filename,
	std::vector<std::string> imgfilelist)
{
	osg::ref_ptr<osg::Group> ret = new osg::Group;

	std::ifstream in(filename.c_str());
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
			cv2cg(K,C,R,0.1,100,img->s(),img->t(),*cam);
			//osg::Vec3 center(C[0],C[1],C[2]);
			//osg::Vec3 lookdir(-R[2][0],-R[2][1],-R[2][2]);
			//osg::Vec3 up(R[1][0],R[1][1],R[1][2]);
			//cam->setViewMatrixAsLookAt(center, center+lookdir*10, up);

			std::string strnum;
			helper::num2str(i, strnum);
			cam->setName(std::string("cam")+strnum);
			osg::ref_ptr<osg::MatrixTransform> mtf =
				Make_Photo(*cam, *img);
			mtf->setUserData(cam);
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
	std::cout<<filename<<std::endl;
	osgDB::writeNodeFile(*ret, filename+std::string(".osg"));
	return ret;
}
#endif
