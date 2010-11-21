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

/* SwitcherAPI.cpp */

#include "SwitcherAPI.h"
#include <osg/Point>

void Interpolate_Camera(const osg::Camera& s, const osg::Camera& e,
												const double t, osg::Camera& camera)
{
	//view matrix
	const osg::Matrix& vms = s.getViewMatrix();
	const osg::Matrix& vme = e.getViewMatrix();
	osg::Quat qs = vms.getRotate();
	osg::Quat qe = vme.getRotate();
	osg::Vec3 transS = vms.getTrans();
	osg::Vec3 transE = vme.getTrans();

	osg::Quat q;
	osg::Vec3 trans;
	q.slerp(t, qs, qe);
	trans = Interpolate(transS,transE,t);
	osg::Matrix vmat;
	vmat.setRotate(q);
	vmat.setTrans(trans);
	camera.setViewMatrix(vmat);

	//proj matrix
	double ls,rs,bs,ts,ns,fs;
	s.getProjectionMatrixAsFrustum(ls,rs,bs,ts,ns,fs);
	double le,re,be,te,ne,fe;
	e.getProjectionMatrixAsFrustum(le,re,be,te,ne,fe);
	double lt,rt,bt,tt,nt,ft;
	lt = Interpolate(ls,le,t);
	rt = Interpolate(rs,re,t);
	bt = Interpolate(bs,be,t);
	tt = Interpolate(ts,te,t);
	nt = Interpolate(ns,ne,t);
	ft = Interpolate(fs,fe,t);
	camera.setProjectionMatrixAsFrustum(lt,rt,bt,tt,nt,ft);
}

osg::ref_ptr<osg::MatrixTransform>
Make_Photo(const osg::Camera& camera,
					 osg::Image& image)
{
	osg::Matrix proj = camera.getProjectionMatrix();
	osg::Matrix view = camera.getViewMatrix();

	const double znear = proj(3,2) / (proj(2,2)-1.0) * 1.1;

	const double nLeft = znear * (proj(2,0)-1.0) / proj(0,0);
	const double nRight = znear * (1.0+proj(2,0)) / proj(0,0);
	const double nTop = znear * (1.0+proj(2,1)) / proj(1,1);
	const double nBottom = znear * (proj(2,1)-1.0) / proj(1,1);

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
	t->setCharacterSize(48.0f);
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

osg::ref_ptr<osg::Camera> 
readCameraFile(std::string str, double n, double f)
{
	std::ifstream in(str.c_str());
	osg::ref_ptr<osg::Camera> ret = new osg::Camera;
	//ret->setName(str);
	double K[3][3]={0},C[3],R[3][3];
	bool readK,readC,readR;
	readK=readC=readR=false;
	while(in) {
		if(readK && readR && readC)
			break;

		std::string str;
		std::getline(in, str, '\n');
		if(str.length()==0) continue;

		if(str.find("n=")!=str.npos) {
			std::stringstream ss;
			ss << str;
			std::string tmp;
			while(true) {
				ss >> tmp;
				if(tmp.find("n=")!=tmp.npos) {
					sscanf(tmp.c_str(),"n=%lf", &n);
					break;
				}
			}
		}

		if(str.find("f=")!=str.npos) {
			std::stringstream ss;
			ss << str;
			std::string tmp;
			while(true) {
				ss >> tmp;
				if(tmp.find("f=")!=tmp.npos) {
					sscanf(tmp.c_str(),"f=%lf", &f);
					break;
				}
			}
		}

		if(str.find_first_of("x=")==0) {
			double x,y,w,h;
			sscanf(str.c_str(),
				"x=%lf y=%lf w=%lf h=%lf",
				&x,&y,&w,&h
				);
			osg::ref_ptr<osg::Viewport> vp = new osg::Viewport(x,y,w,h);
			ret->setViewport(vp);
		}

		if(str == std::string("K(alphaX alphaY u0 v0)=")) {
			in >> K[0][0] >> K[1][1] >> K[0][2] >> K[1][2];
			readK=true;
			continue;
		}

		if(str == std::string("R=")) {
			for(int i=0; i<3; ++i)
				for(int j=0; j<3; ++j)
					in >> R[i][j];
			readR=true;
			continue;
		}

		if(str == std::string("C=")) {
			in >> C[0] >> C[1] >> C[2];
			readC=true;
			continue;
		}

		if(str == std::string("T=")) {
			double T[3];
			in >> T[0] >> T[1] >> T[2];
			double Rt[3][3];
			MatrixManip::Transpose(3,3,R[0],Rt[0]);
			MatrixManip::Product331(Rt[0], T, C);
			C[0]*=-1;C[1]*=-1;C[2]*=-1;
			readC=true;
			continue;
		}

		if(str == std::string("P=")) {
			double P[3][4], T[3];
			for(int i=0; i<3; ++i) for(int j=0; j<4; ++j)
				in >> P[i][j];
			MatrixManip::Print(3,4,P[0],"P");
			if( !CameraAlgebra::Decompose(P, K, R, C, T) ) {
				std::cout<<"Camera Decomposition failed! exit!"<<std::endl;
				system("pause");
				exit(-1);
			}
			readK = readR = readC = true;
		}
	}
	if(readK && readR && readC) {
		if(ret->getViewport()==0)
			ret->setViewport(0,0,500,500);
		Camera_Composition(K,C,R,n,f,0,*ret);
	}
	return ret;
}

osg::ref_ptr<osg::Node> createSceneFromFile(std::string filename)
{
	if(osgDB::getFileExtension(filename)==std::string("bundler")) {
		return createSceneFromBundlerResult(filename);
	}

	std::string filedir = osgDB::getFilePath(filename);
	filedir+=std::string("\\");
	std::ifstream in(filename.c_str());

	osg::ref_ptr<osg::Group> ret = new osg::Group;

	std::string modelfile;
	in >> modelfile;

	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(filedir+modelfile);
	ret->addChild(model);

	osg::ref_ptr<osg::Camera> cam;
	osg::ref_ptr<osg::Image> img;

	bool atImageLine=true;
	int cnt=0;
	while(in) {
		std::string str;
		std::getline(in, str, '\n');
		if(str.length()==0) continue;

		if(atImageLine) {
			img = osgDB::readImageFile(filedir+str);
		} else {
			cam = readCameraFile(filedir+str);
			std::stringstream ss;
			ss << cnt;
			++cnt;
			std::string strnum;
			ss >> strnum;
			cam->setName(std::string("cam")+strnum);
			osg::ref_ptr<osg::MatrixTransform> mtf = 
				Make_Photo(*cam, *img);
			mtf->setUserData(cam);
			ret->addChild(mtf);
		}

		atImageLine = !atImageLine;
	}

	return ret;
}

osg::ref_ptr<osg::Node>
createSceneFromBundlerResult(std::string filename)
{
	std::string filedir = osgDB::getFilePath(filename);
	filedir+=std::string("\\");
	std::ifstream in(filename.c_str());

	std::string bundlerfile;
	in >> bundlerfile;
	bundlerfile = filedir + bundlerfile;

	std::vector<std::string> imgfilelist;
	while(in) {
		std::string str;
		std::getline(in, str, '\n');
		if(str.length()==0) continue;

		imgfilelist.push_back(filedir+str);
	}

	return readBundlerFile(bundlerfile, imgfilelist);
}

osg::ref_ptr<osg::Node>
readBundlerFile(std::string filename,
								std::vector<std::string> imgfilelist)
{
	osg::ref_ptr<osg::Group> ret = new osg::Group;

	std::ifstream in(filename.c_str());
	while(in) {
		std::string str;
		std::getline(in, str, '\n');
		if(str.length()==0) continue;
		if(str[0]=='#') continue;

		std::stringstream ss;
		ss << str;
		int nCam, nPt;
		ss >> nCam >> nPt;

		for(int i=0; i<nCam; ++i) {
			double f,k1,k2;
			double R[3][3],t[3],C[3],K[3][3]={0};
			in >> f >> k1 >> k2;
			for(int a=0; a<3; ++a)
				for(int b=0; b<3; ++b)
					in >> R[a][b];
			in >> t[0] >> t[1] >> t[2];
			if(f==0) continue;

			osg::ref_ptr<osg::Image> img = osgDB::readImageFile(imgfilelist[i]);
			if( !img.valid() ) continue;
			osg::ref_ptr<osg::Camera> cam = new osg::Camera;
			MatrixManip::Zeros(3,3,K[0]);
			K[0][0]=K[1][1]=f;
			K[2][2]=1;
			K[0][2] = img->s()/2;
			K[1][2] = img->t()/2;
			MatrixManip::ProductAtB(3,3,3,1,R[0],t,C);
			C[0]*=-1;C[1]*=-1;C[2]*=-1;
			//CameraAlgebra::RotationMatrix_PH_CV(R[0]);
			cam->setViewport(0,0,500,500);
			Camera_Composition(K,C,R,1,1000,0,*cam);
			osg::Vec3 center(C[0],C[1],C[2]);
			osg::Vec3 lookdir(-R[2][0],-R[2][1],-R[2][2]);
			osg::Vec3 up(R[1][0],R[1][1],R[1][2]);
			cam->setViewMatrixAsLookAt(center, center+lookdir*10, up);

			std::stringstream ss;
			ss << i;
			std::string strnum;
			ss >> strnum;
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
	return ret;
}