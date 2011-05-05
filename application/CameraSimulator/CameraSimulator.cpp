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

/* CameraSimulator.cpp */

#include "CameraSimulator.h"

#include "SimulatorAPI.h"

#include "Log.h"
#include "OpenCVHelper.h"
#include "CV2CG.h"
#include "Reader.h"

using namespace CV2CG;

osg::ref_ptr<osg::Vec3Array> v3a;
osg::ref_ptr<osg::Node> scene;

osg::ref_ptr<osg::Node> createScene(std::string filename)
{
#ifdef USE_IN_CHINA
	setlocale(LC_ALL,"chs");
#endif
	std::ifstream fin(filename.c_str());

	osg::ref_ptr<osg::Group> ret = new osg::Group;
	ret->setName("CameraSimulator.SceneRoot");
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->setName("CameraSimulator.VertexLineGeode");
	ret->addChild(geode);

	bool addText = true;
	double textSize = 24.0;

	v3a = new osg::Vec3Array;//
	std::string str;
	while(helper::readValidLine(fin,str)) {

		std::string type;
		double num;
		std::stringstream ss;
		ss << str;
		ss >> type >> num;

		if(type == "VERTEX") {
			v3a = ReadVertex((int)num, fin);
			osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
			osg::ref_ptr<osg::DrawArrays> points =
				new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, v3a->size());
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
			colors->push_back(osg::Vec4(1,0,0,1));
			geom->setVertexArray(v3a);
			geom->setColorArray(colors);
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			geom->addPrimitiveSet(points);
			geom->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(3));
			geode->addDrawable(geom);
		} else if(type == "LINE") {
			osg::ref_ptr<osg::Geometry> geom = ReadLine((int)num, fin);
			geode->addDrawable(geom);
			geom->setVertexArray(v3a);
		} else if(type == "MODEL") {
			osg::ref_ptr<osg::Group> group = ReadModel((int)num, fin);
			ret->addChild(group);
		} else if(type == "NOTEXT") {
			addText = false;
		} else if(type == "TEXTSIZE") {
			textSize=num;
			TagI("TEXTSIZE=%lf\n",textSize);
		} else {
			TagW("ignore unknown type : %s\n", type.c_str());
		}
	}
	geode->getOrCreateStateSet()->setMode(
		GL_LIGHTING, osg::StateAttribute::OFF);

	if(addText)
	{
		for(unsigned int i=0; i<v3a->size(); ++i) {
			osgText::Text* t = new osgText::Text;
			std::stringstream ss;
			ss << i;
			std::string str;
			ss >> str;
			t->setText(str);
			t->setPosition(v3a->at(i));
			t->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			t->setCharacterSize(textSize);
			t->setAxisAlignment(osgText::Text::SCREEN);
			geode->addDrawable(t);
		}
	}

	return ret;
}

double kkk=-0.01;

struct SnapImage : public osg::Camera::DrawCallback
{
	std::ofstream& allfile;
	SnapImage(const std::string& filename, std::ofstream& file):
        _filename(filename),
		_snapImage(false),
		allfile(file)
	{
		_image = new osg::Image;
	}

	virtual void operator () (osg::RenderInfo& renderInfo) const
	{
		if (!_snapImage) return;

		osg::Camera* camera = renderInfo.getCurrentCamera();
		osg::Viewport* viewport = camera ? camera->getViewport() : 0;

		static int cnt=0;

		if (viewport && _image.valid())
		{
// 			Distort_Image(_image, kkk, viewport->width()/2, viewport->height()/2,
// 				int(viewport->width()),int(viewport->height()));

#ifdef USE_IN_CHINA
	setlocale(LC_ALL,"chs");
#endif
			std::string strnum;
			helper::num2str(cnt,strnum);
			std::string parName = _filename+strnum+std::string(".par");
			std::string cpName = _filename+strnum+std::string(".cp");
			std::string pngName = _filename+strnum+std::string(".png");
			allfile << helper::getNameWithExtension(pngName) << std::endl;
			allfile << helper::getNameWithExtension(parName) << std::endl;

			//write CG&&CV parameters
			std::ofstream out(parName.c_str());
			CV_CG_Report(*camera, out);
			out.close();

			//write control points for calibration
			std::ofstream outCP(cpName.c_str());
			CG_Project_Report(*camera, v3a, outCP);
			out.close();

			//write image
			_image->readPixels(/*0,0,*/int(viewport->x()),int(viewport->y()),
				int(viewport->width()),int(viewport->height()),
				GL_RGBA,
				GL_UNSIGNED_BYTE);
			osgDB::writeImageFile(*_image, pngName);

			LogI("%d: Taken screenshot! Results are"
				" written to '%s'\n", cnt, _filename.c_str());
			++cnt;
		}
		_snapImage = false;
	}

	std::string                         _filename;
	mutable bool                        _snapImage;
	mutable osg::ref_ptr<osg::Image>    _image;
};

struct SnapeImageHandler : public osgGA::GUIEventHandler
{
	SnapeImageHandler(int key,SnapImage* si):_key(key),_snapImage(si) {}

	osg::Camera* camera;

	double _dist;
	osg::Vec3 _center;

	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		if (ea.getHandled()) return false;
		osgViewer::View* viewer = dynamic_cast<osgViewer::View*>(&aa);
		if(!viewer) return false;

		camera = viewer->getCamera();

		switch(ea.getEventType())
		{
		case(osgGA::GUIEventAdapter::KEYDOWN):
		{
			//hold s and drag : do not change eye position, only change rotation
			if(ea.getKey()=='s') {
				osgGA::TrackballManipulator* manip =
					dynamic_cast<osgGA::TrackballManipulator*>(
					viewer->getCameraManipulator() );
				if(/*_lock ||*/ !manip) return true;
				//_lock = true;
				_dist = manip->getDistance();
				_center = manip->getCenter();
				manip->setDistance(0);
				osg::Vec3 eye,cen,up;
				viewer->getCamera()->getViewMatrixAsLookAt(eye, cen, up);
				manip->setCenter(eye);
				return true;
			}
			return false;
		}
		case(osgGA::GUIEventAdapter::KEYUP):
			{
				const int tmpKey = ea.getKey();
				if(tmpKey=='s') {
					osgGA::TrackballManipulator* manip =
						dynamic_cast<osgGA::TrackballManipulator*>(
						viewer->getCameraManipulator() );
					if(!manip) return true;
					//_lock = false;
					manip->setDistance(_dist);
					manip->setCenter(_center);
					viewer->home();
					return true;
				}
				if(tmpKey == _key)
				{
					_snapImage->_snapImage = true; return true;
				}
				if(tmpKey == 'h')
				{
					printUsage(); return true;
				}
				if(tmpKey == 'g')
				{
					CG_Report(*camera); return true;
				}
				if(tmpKey == 'v')
				{
					CV_Report(*camera); return true;
				}
				double dx,dy;
				dy = (tmpKey == osgGA::GUIEventAdapter::KEY_Up) ? -0.01 :
					( (tmpKey == osgGA::GUIEventAdapter::KEY_Down) ? 0.01 : 0 );
				dx = (tmpKey == osgGA::GUIEventAdapter::KEY_Right) ? -0.01 :
					( (tmpKey == osgGA::GUIEventAdapter::KEY_Left) ? 0.01 : 0 );
				if(!(dx==0 && dy==0)) {
					Modify_PrinciplePoint(*camera, dx, dy);
					return true;
				}
				return false;
			}
		case(osgGA::GUIEventAdapter::SCROLL):
			{
				double l,r,t,b,n,f;
				camera->getProjectionMatrixAsFrustum(l,r,b,t,n,f);
				if (ea.getScrollingMotion()==
					osgGA::GUIEventAdapter::SCROLL_UP) {
						n*=0.9;/* kkk*=0.5;*/
						std::cout<<n<<std::endl;
				} else if (ea.getScrollingMotion()==
					osgGA::GUIEventAdapter::SCROLL_DOWN) {
						n/=0.9;/* kkk*=2;*/
						std::cout<<n<<std::endl;
				}
				camera->setProjectionMatrixAsFrustum(l,r,b,t,n,f);
				return true;
			}
		}

		return false;
	}

	int                     _key;
	osg::ref_ptr<SnapImage> _snapImage;
};

int CameraSimulator::run()
{
	scene = createScene(inputFileName);
	if (!scene)
	{
		TagE("No model Created\n");
		return 1;
	}

	std::string swtfilename = (outputFileName+std::string(".swt"));
	std::string outfilename_nodir = helper::getNameWithExtension(outputFileName);
	std::ofstream allfile(swtfilename.c_str());
	allfile << outfilename_nodir+std::string("_model.osg") << std::endl;

	//write model
	osgDB::writeNodeFile(*scene, outputFileName+std::string("_model.osg"));

	osgViewer::Viewer viewer;

	SnapImage* postDrawCallback = new SnapImage(outputFileName, allfile);
	viewer.getCamera()->setPostDrawCallback(postDrawCallback);
	SnapeImageHandler* handler = new SnapeImageHandler('p',postDrawCallback);
	handler->camera = viewer.getCamera();
	viewer.addEventHandler(handler);

	viewer.getCamera()->setComputeNearFarMode(
		osg::Camera::DO_NOT_COMPUTE_NEAR_FAR);

	viewer.setSceneData(scene);

	printUsage(false);

	viewer.setUpViewInWindow(50,50,500,500);

	viewer.getCamera()->setClearColor(osg::Vec4(0,0.1,0.3,1));

	return viewer.run();
}
