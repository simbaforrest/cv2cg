#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "Log.hxx"
#include "Log.h"
#include "OpenCVHelper.h"

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
#include <osg/NodeCallback>
#include <osg/Point>
#include "CV2CG.h"

using namespace std;
using namespace CV2CG;

osgViewer::Viewer viewer;

class PrintViewMatUpdator : public osg::NodeCallback {
	int cnt;
	public:
		PrintViewMatUpdator() : cnt(0) {}
		void operator()(osg::Node* node, osg::NodeVisitor* nv) {
			cout<<"frame="<<cnt++<<endl;
			cout<<(viewer.getCamera()->getViewMatrix())<<endl;
		}
};

void printtest(double R[])
{
	cout<<"R=\n"<<helper::PrintMat<>(3,3,R)<<endl;
}

int main() {
	double R[] = {
0.844733200000000,	-0.505792000000000,	0.174929300000000,
-0.267952500000000,	-0.682641800000000,	-0.679854100000000,
0.463278900000000,	0.527422600000000,	-0.712178400000000 };

	double U[9], S[9], VT[9];
	helper::svd(3,3,R,U,S,VT);
	
	cout<<"R=\n"<<helper::PrintMat<>(3,3,R)<<endl;
	cout<<"U=\n"<<helper::PrintMat<>(3,3,U)<<endl;
	cout<<"S=\n"<<helper::PrintMat<>(3,3,S)<<endl;
	cout<<"VT=\n"<<helper::PrintMat<>(3,3,VT)<<endl;

	double dotret = helper::dot(3,3,R,R+3);
	cout<<"dot(R(1,:),R(2,:))="<<dotret<<endl;

	cout<<"norm(R(1,:))="<<helper::normL2(1,3,R)<<endl;

	double Q[4];
	helper::mat2quat(R,Q);
	cout<<"Q(R)=\n"<<helper::PrintMat<>(1,4,Q)<<endl;

//	osg::ref_ptr<osg::Node> node = osgDB::readNodeFile("axes.osg");
//	viewer.setSceneData(node);
//	node->setUpdateCallback(new PrintViewMatUpdator);
//	viewer.run();
	printtest({1,2,3,4,5,6,7,8,9});
	return 0;
}
