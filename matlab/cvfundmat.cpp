#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "Log.h"
#include "Log.hxx"
#include "OpenCVHelper.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	if(argc<3) {
		cout<<"[usage]\n\t"<<argv[0]<<" <matched_points_file> "
			"<out_F_matrix_file> [out_inlier_file]"<<endl;
		return -1;
	}
	
	//matched image points
	vector<Point2f> p1, p2;
	//inliers in p1-p2 for fmatrix, 0 means outlier
	vector<uchar> inliers;

	std::ifstream in(argv[1]);
	if(!in) {
		cout<<"[cvfundmat] can not open "<<argv[1]<<endl;
		return -1;
	}
	string str;
	while(helper::readValidLine(in,str)) {
		if(str[0]=='#') continue;
		
		float u1,v1,u2,v2;
		sscanf(str.c_str(), 
			"%f%*[^0-9-+.eE]%f%*[^0-9-+.eE]%f%*[^0-9-+.eE]%f",
			&u1,&v1,&u2,&v2);

		p1.push_back(Point2d(u1,v1));
		p2.push_back(Point2d(u2,v2));
	}
	in.close();

	Mat fmat = findFundamentalMat(Mat(p1), Mat(p2), inliers);
	double F[9];
	std::copy(fmat.begin<double>(), fmat.end<double>(), F);

	helper::WriteFile(3,3,F,argv[2]);

	if(argc>3) {
		std::ofstream outI(argv[3]);
		for(int i=0; i<(int)inliers.size(); ++i) {
			outI << (int)inliers[i] << " ";
		}
		outI<<endl;
		outI.close();
	}

	return 0;
}
