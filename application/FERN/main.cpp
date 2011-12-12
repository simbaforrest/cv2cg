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

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "OpenCVHelper.h"
#include "Log.h"

using helper::ImageSource;
using namespace std;
using namespace cv;

Log::Level Log::level = Log::LOG_INFO;

struct FERNSprocessor : public ImageSource::Processor {
	Mat object; //template
	LDetector ldetector;
	PlanarObjectDetector detector;
	vector<Mat> objpyr, imgpyr;
	Size patchSize;
	vector<KeyPoint> objKeypoints, imgKeypoints;
	PatchGenerator gen;
	vector<Point2f> cpts; //template corners
	double K[9];
	bool savekeys;
	Mat H; //homography

	int framecnt;

	struct KeyFrame {
		int id;
		Mat frame;
		double R[3][3];
		double T[3];
		double rms,ncc;
		double duration; //time to track
	};
	vector<KeyFrame> keyframes;

	FERNSprocessor() : patchSize(32, 32),
			ldetector(7, 20, 2, 2000, patchSize.width, 2),
			gen(0,256,5,true,0.8,1.2,-CV_PI/2,CV_PI/2,-CV_PI/2,CV_PI/2) {
		ldetector.setVerbose(true);
		savekeys = false;
		framecnt = 0;
	}

	void loadK(double const kmat[9]) {
		for(int i=0; i<9; ++i) K[i]=kmat[i];
	}

	void loadtemplate(std::string name) {
		object = imread( name, CV_LOAD_IMAGE_GRAYSCALE );
		if(object.empty()) {
			loglne("Can not load template image, exit!");
			exit(-1);
		}
		buildPyramid(object, objpyr, ldetector.nOctaves-1);

		string model_filename = name + "_model.xml.gz";
		loglni("[loadtemplate] try to load "<<model_filename<<"...");
		FileStorage fs(model_filename, FileStorage::READ);
		if( fs.isOpened() ) {
			detector.read(fs.getFirstTopLevelNode());
			loglni("[loadtemplate] successfully loaded "<<model_filename.c_str());
		} else {
			loglni("[loadtemplate] try to train the model...");
			ldetector.setVerbose(true);
			ldetector.getMostStable2D(object, objKeypoints, 100, gen);
			detector.setVerbose(true);
			detector.train(objpyr, objKeypoints, patchSize.width,
				100, 11, 10000, ldetector, gen);
			loglni("[loadtemplate] training DONE! saving...");
			if( fs.open(model_filename, FileStorage::WRITE) )
				detector.write(fs, "ferns_model");
		}
		fs.release();

		cpts.push_back(Point2f(0,0));
		cpts.push_back(Point2f(object.cols,0));
		cpts.push_back(Point2f(object.cols,object.rows));
		cpts.push_back(Point2f(0,object.rows));
	}

	inline bool validateH() const {
		vector<Point2f> dstCrns(4);
		Mat tmpmat(dstCrns);
		perspectiveTransform(Mat(cpts), tmpmat, H);
		return validateH(dstCrns);
	}

	inline bool validateH(const vector<Point2f>& dstCrns) const {
		double tmpdir[3][2]= {
			{dstCrns[1].x-dstCrns[0].x, dstCrns[1].y-dstCrns[0].y},
			{dstCrns[2].x-dstCrns[0].x, dstCrns[2].y-dstCrns[0].y},
			{dstCrns[3].x-dstCrns[0].x, dstCrns[3].y-dstCrns[0].y}
		};
		double d12 = helper::cross2D(tmpdir[0],tmpdir[1]) * 0.5;
		double d23 = helper::cross2D(tmpdir[1],tmpdir[2]) * 0.5;
		double area = abs(d12+d23);
		bool s123 = d12*d23>0;
		return s123 && area>=64;
	}

	void evaluateH(const Mat& gray, const Mat& temp,
			double& rms, double& zncc) {
		Mat fltemp;
		temp.convertTo(fltemp,CV_32F);
		Mat warp;
		warpPerspective(gray,
			warp, H,
			temp.size(), INTER_LINEAR|WARP_INVERSE_MAP);
		warp.convertTo(warp,CV_32F);
		Mat error = warp-fltemp;
		Mat mask = warp!=0;
		rms = helper::rms<float>(error, &mask);
		zncc = helper::zncc<float>(warp,fltemp,&mask);
	}

	inline void GetCameraPose(double R[3][3], double T[3],
		bool verbose=true, bool doAR=false) {
		double Homo[9];
		std::copy(H.begin<double>(), H.end<double>(), Homo);
		if(doAR) { // R = R * [0,1,0;1,0,0;0,0,-1]
			double tmp[9];
			CameraHelper::RTfromKH(K,Homo,tmp,T);
			double R1[9]= {0,1,0,1,0,0,0,0,-1};
			helper::mul(3,3,3,3,tmp,R1,R[0]);
		} else {
			CameraHelper::RTfromKH(K,Homo,R[0],T);
		}
		if(verbose) {
			cout<<"R=\n"<<helper::PrintMat<>(3,3,R[0])<<endl;
			cout<<"T=\n"<<helper::PrintMat<>(1,3,T)<<endl;
		}
	}

	inline void draw3D(Mat &image) {
		static double crns[8][3] = {
			{0, 0, 0},
			{0, object.cols, 0},
			{object.rows, object.cols, 0},
			{object.rows, 0, 0},
			{object.rows*0.4, object.cols*0.4, object.rows*0.5},
			{object.rows*0.4, object.cols*0.6, object.rows*0.5},
			{object.rows*0.6, object.cols*0.6, object.rows*0.5},
			{object.rows*0.6, object.cols*0.4, object.rows*0.5}
		};
		//homo to P
		double R[3][3],T[3];
		GetCameraPose(R,T,false,true);
		helper::drawPyramid(image,K,R[0],T,crns);
	}

	inline void CapKeyFrame(int id, Mat const& frame,
			double const R[3][3], double const T[3],
			double const rms, double const ncc
			, double const dur) {
		KeyFrame newkf;
		newkf.id = id;
		newkf.frame = frame.clone();
		newkf.duration = dur;
		for(int i=0; i<3; ++i) {
			newkf.T[i] = T[i];
			for(int j=0; j<3; ++j) {
				newkf.R[i][j]=R[i][j];
			}
		}
		newkf.rms = rms;
		newkf.ncc = ncc;
		keyframes.push_back(newkf);
	}

	inline bool SaveKeyFrames(string name) {
		cout<<"[SaveKeyFrames] "<<(int)keyframes.size()<<" frame(s) total!"<<endl;
		if(keyframes.size()<=0) return true;
		name = DirHelper::getFileDir(name);
		string mainname = helper::legalDir(name) + string("frames.main");
		string rmsnccname = name + string("frames.rmsncc");
		string framesRname = name + string("frames.R");
		string framesTname = name + string("frames.T");
		string framesDURname = name + string("frames.DUR");
		std::ofstream mainout(mainname.c_str());
		mainout << "CAMERAFRUSTUM "<<keyframes.size()<<" 1"<< endl;
		mainout << name << endl;
		std::ofstream rmsncc(rmsnccname.c_str());
//		rmsncc <<"#format: frame_id rms ncc"<<endl;
		std::ofstream framesR(framesRname.c_str());
//		framesR <<"#format: frame_id R[0][0] R[0][1] ... R[2][2]"<<endl;
		std::ofstream framesT(framesTname.c_str());
//		framesT <<"#format: frame_id T[0] T[1] T[2]"<<endl;
		std::ofstream framesDUR(framesDURname.c_str());;
//		framesDUR <<"#format: frame_id dur"<<endl;
		for(int i=0; i<(int)keyframes.size(); ++i) {
			const KeyFrame& kf = keyframes[i];
			if(kf.frame.empty()) {
				continue;
			}
			string num;
			helper::num2str(kf.id,num);
			string prefix = name + string("frame") + num;
			string relativePrefix = string("frame") + num;

			string parname = prefix + string(".par");
			std::ofstream par(parname.c_str());
			par << "n=0.1\nf=10000\n" << endl;
			par << "K=\n" << helper::PrintMat<>(3,3,K) << endl;
			par << "R=\n" << helper::PrintMat<>(3,3,kf.R[0]) << endl;
			par << "T=\n" << helper::PrintMat<>(3,1,kf.T) << endl;
			par.close();
			cout<<"[SaveKeyFrames] "<<parname<<" saved."<<endl;

			string imgname = prefix + string(".jpg");
			imwrite(imgname, kf.frame);
			cout<<"[SaveKeyFrames] "<<imgname<<" saved."<<endl;

			string relativeParname = relativePrefix + string(".par");
			string relativeImgname = relativePrefix + string(".jpg");
			mainout << relativeImgname << endl;
			mainout << relativeParname << endl;

			rmsncc <<kf.id<<" "<<kf.rms<<" "<<kf.ncc<<endl;
			framesR <<kf.id<<" "<<helper::PrintMat<>(1,9,kf.R[0]);
			framesT <<kf.id<<" "<<helper::PrintMat<>(1,3,kf.T);
			framesDUR <<kf.id<<" "<<kf.duration<<endl;
		}
		rmsncc.close();
		framesR.close();
		framesT.close();
		mainout.close();
		framesDUR.close();
		cout<<"[SaveKeyFrames] DONE!"<<endl;
		return true;
	}

//Override
	inline void operator()(cv::Mat& frame) {
		if(frame.empty()) return;
		static helper::PerformanceMeasurer PM(1000);
		PM.tic();

		Mat gray;
		cvtColor(frame, gray, CV_RGB2GRAY);
		buildPyramid(gray, imgpyr, ldetector.nOctaves-1);
		double rms=-1, zncc=-1;
		double camR[3][3]={{1,0,0},{0,1,0},{0,0,1}},camT[3]={0};

		vector<int> pairs;
		vector<Point2f> dst_corners;
		objKeypoints = detector.getModelPoints();
		ldetector(imgpyr, imgKeypoints, 300);
		bool found = detector(imgpyr, imgKeypoints, H, dst_corners, &pairs);

		if(found && validateH(dst_corners) ) {
			evaluateH(gray, object, rms, zncc);
			static double crns[4][2] = {
				{0,object.rows},
				{object.cols,object.rows},
				{object.cols,0},
				{0,0}
			};
			helper::drawHomography(frame, H, crns);
			draw3D(frame);
			GetCameraPose(camR,camT);
		} else {
			for(int i=0; i<3; ++i) {
				camT[i]=numeric_limits<double>::quiet_NaN();
				for(int j=0; j<3; ++j)
					camR[i][j]=numeric_limits<double>::quiet_NaN();
			}
			rms = zncc = numeric_limits<double>::quiet_NaN();
		}
		double dur = PM.toc();
		cout<<"[evaluateH] RMS="<<rms<<"|ZNCC="<<zncc<<endl;
		cout<<"process duration="<<dur<<endl;

		if(savekeys) {
			CapKeyFrame(framecnt++, frame, camR, camT, rms, zncc, dur);
		}
	}

	inline void handle(char key) {
		if(key!=-1) cout<<"receive key:"<<int(key)<<endl;
	}
};
FERNSprocessor processor;

int main(int argc, char** argv) {
	if(argc<4) {
		cout<<"[usage] "<<argv[0]<<" <url>" //1
			" <K matrix file>" //2
			" <template file>" //3
			" [keyframe saving path]"<<endl;
		cout<<"url example:\n";
		cout<<"photo:///home/simbaforrest/Videos/Webcam/seq_UMshort/*\n";
		cout<<"camera://0\n";
		cout<<"video:///home/simbaforrest/Videos/Webcam/keg_april.ogv"<<endl;
		return -1;
	}

	cv::Ptr<ImageSource> is = helper::createImageSource(argv[1]);
	if(is.empty()) {
		loglne("[main] createImageSource failed!");
		return -1;
	}
	is->reportInfo();

	loglni("[main] loading K matrix from: "<<argv[2]);
	double K[9];
	std::ifstream kfile(argv[2]);
	for(int i=0; i<9; ++i) kfile >> K[i];
	processor.loadK(K);
	loglni("[main] K matrix loaded:");
	loglni(helper::PrintMat<>(3,3,K));

	loglni("[main] loading template image from: "<<argv[3]);
	processor.loadtemplate(argv[3]);

	if(argc>4) processor.savekeys = true;

	is->run(processor, -1);

	if(processor.savekeys) {
		processor.SaveKeyFrames(argv[4]);
	}

	return 0;
}
