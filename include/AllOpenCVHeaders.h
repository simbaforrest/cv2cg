#pragma once

#include "opencv2/opencv.hpp"

/*
#if (CV_MAJOR_VERSION*100+CV_MINOR_VERSION*10+CV_SUBMINOR_VERSION)>=233
//#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/legacy/legacy.hpp"
#endif
*/

#pragma once

#include <opencv2/core/version.hpp>

///////////////////////////////////////////////////////////////////////
// defines for headers

#if		CV_MAJOR_VERSION == 2
#define	UTIL_OPENCV_HEADER(module)	<opencv2/module/module.hpp>
#elif	CV_MAJOR_VERSION == 3
#define	UTIL_OPENCV_HEADER(module)	<opencv2/module.hpp>
#endif


// include those headers by default
//#include UTIL_OPENCV_HEADER(core)
//#include UTIL_OPENCV_HEADER(imgproc)
//#include UTIL_OPENCV_HEADER(highgui)

// additional headers can be specified as follows
//#include UTIL_OPENCV_HEADER(features2d)
//#include UTIL_OPENCV_HEADER(nonfree)
//#include UTIL_OPENCV_HEADER(calib3d)


///////////////////////////////////////////////////////////////////////
// defines for libraries
//
#define UTIL_OPENCV_LIB_VERSION	CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)
#define UTIL_OPENCV_LIB_PREFIX "opencv_"
#ifndef	_DEBUG
#define UTIL_OPENCV_LIB_APPENDIX ".lib"
#else
#define UTIL_OPENCV_LIB_APPENDIX "d.lib"
#endif
#define UTIL_OPENCV_LIB(module) UTIL_OPENCV_LIB_PREFIX #module UTIL_OPENCV_LIB_VERSION UTIL_OPENCV_LIB_APPENDIX

// libraries can be specified as follows
//#pragma comment(lib, UTIL_OPENCV_LIB(calib3d))
//#pragma comment(lib, UTIL_OPENCV_LIB(contrib))
//#pragma comment(lib, UTIL_OPENCV_LIB(core))
//#pragma comment(lib, UTIL_OPENCV_LIB(features2d))
//#pragma comment(lib, UTIL_OPENCV_LIB(flann))
//#pragma comment(lib, UTIL_OPENCV_LIB(gpu))
//#pragma comment(lib, UTIL_OPENCV_LIB(highgui))
//#pragma comment(lib, UTIL_OPENCV_LIB(imgproc))
//#pragma comment(lib, UTIL_OPENCV_LIB(nonfree))
//#pragma comment(lib, UTIL_OPENCV_LIB(legacy))
//#pragma comment(lib, UTIL_OPENCV_LIB(ml))
//#pragma comment(lib, UTIL_OPENCV_LIB(objdetect))
//#pragma comment(lib, UTIL_OPENCV_LIB(ts))
//#pragma comment(lib, UTIL_OPENCV_LIB(video))

