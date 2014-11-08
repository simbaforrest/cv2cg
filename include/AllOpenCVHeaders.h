#pragma once

#include "opencv2/opencv.hpp"

#if (CV_MAJOR_VERSION*100+CV_MINOR_VERSION*10+CV_SUBMINOR_VERSION)>=233
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/legacy/legacy.hpp"
#endif
