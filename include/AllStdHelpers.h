#pragma once
/* AllStdHelpers.h
   a set of helper functions for general C/C++ */

//helper include
#ifndef OPENCV_FOUND
#define OPENCV_FOUND //for TimeHelper.hpp, use opencv's gitTickCount function
#endif
#include "lch.hpp"

#include "ClusterHelper.h"
#include "SearchHelper.h"

//adding to helper namespace
namespace helper {
using namespace ClusterHelper;
using namespace SearchHelper;
}//end of helper
