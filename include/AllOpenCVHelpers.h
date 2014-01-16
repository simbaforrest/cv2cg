#pragma once
/*
 *  Copyright (c) 2010  Chen Feng (cforrest (at) umich.edu)
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

/* AllOpenCVHelpers.h
   a set of helper functions for easier access to opencv */

#include "OpenCVHeaders.h"

#include "cv2cgConfig.h"

#include "CvMatHelper.h"
#include "CameraHelper.h"
#include "UtilHelper.h"
#include "VisHelper.h"
#include "ImageHelper.h"
#include "RotationHelper.h"
#include "PerformanceHelper.h"
#include "DetectorHelper.h"

namespace helper
{
using namespace CvMatHelper;
using namespace CameraHelper;
using namespace VisHelper;
using namespace UtilHelper;
using namespace ImageHelper;
using namespace RotationHelper;
using namespace PerformanceHelper;
using namespace DetectorHelper;
}
