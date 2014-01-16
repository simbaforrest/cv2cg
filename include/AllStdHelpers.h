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

/* AllStdHelpers.h
   a set of helper functions for general C/C++ */

//helper include
#include "ClusterHelper.h"
#include "DirHelper.h"
#include "FilterHelper.h"
#include "IOHelper.h"
#include "LogHelper.h"
#include "SearchHelper.h"
#include "StringHelper.h"
#include "singleton.hpp"
#include "config.hpp"

//adding to helper namespace
namespace helper {
using namespace ClusterHelper;
using namespace DirHelper;
using namespace FilterHelper;
using namespace IOHelper;
using namespace LogHelper;
using namespace SearchHelper;
using namespace StringHelper;
using namespace ConfigHelper;
}//end of helper
