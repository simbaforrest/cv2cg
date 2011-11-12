#pragma once
/*
 *  Copyright (c) 2011  Chen Feng (cforrest (at) umich.edu)
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

/* Log.h
	after include this, define the global extern variable for log control:
	Log::Level Log::level = Log::LOG_DEBUG; //log everything
*/

#include <stdio.h>

namespace Log
{
enum Level { 
	LOG_QUIET=0,
	LOG_ERROR,
	LOG_INFO,
	LOG_DEBUG
};

extern Level level;// = DEBUG;
}

// c++ style log, output to cerr
// e.g.
// loglne("[error] in function xxx!");
#define loglne(A) ((Log::level >= Log::LOG_ERROR)?(std::cerr<<A<<std::endl,0):(0))
#define loglni(A) ((Log::level >= Log::LOG_INFO)?(std::cerr<<A<<std::endl,0):(0))
#define loglnd(A) ((Log::level >= Log::LOG_DEBUG)?(std::cerr<<A<<std::endl,0):(0))

#define loge(A) ((Log::level >= Log::LOG_ERROR)?(std::cerr<<A,0):(0))
#define logi(A) ((Log::level >= Log::LOG_INFO)?(std::cerr<<A,0):(0))
#define logd(A) ((Log::level >= Log::LOG_DEBUG)?(std::cerr<<A,0):(0))

// c style log
#define LOG_FILE stderr

#define LogD(...) (Log::level >= Log::LOG_DEBUG ? fprintf(LOG_FILE,__VA_ARGS__) : 0)
#define TagD(...) (Log::level >= Log::LOG_DEBUG ? fprintf(LOG_FILE, "[__FUNCTION__ debug]"),fprintf(LOG_FILE,__VA_ARGS__) : 0)

#define LogI(...) (Log::level >= Log::LOG_INFO ? fprintf(LOG_FILE,__VA_ARGS__) : 0)
#define TagI(...) (Log::level >= Log::LOG_INFO ? fprintf(LOG_FILE, "[__FUNCTION__ info]"),fprintf(LOG_FILE,__VA_ARGS__) : 0)

#define LogE(...) (Log::level >= Log::LOG_ERROR ? fprintf(LOG_FILE,__VA_ARGS__) : 0)
#define TagE(...) (Log::level >= Log::LOG_ERROR ? fprintf(LOG_FILE, "[__FUNCTION__ error]"),fprintf(LOG_FILE,__VA_ARGS__) : 0)

