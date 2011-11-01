#pragma once
/*
 *  Copyright (c) 2011  Chen Feng (cforrest (at) umich.edu)
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

/* Log.h
	after include this, define the global extern variable for log control:
	Log::Level Log::level = Log::DEBUG; //log everything
*/

#include <stdio.h>

namespace Log
{
enum Level { QUIET, ERROR, INFO, DEBUG };

extern Level level;// = DEBUG;
}

// c++ style log, output to cerr
// e.g.
// loglne("[error] in function xxx!");
#define loglne(A) ((Log::level >= Log::ERROR)?(std::cerr<<A<<std::endl,0):(0))
#define loglni(A) ((Log::level >= Log::INFO)?(std::cerr<<A<<std::endl,0):(0))
#define loglnd(A) ((Log::level >= Log::DEBUG)?(std::cerr<<A<<std::endl,0):(0))

#define loge(A) ((Log::level >= Log::ERROR)?(std::cerr<<A,0):(0))
#define logi(A) ((Log::level >= Log::INFO)?(std::cerr<<A,0):(0))
#define logd(A) ((Log::level >= Log::DEBUG)?(std::cerr<<A,0):(0))

// c style log
#define LOG_FILE stderr

#define LogD(...) (Log::level >= Log::DEBUG ? fprintf(LOG_FILE,__VA_ARGS__) : 0)
#define TagD(...) (Log::level >= Log::DEBUG ? fprintf(LOG_FILE, "[__FUNCTION__ debug]"),fprintf(LOG_FILE,__VA_ARGS__) : 0)

#define LogI(...) (Log::level >= Log::INFO ? fprintf(LOG_FILE,__VA_ARGS__) : 0)
#define TagI(...) (Log::level >= Log::INFO ? fprintf(LOG_FILE, "[__FUNCTION__ info]"),fprintf(LOG_FILE,__VA_ARGS__) : 0)

#define LogE(...) (Log::level >= Log::ERROR ? fprintf(LOG_FILE,__VA_ARGS__) : 0)
#define TagE(...) (Log::level >= Log::ERROR ? fprintf(LOG_FILE, "[__FUNCTION__ error]"),fprintf(LOG_FILE,__VA_ARGS__) : 0)

