#pragma once
/* 
 *  Copyright (c) 2010  Chen Feng (cforrest (at) umich.edu)
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

/* Log.h */

#include <stdio.h>

namespace Log {
	//switch them
	extern bool debug, info, error, warn;
	extern FILE *logfile;

	inline static bool tag(const char* tag, const char* level, bool control) {
		if(control && logfile)
			fprintf(logfile, "[%s %s] ", tag, level);
		return true;
	}
}//Log

#define LogD(...) fprintf(Log::debug?Log::logfile:0,__VA_ARGS__)
#define LogI(...) fprintf(Log::info?Log::logfile:0,__VA_ARGS__)
#define LogE(...) fprintf(Log::error?Log::logfile:0,__VA_ARGS__)
#define LogW(...) fprintf(Log::warn?Log::logfile:0,__VA_ARGS__)
#define TagD(...) Log::tag(__FUNCTION__, "debug", Log::debug),\
                    fprintf(Log::debug?Log::logfile:0,__VA_ARGS__)
#define TagI(...) Log::tag(__FUNCTION__, "info", Log::info),\
                    fprintf(Log::info?Log::logfile:0,__VA_ARGS__)
#define TagE(...) Log::tag(__FUNCTION__, "error", Log::error),\
                    fprintf(Log::error?Log::logfile:0,__VA_ARGS__)
#define TagW(...) Log::tag(__FUNCTION__, "warn", Log::warn),\
                    fprintf(Log::warn?Log::logfile:0,__VA_ARGS__)