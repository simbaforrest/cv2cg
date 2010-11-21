#ifndef __MATRIX_MANIP_LOG_HEADER__
#define __MATRIX_MANIP_LOG_HEADER__
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

/* MatrixManipLog.h */

#include <stdio.h>

namespace MatrixManip {
	namespace Log {
		//switch them
		static bool debug=true, info=true, error=true, warn=true;
		static FILE *logfile = stdout;

		/// With Tag at the beginning
		// debug
		inline static void d(const char* msg, const char* tag) {
			if(debug && logfile) 
				fprintf(logfile, "[%s debug] %s\n", tag, msg);
		}

		// info
		inline static void i(const char* msg, const char* tag) {
			if(info && logfile)
				fprintf(logfile, "[%s info] %s\n", tag, msg);
		}

		// error
		inline static void e(const char* msg, const char* tag) {
			if(error && logfile)
				fprintf(logfile, "[%s error] %s\n", tag, msg);
		}

		// warn
		inline static void w(const char* msg, const char* tag) {
			if(warn && logfile)
				fprintf(logfile, "[%s warn] %s\n", tag, msg);
		}

		/// Without Tag
		// debug
		inline static void d(const char* msg) {
			if(debug && logfile) 
				fprintf(logfile, "\t%s\n", msg);
		}

		// info
		inline static void i(const char* msg) {
			if(info && logfile)
				fprintf(logfile, "\t%s\n", msg);
		}

		// error
		inline static void e(const char* msg) {
			if(error && logfile)
				fprintf(logfile, "\t%s\n", msg);
		}

		// warn
		inline static void w(const char* msg) {
			if(warn && logfile)
				fprintf(logfile, "\t%s\n", msg);
		}
	};//Log
}//namespace

#endif//__MATRIX_MANIP_LOG_HEADER__