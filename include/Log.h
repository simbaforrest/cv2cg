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
a lightweight logger with dynamic log level control
Option before including this header file:
Turn off all log by defining SUPPRESS_LOG
Dynamically control log level by setting:
LogHelper::GLogControl::Instance().level
to following four types:
LogHelper::LOG_QUIET
LogHelper::LOG_ERROR
LogHelper::LOG_INFO
LogHelper::LOG_DEBUG
Provide 3 types of log:
1. <log|tag>[l]<e|i|d>
i.e. loge/logi/.../logle/.../tage/.../tagle/.../tagld
totally similar to std::cout
example:
using LogHelper::el;
loge<<"an error happened!"<<el();
logle<<"another error!";

2. c<log|tag><e|i|d>
i.e. cloge/clogi/.../ctage/.../ctagd
totally similar to printf
example:
cloge("an error happened, par=%d\n",par);

3. flog[l]<e|i|d>
i.e. floge/flogi/.../flogle/.../flogld
mix between c++ style and c style; same log speed
with type 2, faster than type 1.
example:
floge("an error happened!\n");
flogle("another error, par="<<par);
*/

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "singleton.hpp"

namespace LogHelper {

	enum Level {
		LOG_QUIET=0,
		LOG_ERROR=1,
		LOG_INFO=2,
		LOG_DEBUG=3
	};

	struct Control {
		int level;
		Control() : level(LOG_INFO) {}
	};
	typedef helper::Singleton<Control> GLogControl;

#define is_log_quiet  (LogHelper::GLogControl::Instance().level<=LogHelper::LOG_QUIET)
#define is_error_log  (LogHelper::GLogControl::Instance().level>=LogHelper::LOG_ERROR)
#define is_info_log   (LogHelper::GLogControl::Instance().level>=LogHelper::LOG_INFO)
#define is_debug_log  (LogHelper::GLogControl::Instance().level>=LogHelper::LOG_DEBUG)

	struct Endline {};
	inline Endline el() { return Endline(); }

	struct Log {
		Log(bool vb_=true, bool el_=false, std::ostream& os_=std::cout)
			: vb(vb_), el(el_), os(os_) {}

		template<typename Type>
		const Log& operator<<(const Type& t) const {
			if(vb) os<<t; return (*this);
		}

		//manual endline
		inline const Log& operator<<(const Endline& el_) const {
			if(vb) os<<std::endl; return (*this);
		}
		//auto endline
		inline ~Log() { if(vb && el) os<<std::endl; }

		inline void verbose(bool val) { vb=val; }
		inline void endline(bool val) { el=val; }

	private:
		bool vb,el;//verbose, endline
		std::ostream& os;
	};//Log

	struct NullLog {
		NullLog(bool vb_=true, bool el_=false, std::ostream& os_=std::cout) {}
		template<typename Type>
		const NullLog& operator<<(const Type& t) const { return (*this); }
	};//NullLog

#ifndef SUPPRESS_LOG

#define logif(val)  LogHelper::Log(val)
#define loglif(val) LogHelper::Log(val, true)
#define tagif(val)  logif(val)<<"["<<__FUNCTION__<<"] "
#define taglif(val) loglif(val)<<"["<<__FUNCTION__<<"] "

#define loge        logif(is_error_log)
#define logi        logif(is_info_log)
#define logd        logif(is_debug_log)

#define logle       loglif(is_error_log)
#define logli       loglif(is_info_log)
#define logld       loglif(is_debug_log)

#define tage        loge<<"["<<__FUNCTION__<<" error] "
#define tagi        logi<<"["<<__FUNCTION__<<"] "
#define tagd        logd<<"["<<__FUNCTION__<<" debug] "

#define tagle       logle<<"["<<__FUNCTION__<<" error] "
#define tagli       logli<<"["<<__FUNCTION__<<"] "
#define tagld       logld<<"["<<__FUNCTION__<<" debug] "

// faster log then above logs
// e.g.
// flogle("[error] in function xxx!"<<12.3);
#define flogle(A) (is_error_log?(std::cout<<A<<std::endl,0):(0))
#define flogli(A) (is_info_log?(std::cout<<A<<std::endl,0):(0))
#define flogld(A) (is_debug_log?(std::cout<<A<<std::endl,0):(0))

#define floge(A) (is_error_log?(std::cerr<<A,0):(0))
#define flogi(A) (is_info_log?(std::cerr<<A,0):(0))
#define flogd(A) (is_debug_log?(std::cerr<<A,0):(0))

// c style log
#ifndef CLOG_FILE
#define CLOG_FILE stdout
#endif

#define cloge(...) {is_error_log?fprintf(CLOG_FILE,__VA_ARGS__):0;}
#define ctage(...) {is_error_log?\
	(fprintf(CLOG_FILE,"["),\
	fprintf(CLOG_FILE,__FUNCTION__),\
	fprintf(CLOG_FILE,"] "),\
	fprintf(CLOG_FILE,__VA_ARGS__)):0;}
#define clogi(...) {is_info_log?fprintf(CLOG_FILE,__VA_ARGS__):0;}
#define ctagi(...) {is_info_log?\
	(fprintf(CLOG_FILE,"["),\
	fprintf(CLOG_FILE,__FUNCTION__),\
	fprintf(CLOG_FILE,"] "),\
	fprintf(CLOG_FILE,__VA_ARGS__)):0;}
#define clogd(...) {is_debug_log?fprintf(CLOG_FILE,__VA_ARGS__):0;}
#define ctagd(...) {is_debug_log?\
	(fprintf(CLOG_FILE,"["),\
	fprintf(CLOG_FILE,__FUNCTION__),\
	fprintf(CLOG_FILE,"] "),\
	fprintf(CLOG_FILE,__VA_ARGS__)):0;}

#else//SUPPRESS_LOG

#define logif(val)  LogHelper::NullLog()
#define loglif(val) LogHelper::NullLog()
#define tagif(val)  LogHelper::NullLog()
#define taglif(val) LogHelper::NullLog()

#define loge        LogHelper::NullLog()
#define logi        LogHelper::NullLog()
#define logd        LogHelper::NullLog()

#define logle       LogHelper::NullLog()
#define logli       LogHelper::NullLog()
#define logld       LogHelper::NullLog()

#define tage        LogHelper::NullLog()
#define tagi        LogHelper::NullLog()
#define tagd        LogHelper::NullLog()

#define tagle       LogHelper::NullLog()
#define tagli       LogHelper::NullLog()
#define tagld       LogHelper::NullLog()

	//faster log than above logs
#define flogle(A)
#define flogli(A)
#define flogld(A)

#define floge(A)
#define flogi(A)
#define flogd(A)

	// c style log
#ifndef CLOG_FILE
#define CLOG_FILE stdout
#endif

#define cloge(...)
#define ctage(...)
#define clogi(...)
#define ctagi(...)
#define clogd(...)
#define ctagd(...)

#endif//SUPPRESS_LOG

}//end of LogHelper

///////////////////////////////////////////////////
#ifdef LOG_UNIT_TEST

#include <time.h>
#define NTIMES 10000

const char LogMode[4][10] = {
	{"LOG_QUIET"},
	{"LOG_ERROR"},
	{"LOG_INFO"},
	{"LOG_DEBUG"}
};

namespace test {
	struct Test{
		void operator()() {
			using LogHelper::el;
			loge << std::setw(10) << std::setfill('.') << "loge" << el();
			logi << "logi" << 12.0 << el();
			logd << "logd" << 10 << el();

			logle << std::setw(10) << std::setfill('.') << "logle";
			logli << "logli" << 12.0;
			logld << "logld" << 10;

			tage << std::setw(10) << std::setfill('.') << "tage" << el();
			tagi << "tagi" << 12.0 << el();
			tagd << "tagd" << 10 << el();

			tagle << std::setw(10) << std::setfill('.') << "tagle";
			tagli << "tagli" << 12.0;
			tagld << "tagld" << 10;

			flogle("flogle"<<12);
			flogli("flogli"<<13);
			flogld("flogld"<<14);

			floge("floge\\n\n");
			flogi("flogi\\n\n");
			flogd("flogd\\n\n");

			//c style
			cloge("test cloge: %d\n", 10);
			ctage("test ctage: %f\n", 22.2);
			clogi("test clogi: %d\n", 10);
			ctagi("test ctagi: %f\n", 22.2);
			clogd("test clogd: %d\n", 10);
			ctagd("test ctagd: %f\n", 22.2);
		}
	};
}

int main(int argc, const char **argv)
{
	using namespace LogHelper;
	int t;
	test::Test tt;
	GLogControl::Instance().level = LOG_ERROR;
	for(int cnt=0; cnt<NTIMES; ++cnt) {
		const int sw = cnt % 4;
		switch(sw) {
		case 0: GLogControl::Instance().level = LOG_QUIET; break;
		case 1: GLogControl::Instance().level = LOG_ERROR; break;
		case 2: GLogControl::Instance().level = LOG_INFO; break;
		case 3: GLogControl::Instance().level = LOG_DEBUG; break;
		};
		//std::cout<<"-----------------------------"<<sw<<std::endl;
		tt();
	}
	t = clock();

	std::cout
#ifdef SUPPRESS_LOG
		<<"Suppressed Log "
#else
		<<"Log "
#endif
		<<NTIMES
		<<" times took me "<<t
		<<" clicks ("<<((float)t)/CLOCKS_PER_SEC
		<<" seconds)."<<std::endl;
	return 0;
}
#endif//LOG_UNIT_TEST