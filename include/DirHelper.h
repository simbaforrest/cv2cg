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

/* DirHelper.h
   helpers related to directory, path, filename*/

#include <string>
#include <vector>
#include <stdlib.h> //getenv

#include "StringHelper.h"

namespace DirHelper
{
#ifdef __linux
#include <unistd.h>
inline std::string getCurrentDir()
{
	char cwd[FILENAME_MAX];
	if(getcwd(cwd,sizeof(cwd))==NULL) return std::string("./");
	return std::string(cwd);
}

#elif _WIN32
#include <direct.h>
inline std::string getCurrentDir()
{
	char* pcwd;
	if((pcwd=getcwd(NULL,0))==NULL) return std::string(".\\");
	std::string ret(pcwd);
	free(pcwd);
	return ret;
}

#else
inline std::string getCurrentDir()
{
	return std::string("./");
}
#endif

/**
 * return the $PATH environment variable as a list of directory strings
 */
inline std::vector<std::string> getEnvPath()
{
	std::string path(getenv("PATH"));
#ifdef unix
	char delim=':';
#else
	char delim=';';
#endif
	return StringHelper::split(path,delim);
}

//dir, "D:/test/test.txt" -> "D:/test/"
inline std::string getFileDir(const std::string &fileName)
{
	std::string::size_type slash1 = fileName.find_last_of('/');
	std::string::size_type slash2 = fileName.find_last_of('\\');
	if (slash1==std::string::npos) {
		if (slash2==std::string::npos) {
			return std::string("./");
		}
		return std::string(fileName,0,slash2+1);
	}
	if (slash2==std::string::npos) {
		return std::string(fileName,0,slash1+1);
	}
	return std::string(fileName, 0, 1 + slash1>slash2 ?  slash1 : slash2);
}

// no ext, pure name
inline std::string getNameNoExtension(const std::string &str)
{
	std::string::size_type begin = str.find_last_of('\\');
	std::string::size_type last = str.find_last_of('.');
	if(last == str.npos) {
		last = str.length()-1;
	}
	if ((begin == str.npos) || (begin > last)) {
		return str;
	} else {
		return str.substr(begin+1, last-begin-1);
	}
}

inline std::string getNameWithExtension(const std::string &fileName)
{
	std::string::size_type slash1 = fileName.find_last_of('/');
	std::string::size_type slash2 = fileName.find_last_of('\\');
	if (slash1==std::string::npos) {
		if (slash2==std::string::npos) {
			return fileName;
		}
		return std::string(fileName.begin()+slash2+1,fileName.end());
	}
	if (slash2==std::string::npos) {
		return std::string(fileName.begin()+slash1+1,fileName.end());
	}
	return std::string(fileName.begin()+(slash1>slash2?slash1:slash2)+1,fileName.end());
}

inline std::string getFileExtensionNoDot(const std::string &fileName)
{
	std::string::size_type dot = fileName.find_last_of('.');
	if (dot==std::string::npos) {
		return std::string("");
	}
	return std::string(fileName.begin()+dot+1,fileName.end());
}

inline std::string getFileExtensionWithDot(const std::string &fileName)
{
	std::string::size_type dot = fileName.find_last_of('.');
	if (dot==std::string::npos) {
		return std::string("");
	}
	return std::string(fileName.begin()+dot,fileName.end());
}

//make sure dir ends with '/' or '\\', if not, modify it. e.g.:
//string dir = "./data";
//legalDir(dir) returns "./data/"
inline std::string &legalDir(std::string &dir)
{
#ifdef _WIN32
	char sep = '\\';
#else
	char sep = '/';
#endif
	if(*dir.rbegin()!=sep) {
		dir.push_back(sep);    //ensure last char==sep
	}
	return dir;
}

}//DirHelper
