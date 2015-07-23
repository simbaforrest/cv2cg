#pragma once
/* DirHelper
 * helpers related to directory, path, filename */

#include <string>
#include <vector>
#include <stdlib.h> //getenv

#include "StringHelper.hpp"

namespace DirHelper
{
#ifdef _WIN32
const char filesep = '\\';
#else
const char filesep = '/';
#endif

// similar to matlab's fullfile
// example:
// std::string combined = DirHelper::FullFile()<<folderA<<folderB<<fileC;
class FullFile {
	std::string out;
public:
	FullFile(std::string init="") : out(init) {}
	FullFile(FullFile& other) : out(other.out) {}

	inline operator std::string() { return out; }

	inline FullFile& operator<<(std::string other) {
		if (other.empty())
			return *this;
		if (out.empty() || out[out.length()-1]==filesep || other[0]==filesep)
			out += other;
		else
			out += filesep + other;
		return *this;
	}
};

// similar to matlab's fileparts
// if in=parent/child/file.txt
// then path=parent/child
// name=file, ext=txt
inline void fileparts(const std::string& str, std::string* pPath=0,
	std::string* pName=0, std::string* pExt=0)
{
	std::string::size_type last_sep = str.find_last_of(filesep);
	std::string::size_type last_dot = str.find_last_of('.');
	if (last_sep!=std::string::npos && last_dot<last_sep) // "D:\parent\child.folderA\file", "D:\parent\child.folderA\"
		last_dot = std::string::npos;

	std::string path, name, ext;

	if (last_sep==std::string::npos) {
		path = "";
		if(last_dot==std::string::npos) { // "test"
			name = str;
			ext = "";
		} else { // "test.txt"
			name = str.substr(0, last_dot);
			ext = str.substr(last_dot+1);
		}
	} else {
		path = str.substr(0, last_sep);
		if(last_dot==std::string::npos) { // "d:/parent/test", "d:/parent/child/"
			name = str.substr(last_sep+1);
			ext = "";
		} else { // "d:/parent/test.txt"
			name = str.substr(last_sep+1, last_dot-last_sep-1);
			ext = str.substr(last_dot+1);
		}
	}
	
	if(pPath!=0) {
		*pPath = path;
	}
	if(pName!=0) {
		*pName = name;
	}
	if(pExt!=0) {
		*pExt = ext;
	}
}

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

//"D:/test/test.txt" -> "D:/test/"
inline std::string getFileDir(const std::string &fileName)
{
	std::string path;
	fileparts(fileName, &path);
	return path;
}

//"D:/parent/test.txt" -> "test"
//"D:/parent/test" -> "test"
inline std::string getNameNoExtension(const std::string &fileName)
{
	std::string name;
	fileparts(fileName, 0, &name);
	return name;
}

//"D:/parent/test.txt" -> "test.txt"
inline std::string getNameWithExtension(const std::string &fileName)
{
	std::string name, ext;
	fileparts(fileName, 0, &name, &ext);
	return name+"."+ext;
}

//"D:/parent/test.txt" -> "txt"
inline std::string getFileExtensionNoDot(const std::string &fileName)
{
	std::string ext;
	fileparts(fileName, 0, 0, &ext);
	return ext;
}

//"D:/parent/test.txt" -> ".txt"
inline std::string getFileExtensionWithDot(const std::string &fileName)
{
	std::string ext;
	fileparts(fileName, 0, 0, &ext);
	ext = "." + ext;
	return ext;
}

//make sure dir ends with '/' or '\\', if not, modify it. e.g.:
//string dir = "./data";
//legalDir(dir) returns "./data/"
inline std::string &legalDir(std::string &dir)
{
	if(*dir.rbegin()!=filesep) {
		dir.push_back(filesep);    //ensure last char==sep
	}
	return dir;
}

}//DirHelper
