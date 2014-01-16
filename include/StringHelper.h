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

/* StringHelper.h
   helper functions for std::string */

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace StringHelper {

inline void trim(std::string& s)
{
	size_t spos = s.find_first_not_of(" \n\r\t");
	if (spos == std::string::npos) {
		s = std::string("");
		return;
	}
	s.erase(0, spos);
	size_t epos = s.find_last_not_of(" \n\r\t");
	s.erase(epos + 1, epos != std::string::npos ? s.size() - epos - 1 : 0);
}

//string split function from http://stackoverflow.com/a/236803/2303236
inline std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems)
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

inline std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

template<typename TYPE>
void num2str(TYPE num, std::string &str, const int width=0, const char fill='0')
{
	std::stringstream ss;
	if(width>0)
		ss << std::setw(width)
			<< std::setfill(fill);
	ss << num;
	ss >> str;
}

template<typename TYPE>
std::string num2str(TYPE num, const int width=0, const char fill='0')
{
	std::ostringstream oss;
	if(width>0)
		oss << std::setw(width)
			<< std::setfill(fill);
	oss << num;
	return oss.str();
}

}//end of StringHelper
