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

/* config.hpp */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>
#include <map>

#include "singleton.hpp"
#include "StringHelper.h"
#include "IOHelper.h"

namespace ConfigHelper {
/**
 Config can be loaded from config file and also reset by command line arguments
 config file format:
 key=val
 key=[array]

 for example:
 varInt=1
 varDouble=12.23
 varString=hello
 varArray=[1, 2, 3]
 varArray2=[1 2;
 3,4
 5 6]

 note that comment line must be a separate line starting with character '#'
 */
struct Config {
	typedef std::map<std::string, std::string> ConfigMap;
	typedef ConfigMap::value_type ConfigPair;

	// for accessing all compulsory config pairs
	// will throw runtime_error if key not found
	template<typename T>
	T get(const std::string& key) const {
		ConfigMap::const_iterator itr = cm.find(key);
		if (itr == cm.end()) {
			throw std::runtime_error("[Config error] no such key: " + key);
		}
		std::stringstream ss;
		ss << itr->second;
		T ret;
		ss >> ret;
		return ret;
	}

	//optional get
	template<typename T>
	T get(const std::string& key, T defaultValue) const {
		T ret;
		if (get<T>(key, ret, true))
			return ret;
		return defaultValue;
	}

	// return the raw string in key=RawString line in the cfg file
	std::string getValAsString(const std::string& key,
			std::string defaultValue = "") const {
		ConfigMap::const_iterator itr = cm.find(key);
		if (itr == cm.end())
			return defaultValue;
		std::string ret = itr->second;
		StringHelper::trim(ret);
		return ret;
	}

	/**
	 get matrix from config key value pair with form as follow:
	 key=[num0, num1, num2,...,numN]
	 or
	 key=[num0,num1;\n num2,num3;\n ...]

	 return number of numbers read
	 */
	template<typename ArrayType>
	int get(const std::string& key, const int N, ArrayType& dst) const {
		std::string line = getValAsString(key);
		if (line.empty())
			return 0;
		const char* start = line.c_str();
		int i = 0;
		for (int cnt = 0; i < N && cnt < (int)line.length(); ++i) {
			int skip = 0;
			double val;
			sscanf(start + cnt, "%*[^0-9-+.eE]%lf%n", &val, &skip);
			if (skip <= 0) {
				throw std::runtime_error(
						"[Config::get error] not enough data!");
			}
			dst[i] = val; //auto hidden convert to dst[i]'s type
			cnt += skip;
		}
		return i;
	}

	/**
	 load from config file whose file path is fn
	 return true if successfully loaded and false otherwise
	 note: runtime_error will be thrown if parsing failed!
	 */
	inline bool load(std::string fn) {
		std::ifstream is(fn.c_str());
		if (!is.is_open())
			return false;
		std::cout << "[Config] load: " << fn << std::endl;
		std::string key, val;
		ParseState ps = PS_BEGIN;
		std::string line;
		while (IOHelper::readValidLine(is,line,'#')) {
			if (ps != PS_ARRAY)
				ps = PS_BEGIN;
			if (PS_FAIL == (ps = parseLine(line, ps, key, val))) {
				std::cout << "[Config warn] skip invalid line: " << line
						<< std::endl;
			} else if (ps == PS_DONE) {
				std::cout << key << "->" << val << std::endl;
			}
		} //while
		if (ps != PS_DONE) {
			throw std::runtime_error(
					"[Config::load error] parsing config file failed!");
		}
		std::cout << std::endl;
		return true;
	}

	//can be used to reset/add the ConfigMap from command line
	inline void set(const int argc, const char **argv) {
		if (argc > 0) {
			std::cout << "[Config] add/reset:" << std::endl;
		}
		for (int i = 0; i < argc; ++i) {
			std::string arg(argv[i]), key, val;
			if (!parseLine(arg, key, val)) {
				std::cout << "[Config warn] skip invalid argument: " << arg
						<< std::endl;
			} else {
				std::cout << key << "=>" << val << std::endl;
			}
		} //end for
		std::cout << std::endl;
	}

	ConfigMap& map() {
		return cm;
	}
	const ConfigMap& map() const {
		return cm;
	}

protected:
	// for both compulsory and optional config pairs
	// will throw runtime_error if key not found and !isOptional
	template<typename T>
	bool get(const std::string& key, T& ret, bool isOptional = true) const {
		ConfigMap::const_iterator itr = cm.find(key);
		if (itr == cm.end()) {
			if (isOptional)
				return false;
			throw std::runtime_error("[Config error] no such key: " + key);
		}
		std::stringstream ss;
		ss << itr->second;
		ss >> ret;
		return true;
	}

	//parse string in form "key=val" and insert to ConfigMap
	//return true if successfully parsed and false otherwise
	inline bool parseLine(const std::string& line, std::string& key,
			std::string& val) {
		size_t split_pos = line.find_first_of('=');
		if (split_pos == std::string::npos || split_pos >= line.size() - 1
				|| split_pos <= 0) {
			return false;
		}
		key = std::string(line, 0, split_pos);
		val = std::string(line, split_pos + 1, line.size() - split_pos - 1);
		StringHelper::trim(key);
		StringHelper::trim(val);
		cm[key] = val;
		return true;
	}

	enum ArrayState {
		AS_BEGIN = 0, //start with [
		AS_END, //end with ]
		AS_SUPPRESS_END, //end with ];
		AS_INSIDE
	};
	inline static ArrayState arrayState(const std::string& str) {
		const char lastCh = str[str.length() - 1];
		if (lastCh == ']')
			return AS_END;
		if (lastCh == ';') {
			std::string::size_type pos = str.find_last_not_of("; \n\r\t");
			if (pos != std::string::npos && str[pos] == ']') { //suppress output
				return AS_SUPPRESS_END;
			}
		}
		if (str[0] == '[')
			return AS_BEGIN;
		return AS_INSIDE;
	}

	enum ParseState {
		PS_BEGIN = 0, PS_DONE, PS_ARRAY, PS_FAIL
	};
	//parse more complex config variables such as array variable
	inline ParseState parseLine(const std::string& line, const ParseState state,
			std::string& key, std::string& val) {
		if (state == PS_BEGIN) {
			if (!parseLine(line, key, val))
				return PS_FAIL;
			ArrayState as = arrayState(val);
			switch (as) {
			case AS_BEGIN:
				return PS_ARRAY;
			case AS_SUPPRESS_END:
				val = cm[key];
				if ((int) val.length() > 10) {
					val = val.substr(0, 10) + "...]";
				}
				//means not array var
				return PS_DONE;
			case AS_INSIDE:
				//means array but finished within in one line
				return PS_DONE;
			case AS_END:
				return PS_DONE;
			}
			return PS_DONE; //this would never be reached
		} else if (state == PS_ARRAY) {
			std::string nline(line);
			StringHelper::trim(nline);
			if (nline.empty()) {
				std::cout
						<< "[Config::parseLine warn] skip empty line in array!"
						<< std::endl;
				return PS_ARRAY;
			}
			std::string oline = cm[key];
			if (oline[oline.length() - 1] == ';'
					|| oline[oline.length() - 1] == '[')
				cm[key].append(nline);
			else
				cm[key].append(std::string(";") + nline);

			ArrayState as = arrayState(nline);
			switch (as) {
			case AS_END:
				val = cm[key];
				return PS_DONE;
			case AS_SUPPRESS_END:
				val = cm[key];
				if ((int) val.length() > 10) {
					val = val.substr(0, 10) + "...]";
				}
				return PS_DONE;
			default:
				return PS_ARRAY;
			}
		}
		return PS_FAIL;
	}
protected:
	ConfigMap cm;
};
} //end of ConfigHelper

namespace helper {
typedef helper::Singleton<ConfigHelper::Config> GConfig;
} //end of helper

#ifdef CONFIG_UNIT_TEST
namespace test {
	template<typename T>
	void findout(std::string key) {
		static const ConfigHelper::Config& cfg=helper::GConfig::Instance();
		std::cout<<key<<"->"<<cfg.get<T>(key)<<std::endl;
	}
}
int main() {
	using helper::GConfig;
	if(!GConfig::Instance().load("sample.cfg")) {
		std::cout<<"[main] fail to load sample.cfg!"<<std::endl;
		exit(-1);
	}
	try {
		test::findout<bool>("bTest");
		test::findout<int>("iTest");
		test::findout<float>("fTest");
		bool flag=true;
		if(GConfig.Instance().get("tTest", flag, true)) {
			std::cout<<"tTest->"<<flag<<std::endl;
		} else {
			std::cout<<"default tTest->"<<flag<<std::endl;
		}
		test::findout<bool>("tTest"); //invalid key will lead to a std::runtime_error
		test::findout<std::string>("sTest");
	} catch (std::exception& e) {
		std::cout<<e.what()<<std::endl;
	}
	return 0;
}
#endif//CONFIG_UNIT_TEST
