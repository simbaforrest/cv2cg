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
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <map>

#include "singleton.hpp"

namespace ConfigHelper {
	struct Config {
		typedef std::map<std::string, std::string> ConfigMap;
		typedef ConfigMap::value_type ConfigPair;

		// for accessing all compulsory config pairs
		template<typename T>
		T get(const std::string& key) const {
			ConfigMap::const_iterator itr = cm.find(key);
			if(itr==cm.end()) {
				throw std::runtime_error("[Config error] no such key: "+key);
			}
			std::stringstream ss;
			ss << itr->second;
			T ret;
			ss >> ret;
			return ret;
		}

		// return the raw string in key=RawString line in the cfg file
		std::string getRawString(const std::string& key, std::string defaultValue="") const {
			ConfigMap::const_iterator itr = cm.find(key);
			if(itr==cm.end()) return defaultValue;
			std::string ret = itr->second;
			trim(ret);
			return ret;
		}

		// for both compulsory and optional config pairs
		template<typename T>
		bool get(const std::string& key, T& ret, bool isOptional=true) const {
			ConfigMap::const_iterator itr = cm.find(key);
			if(itr==cm.end()) {
				if(isOptional) return false;
				throw std::runtime_error("[Config error] no such key: "+key);
			}
			std::stringstream ss;
			ss << itr->second;
			ss >> ret;
			return true;
		}

		//optional get
		template<typename T>
		T get(const std::string& key, T defaultValue) const {
			T ret;
			if(get<T>(key, ret, true)) return ret;
			return defaultValue;
		}

		inline static void trim(std::string& s) {
			size_t spos = s.find_first_not_of(" \n\r\t");
			s.erase(0, spos!=std::string::npos?spos:0);
			size_t epos = s.find_last_not_of(" \n\r\t");
			s.erase(epos+1, epos!=std::string::npos?s.size()-epos-1:0);
		}

		//parse string in form "key=val" and insert to ConfigMap
		//return true if successuflly parsed and false otherwise
		inline bool parseLine(const std::string& line, std::string& key, std::string& val)
		{
			size_t split_pos = line.find_first_of('=');
			if(split_pos==std::string::npos
				|| split_pos>=line.size()-1
				|| split_pos<=0)
			{
				return false;
			}
			key=std::string(line,0,split_pos);
			val=std::string(line,split_pos+1,line.size()-split_pos-1);
			trim(key); trim(val);
			cm[key]=val;
			return true;
		}

		inline bool load(std::string fn) {
			std::ifstream is(fn.c_str());
			if(!is.is_open()) return false;
			std::cout<<"[Config] load:"<<std::endl;
			while(is) {
				std::string line;
				std::getline(is, line);
				if(line.empty() || line[0]=='#') continue;

				std::string key, val;
				if(!parseLine(line, key, val)) {
					std::cout<<"[Config warn] skip invalid line: "<<line<<std::endl;
				} else {
					std::cout<<key<<"->"<<val<<std::endl;
				}
			}//while
			std::cout<<std::endl;
			return true;
		}

		//can be used to reset/add the ConfigMap from command line
		inline void set(const int argc, const char **argv) {
			if(argc>0) {
				std::cout<<"[Config] add/reset:"<<std::endl;
			}
			for(int i=0; i<argc; ++i) {
				std::string arg(argv[i]), key, val;
				if(!parseLine(arg, key, val)) {
					std::cout<<"[Config warn] skip invalid argument: "<<arg<<std::endl;
				} else {
					std::cout<<key<<"=>"<<val<<std::endl;
				}
			}//end for
			std::cout<<std::endl;
		}

		ConfigMap& map() { return cm; }
		const ConfigMap& map() const { return cm; }

	protected:
		ConfigMap cm;
	};
}//end of ConfigHelper

namespace helper {
	typedef helper::Singleton<ConfigHelper::Config> GConfig;
}//end of helper

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