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
#include <vector>
#include <queue>
#include <cassert>

#include "singleton.hpp"
#include "StringHelper.h"
#include "IOHelper.h"
#include "DirHelper.h"

namespace ConfigHelper {
/**
Config can be loaded from config file and also reset by command line arguments
Format:
A config file is internally a tree of ConfigNode. There are 3 types of such
nodes:
1. Simple:
	key=val
2. Array (nodes are separated by any of ",; \n\r\t"):
	key=[node1,node2;node3 node4
	 node5	node6]
3. Map (similarly, nodes are separated by any of ",; \n\r\t"):
	key={subkey1=node1, subkey2=node2;}

For example:
varInt=1
varDouble=12.23
varString=hello
varArray=[1, 2, 3]
varArray2=[1 2;
3,4
5 6]
varMap={key1=val1, key2=val2 key3=val3;
key4=val4, key5=[v1,v2,v3;v4,v5 v6]}
varArr=[
v1, v2 v3, v4;
v5, v6, v7, v8
v9, v10, v11 v12;
{key1=val1, key2=val2}
]

Note:
1. that comment line must be a separate line whose first non-white char is '#'
2. Array or map immediately followed by ";" means that when printing this node,
it will be printed in a short format like [abc......] or {123......}
3. characters "=[]{},; \n\r\t" are reserved for parsing control; avoid using
these characters in your config file, otherwise parsing may fail; also
characters ":@" are reserved for key_path; avoid using these as key's name
 */
class ConfigNode {
public:
	typedef ConfigNode* Ptr;
	typedef ConfigNode const* ConstPtr;
	typedef std::string ConfigNodeSimple;
	typedef std::vector<Ptr> ConfigNodeArray;
	typedef std::map<std::string, Ptr> ConfigNodeMap;

	enum TYPE {
		SIMPLE=0, ARRAY, MAP, UNKNOWN
	};

private:
	union DATA {
		ConfigNodeSimple* simple;
		ConfigNodeArray* array;
		ConfigNodeMap* map;
	} data;

	TYPE type;

	bool printAll;

public:
	/**
	 * construct a new node from a single line
	 */
	static inline ConfigNode::Ptr create(const std::string& line, const TYPE node_type=UNKNOWN) {
		ConfigNode::Ptr ret = new ConfigNode(line, node_type);
		return ret;
	}

	/**
	 * construct a new SIMPLE/ARRAY/MAP node from lines
	 */
	static inline ConfigNode::Ptr create(std::queue<std::string>& lines, const TYPE node_type=UNKNOWN) {
		ConfigNode::Ptr ret = new ConfigNode(lines, node_type);
		return ret;
	}

	~ConfigNode() { clear(); }

	/**
	 * return type of the node (SIMPLE, ARRAY or MAP)
	 */
	inline TYPE nodeType() const { return type; }
	inline bool isSimple() const { return type==SIMPLE; }
	inline bool isArray() const { return type==ARRAY; }
	inline bool isMap() const { return type==MAP; }

	/**
	 * return size of the node content
	 * for SIMPLE type node, this always return 1
	 */
	inline int size() const {
		switch(type) {
		case SIMPLE: return 1;
		case ARRAY: return data.array->size();
		case MAP: return data.map->size();
		default: throw std::invalid_argument(
				"[ConfigNode::size error] not recognized type="
						+ StringHelper::num2str(type));
		}
	}

	/**
	 * retrieve required ConfigNode, i.e. the node must be in the cfg file
	 * otherwise will throw exception
	 */
	inline ConfigNode& operator[] (const int pos) {
		if(type!=ARRAY) throw std::invalid_argument("[ConfigNode::operator[] error] current node type is not ARRAY!");
		return *data.array->at(pos);
	}
	inline const ConfigNode& operator[] (const int pos) const { return (*this)[pos]; }

	inline ConfigNode& operator[] (const std::string& key) {
		if(type!=MAP) throw std::invalid_argument("[ConfigNode::operator[] error] current node type is not MAP!");
		ConfigNodeMap::iterator itr=data.map->find(key);
		if (itr==data.map->end()) throw std::out_of_range("[ConfigNode::operator[] error] out of range error!");
		return *(itr->second);
	}
	inline const ConfigNode& operator[] (const std::string& key) const { return (*this)[key]; }

	/**
	 * get a ConfigNode Ptr from key-path
	 * key-path is the sequence of key's used to get from current node
	 * to destination node, separated by ":"; array items are referred
	 * by "@pos", e.g. "@1" means the 2nd item of the array
	 *
	 * @return 0 only if the key-path is invalid for current node
	 */
	inline ConfigNode::Ptr getChild(const std::string& key_path) {
		std::vector<std::string> keys = StringHelper::split(key_path, ':');
		ConfigNode::Ptr node_ptr = this;
		for (int i = 0; i < (int) keys.size(); ++i) {
			if (keys[i][0] == '@') {
				unsigned int pos = StringHelper::str2num<int>(
						keys[i].substr(1));
				if (node_ptr->type != ARRAY
						|| pos >= node_ptr->data.array->size())
					return 0;
				node_ptr = node_ptr->data.array->at(pos);
			} else {
				if (node_ptr->type != MAP
						|| node_ptr->data.map->find(keys[i])
								== node_ptr->data.map->end())
					return 0;
				ConfigNodeMap::iterator itr=node_ptr->data.map->find(keys[i]);
				if(itr==node_ptr->data.map->end()) return 0;
				node_ptr = itr->second;
			}
		}
		return node_ptr;
	}

	/**
	 * safe get even if the key-path doesn't exist
	 */
	inline std::string get(const std::string& key_path,
			std::string default_val) const {
		try {
			ConfigNode::ConstPtr node_ptr =
					const_cast<ConfigNode::Ptr>(this)->getChild(key_path);
			if (node_ptr == 0)
				return default_val;
			return node_ptr->str();
		} catch (...) {
			return default_val;
		}
		return default_val;
	}
	template<class T>
	T get(const std::string& key_path, T default_val) const {
		try {
			ConfigNode::ConstPtr node_ptr =
					const_cast<ConfigNode::Ptr>(this)->getChild(key_path);
			if (node_ptr == 0)
				return default_val;
			return (T) (*node_ptr);
		} catch (...) {
			return default_val;
		}
	}

	/**
	 * check if the key-path exist in current node
	 */
	inline bool exist(const std::string& key_path) const {
		return const_cast<ConfigNode::Ptr>(this)->getChild(key_path) != 0;
	}

	/**
	 * clear the node and then re-init
	 */
	inline void reset(const std::string& line) {
		clear();
		std::queue<std::string> lines;
		lines.push(line);
		init(lines, UNKNOWN);
	}

	/**
	 * merge this node with ConfigNode src:
	 * 1. if the two node are all map type, merge the two maps
	 * 2. otherwise swap this node with src
	 */
	inline void merge(ConfigNode::Ptr src) {
		assert(src!=0);
		if(!src->isMap() || !this->isMap()) {
			swap(src);
			return;
		}
		ConfigNodeMap::iterator itr=src->data.map->begin(), it_end=src->data.map->end();
		for(; itr!=it_end; ++itr) {
			const std::string& srckey = itr->first;
			ConfigNode::Ptr dst_ptr = getChild(srckey);
			if(dst_ptr!=0) dst_ptr->merge(itr->second);
			else {
				(*data.map)[srckey]=itr->second;
				itr->second = 0;
			}
		}
	}

	/**
	 * convert the array node into a vector of T
	 * and return the final size of dst
	 *
	 * e.g.
	 * std::vector<double> mat;
	 * cfg["my_mat"]>>mat;
	 */
	inline int operator>>(std::vector<std::string>& dst) const {
		if (type != ARRAY)
			throw std::invalid_argument(
					"[ConfigNode::operator>> error] current node type is not ARRAY!");
		dst.resize(this->size());
		for (int i = 0; i < (int) data.array->size(); ++i) {
			dst[i] = data.array->at(i)->str();
		}
		return (int) dst.size();
	}
	template<class T>
	int operator>>(std::vector<T>& dst) const {
		if(type!=ARRAY) throw std::invalid_argument("[ConfigNode::operator>> error] current node type is not ARRAY!");
		dst.resize(this->size());
		for (int i = 0; i < (int) data.array->size(); ++i) {
			dst[i] = (T) (*data.array->at(i));
		}
		return (int)dst.size();
	}

	/**
	 * convert the simple node to desired data type
	 * usually T will be raw data type such as int, double
	 *
	 * e.g.
	 * int my_int = cfg["my_int"];
	 */
	inline operator std::string() const { return str();	}
	template<class T>
	operator T() const {
		if(type!=SIMPLE) throw std::invalid_argument("[ConfigNode::operator T() error] current node type is not SIMPLE!");
		std::stringstream ss;
		ss << (*data.simple);
		T ret;
		ss >> ret;
		return ret;
	}

	/**
	 * retrieve the raw string of the node
	 */
	inline std::string str() const {
		std::string ret;
		switch (type) {
		case SIMPLE:
			ret = *data.simple; break;
		case ARRAY:
			ret.push_back('[');
			for(int i=0; i<size(); ++i) {
				ret += data.array->at(i)->str();
				ret.push_back(',');
			}
			ret[ret.length()-1]=']';
			break;
		case MAP: {
			ret.push_back('{');
			ConfigNodeMap::iterator itr = data.map->begin(), it_end =
					data.map->end();
			for (; itr != it_end; ++itr) {
				const std::string& key = itr->first;
				ret += key;
				ret.push_back('=');
				ret += itr->second->str();
				ret.push_back(',');
			}
			ret[ret.length()-1]='}';
			break;
		}
		default:
			break;
		}
		return ret;
	}

	inline std::ostream& print(std::ostream& o, const int printShortSizeTh=10, const int level=0) const {
		switch (type) {
		case SIMPLE: {
			o << (*data.simple);
			break;
		}
		case ARRAY: {
			if (!this->printAll) { //inline format
				std::string content = str();
				if ((int) content.length() > printShortSizeTh) {
					content.resize(printShortSizeTh);
					content += "......]";
				}
				o << content;
			} else {
				if (level > 0)
					o << "[\n";
				for (int i = 0; i < (int) data.array->size(); ++i) {
					for (int k = 0; k < level; ++k)
						o << "\t";
					data.array->at(i)->print(o, printShortSizeTh, level + 1)
							<< "\n";
				}
				if (level > 0) {
					for (int k = 1; k < level; ++k)
						o << "\t";
					o << "]";
				}
			}
			break;
		}
		case MAP: {
			ConfigNodeMap::iterator itr = data.map->begin(), it_end =
					data.map->end();
			if (!this->printAll) {
				std::string content = str();
				if ((int) content.length() > printShortSizeTh) {
					content.resize(printShortSizeTh);
					content += "......}";
				}
				o << content;
			} else {
				if (level > 0)
					o << "{\n";
				for (; itr != it_end; ++itr) {
					for (int i = 0; i < level; ++i)
						o << "\t";
					o << itr->first << "=";
					itr->second->print(o, printShortSizeTh, level + 1) << "\n";
				}
				if (level > 0) {
					for (int k = 1; k < level; ++k)
						o << "\t";
					o << "}";
				}
			}
			break;
		}
		default: {
			throw std::invalid_argument(
					"[ConfigNode::print error] not recognized type="
							+ StringHelper::num2str(type));
		}
		}//switch
		return o;
	}

protected:
	inline void swap(ConfigNode::Ptr src) {
		std::swap(this->data, src->data);
		std::swap(this->printAll, src->printAll);
		std::swap(this->type, src->type);
	}

	/**
	 * clear everything inside the node
	 */
	inline void clear() {
		switch (type) {
		case SIMPLE: {
			delete data.simple;
			data.simple = 0;
			break;
		}
		case ARRAY: {
			for (int i = 0; i < (int) data.array->size(); ++i) {
				delete data.array->at(i);
			}
			data.array->clear();
			delete data.array;
			data.array = 0;
			break;
		}
		case MAP: {
			ConfigNodeMap::iterator itr = data.map->begin(), it_end =
					data.map->end();
			for (; itr != it_end; ++itr) {
				delete itr->second;
			}
			data.map->clear();
			delete data.map;
			data.map = 0;
			break;
		}
		default: {
			throw std::invalid_argument(
					"[ConfigNode::clear error] not recognized type="
							+ StringHelper::num2str(type));
			break;
		}
		}
	}

	inline void init(std::queue<std::string>& lines, const TYPE node_type) {
		type = node_type;
		switch (type) {
		case SIMPLE:
			init_simple (lines);
			break;
		case ARRAY:
			init_array(lines);
			break;
		case MAP:
			init_map(lines);
			break;
		default:
			assert(!lines.empty() && !lines.front().empty());
			switch (lines.front()[0]) {
			case '[':
				init_array(lines);
				break;
			case '{':
				init_map(lines);
				break;
			default:
				init_simple(lines);
				break;
			}
			break;
		}
	}

	/**
	 * Simple ConfigNode
	 * Format:
	 * any/Character/Except/For/Comma/SemiColumn/Newline/WhiteSpaces/Squarebrackets/CurlyBrackets
	 */
	inline void init_simple(std::queue<std::string>& lines) {
		type = SIMPLE;
		while(!lines.empty()) {
			do { //process current line
				StringHelper::ltrim(lines.front(), " ,;\n\r\t");
				if (lines.front().empty())
					break;
				std::string::size_type pos = lines.front().find_first_of(
						" ,;[]{}\n\r\t");
				if (pos == std::string::npos) {
					data.simple = new std::string(lines.front());
					lines.front().clear();
					lines.pop();
				} else {
					data.simple = new std::string(lines.front().substr(0, pos));
					lines.front().erase(0, pos);
				}
				StringHelper::trim(*data.simple);
				return;
			} while (!lines.front().empty());
			lines.pop();
		}//while(!lines.empty())
		throw std::runtime_error("[ConfigNode::init_simple error] failed!");
	}

	/**
	 * Array of ConfigNode
	 * Format:
	 * [items, separated, by, comma, or, semicolumn;
	 *     or, newline, or white space(s)]
	 *
	 * Note: lines.front()[0] must be '['
	 */
	inline void init_array(std::queue<std::string>& lines) {
		lines.front().erase(0,1); //remove leading '['
		data.array=new ConfigNodeArray();
		type = ARRAY;
		while(!lines.empty()) {
			do { //process current line
				StringHelper::ltrim(lines.front(), " ,;\n\r\t");
				if (lines.front().empty())
					break;
				switch (lines.front()[0]) {
				case ']': //close current Array
					lines.front().erase(0, 1);
					if(lines.front().length()>0 && lines.front()[0]==';') this->printAll=false;
					else this->printAll=true;
					return;
				case '[': //enter a new Array
					data.array->push_back(new ConfigNode(lines, ARRAY));
					break;
				case '{': //enter a new Map
					data.array->push_back(new ConfigNode(lines, MAP));
					break;
				default: //find next Array Item, separated by any char of ",; \n\r\t"
					data.array->push_back(new ConfigNode(lines, SIMPLE));
					break;
				}//switch
			} while (!lines.front().empty());
			lines.pop();
		}//while(!lines.empty())
		throw std::runtime_error("[ConfigNode::init_array error] failed to find closing ]!");
	}

	/**
	 * Map of ConfigNode
	 * Format:
	 * {key1=node1,key2=node2 key3=node3
	 *  key4=node4; key5=node5
	 * }
	 *
	 * Note: lines.front()[0] must be '{'
	 */
	void init_map(std::queue<std::string>& lines) {
		lines.front().erase(0, 1); //remove leading '{'
		data.map = new ConfigNodeMap();
		type = MAP;
		std::string key;
		while (!lines.empty()) {
			do {//process current line
				StringHelper::ltrim(lines.front(), " ,;\n\r\t");
				if (lines.front().empty())
					break;
				switch (lines.front()[0]) {
				case '}': //close current Map
					if (!key.empty()) {
						throw std::runtime_error(
								"[ConfigNodeMap::init_map error]"
										" map end before key " + key
										+ " found its value pair!");
					}
					lines.front().erase(0, 1);
					if (lines.front().length() > 0 && lines.front()[0] == ';')
						this->printAll = false;
					else
						this->printAll = true;
					return;
				case '[':
					if (key.empty()) {
						throw std::runtime_error(
								"[ConfigNodeMap::init_map error]"
										"Found [ before found its key!");
					}
					this->insert_new_map_pair(key, lines, ARRAY);
					break;
				case '{':
					if (key.empty()) {
						throw std::runtime_error(
								"[ConfigNodeMap::init_map error]"
										"Found { before found its key!");
					}
					this->insert_new_map_pair(key, lines, MAP);
					break;
				default: //find next Map Item, separated by any char of ",; \n\r\t"
					if (key.empty()) { //find key first
						std::string::size_type mpos = lines.front().find_first_of("=");
						if (mpos == std::string::npos) { //no = sign in this line
							std::string tmp(lines.front());
							lines.pop();
							lines.front().insert(0, tmp);//merge to next line
							break;
						}
						key = lines.front().substr(0, mpos);
						StringHelper::trim(key);
						lines.front().erase(0, mpos + 1);
					} else { //find val pair and insert into map
						this->insert_new_map_pair(key, lines, SIMPLE);
					}//if(key.empty())
					break;
				}//switch
			} while (!lines.front().empty());
			lines.pop();
		}//while(!lines.empty())
		throw std::runtime_error("[ConfigNode::init_map error] failed to find closing }!");
	}

	//internal helper, should only be called inside init_map
	inline void insert_new_map_pair(std::string& key, std::queue<std::string>& lines, const TYPE node_type) {
		(*data.map)[key] = new ConfigNode(lines, node_type);
		key.clear();
	}

	//private ctor: disable ConfigNode to be created on stack
	ConfigNode(const std::string& line, const TYPE node_type=UNKNOWN) : printAll(true) {
		std::queue<std::string> lines;
		lines.push(line);
		init(lines, node_type);
	}
	ConfigNode(std::queue<std::string>& lines, const TYPE node_type=UNKNOWN) : printAll(true) {
		init(lines, node_type);
	}

	//make ConfigNode not copy-able
    ConfigNode();
    ConfigNode(const ConfigNode&);
    const ConfigNode& operator=(const ConfigNode& rhs);
    ConfigNode& operator=(ConfigNode& rhs);
};

class Config {
private:
	ConfigNode* root;

public:
	Config() : root(0) {}
	~Config() { clear(); }

	inline void clear() { if(root!=0) delete root; root=0; }

	inline ConfigNode& getRoot() {
		assert(root!=0);
		return *root;
	}
	inline ConfigNode& operator*() {
		return getRoot();
	}
	inline ConfigNode::Ptr operator->() {
		assert(root!=0);
		return root;
	}

	/**
	 * safe get
	 * if key doesn't exist, return default_val
	 * reference key using key-path
	 * e.g.
	 * 1. get("keyOutside:keyInside",0) means to get the value
	 * from a ConfigNodeMap keyOutside's inner node keyInside
	 * 2. get("keyArray:@1",0) means to get the value from a
	 * ConfigNodeArray keyArray's 2nd inner node
	 */
	inline std::string get(const std::string& key, const char* default_val) const {
		if (root != 0)
			return root->get(key, std::string(default_val));
		return std::string(default_val);
	}
	inline std::string get(const std::string& key, std::string default_val) const {
		if(root!=0)
			return root->get(key, default_val);
		return default_val;
	}
	template<class T>
	T get(const std::string& key, T default_val) const {
		if (root != 0)
			return root->get(key, default_val);
		return default_val;
	}

	void reset(const int argc, const char * const * const argv) {
		if(root==0) root = ConfigNode::create("{}");
		std::cout<<"[Config] reset:"<<std::endl;
		for(int i=0; i<argc; ++i) {
			std::string line(argv[i]);
			if(line[0]!='{')
				line="{"+line+"}";
			std::cout<<i<<":"<<line<<std::endl;
			ConfigNode::Ptr node = ConfigNode::create(line);
			root->merge(node);
			delete node;
		}
		std::cout<<std::endl;
	}

	/**
	 * try to load the config file at fn
	 * return true only if loaded and parsed successfully
	 * return false if file not existed or empty file parsed
	 */
	inline bool load(std::string fn) {
		clear();
		std::ifstream is(fn.c_str());
		if (!is.is_open()) return false; //not opened
		std::queue<std::string> lines;
		std::string line;
		lines.push("{");
		while (IOHelper::readValidLine(is, line, '#')) {
			StringHelper::trim(line);
			if(line.empty() || line[0]=='#') continue; //skip lines like "   #blah"
			lines.push(line);
		}
		if (lines.size()==1) return false; //nothing inside except for the first "{"
		lines.push("}");
		root = ConfigNode::create(lines);
		std::cout << "[Config] loaded: " << fn << std::endl;
		return true;
	}

	/**
	 * automatically search and try loading the cfg file with name fname in:
	 * 1. fdir
	 * 2. current dir
	 * 3. each dir in $PATH environment variable
	 */
	inline bool autoLoad(std::string fname, std::string fdir="") {
		std::string fpath;
		if (fdir != "") {
			fpath = fdir + fname;
			if (load(fpath))
				return true;
			logli("tried but failed to load " << fpath);
		}

		//1. try to use current dir
		fpath = DirHelper::getCurrentDir()+"/"+fname;
		if (load(fpath))
			return true;
		logli("tried but failed to load "<<fpath);

		//2. try to search in path
		std::vector<std::string> all_path=DirHelper::getEnvPath();
		for(int i=0; i<(int)all_path.size(); ++i) {
			std::string& path=all_path[i];
			if(load(path+"/"+fname)) return true;
			logli("tried but failed to load "<<path);
		}
		return false;
	}
};//Config
} //end of ConfigHelper

namespace helper {
typedef helper::Singleton<ConfigHelper::Config> GConfig;
} //end of helper
