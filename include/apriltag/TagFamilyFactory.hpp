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

/* TagFamilyFactory.hpp
	to create TagFamily
*/

#include "TagTypes.hpp"

#include "Tag16h5.hpp"
#include "Tag25h7.hpp"
#include "Tag25h9.hpp"
#include "Tag36h9.hpp"
#include "Tag36h11.hpp"

#include "TagFamily.hpp"

namespace april
{
namespace tag
{

struct TagFamilyFactory {
	enum SUPPORT_TYPE {TAG16H5, TAG25H7, TAG25H9, TAG36H9, TAG36H11, TAGTOTAL};
	static const std::string SUPPORT_NAME[];

	inline static Ptr<TagFamily> create(unsigned int type) {
		Ptr<TagFamily> ret;
		vector<INT64> codes;
		switch(type) {
#define MacroCreate(d,m) \
	codes.insert(codes.begin(), codes##d##h##m, codes##d##h##m+sizeof(codes##d##h##m)/sizeof(INT64)); \
	ret = new TagFamily(d,m,codes)
		case TAG16H5:
			MacroCreate(16,5);
			break;
		case TAG25H7:
			MacroCreate(25,7);
			break;
		case TAG25H9:
			MacroCreate(25,9);
			break;
		case TAG36H9:
			MacroCreate(36,9);
			break;
		case TAG36H11:
			MacroCreate(36,11);
			break;
		default:
			std::cerr<<"[TagFamily error] Unknown type!"<<std::endl;
#undef MacroCreate
		}
		return ret;
	}
};

const std::string TagFamilyFactory::SUPPORT_NAME[] = {"TAG16H5", "TAG25H7", "TAG25H9", "TAG36H9", "TAG36H11"};

}//end of tag
}//end of april
