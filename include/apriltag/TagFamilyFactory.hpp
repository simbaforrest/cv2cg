#pragma once
/************************************************************************\

  Copyright 2011 The University of Michigan.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software
  and its documentation for educational, research and non-profit
  purposes, without fee, and without a written agreement is
  hereby granted, provided that the above copyright notice and
  the following paragraph appear in all copies.

  THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY OF MICHIGAN "AS IS" AND 
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF MICHIGAN
  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
  Authors:

			Chen Feng
            Laboratory for Interactive Visualization in Engineering (LIVE)
			Department of Civil and Environmental Engineering
            2350 Hayward Street, Suite 2340 GG Brown Building
            University of Michigan
            Ann Arbor, MI 48109-2125
			Phone:    (734)764-8495
			EMail:    simbaforrest@gmail.com
			WWW site: http://www.umich.edu/~cforrest
            
			Vineet R. Kamat
            Laboratory for Interactive Visualization in Engineering (LIVE)
			Department of Civil and Environmental Engineering
            2350 Hayward Street, Suite 2340 GG Brown Building
            University of Michigan
            Ann Arbor, MI 48109-2125
            Phone:    (734)764-4325
			EMail:    vkamat@umich.edu
			WWW site: http://pathfinder.engin.umich.edu

\************************************************************************/

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
