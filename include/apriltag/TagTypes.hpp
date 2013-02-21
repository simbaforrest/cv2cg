#pragma once

namespace april
{
namespace tag
{

//make sure INT64 has 64bits, i.e. sizeof(INT64)=8
#ifdef _WIN32
typedef __int64 INT64;
typedef unsigned __int64 UINT64;
#else
typedef int64_t INT64;
typedef uint64_t UINT64;
#endif

//A compile time check to ensure sizeof(INT64)==8
struct __COMPILE_TIME_ASSERT__ {
	int __int64_size_check__[sizeof(INT64)==8?1:-1];
	int __uint64_size_check__[sizeof(UINT64)==8?1:-1];
};

}//end of tag
}//end of april

