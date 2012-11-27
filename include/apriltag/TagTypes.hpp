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

}//end of tag
}//end of april

