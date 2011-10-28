namespace april
{
namespace tag
{

//make sure INT64 has 64bits, i.e. sizeof(INT64)=8
#ifdef _WIN32
typedef __int64 INT64;
#else
typedef int64_t INT64;
#endif

}//end of tag
}//end of april

