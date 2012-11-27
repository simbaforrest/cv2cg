#pragma once
/*	singleton.h
 *		Author: (c) Chen Feng <simbaforrest@gmail.com>
 */

namespace helper {

	template <class T>
	class Singleton
	{
	public:
		static T& Instance() {
			static T me;
			return me;
		}
	protected:
		Singleton();
		~Singleton();
	private:
		Singleton(Singleton const&);
		Singleton& operator=(Singleton const&);
	};//end of class Singleton

}//end of namespace helper
