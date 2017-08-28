/*
 * scriptloader.hpp
 *
 *      Author: martin
 */

#ifndef SCRIPTLOADERCLASS_HPP
#define SCRIPTLOADERCLASS_HPP

#include "lua.hpp"

class ScriptLoaderClass
{
public:
	ScriptLoaderClass() {}
	virtual ~ScriptLoaderClass() {};

	virtual int LoadFromFolder(const char *scriptFolder) {return 0;}

};

#endif
