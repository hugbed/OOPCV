#pragma once

#include <iostream>

#ifdef _WIN
#include <windows.h>
#endif

namespace FileIO
{
	std::string getCurrentDirectory() {
#ifdef _WIN
		char buffer[MAX_PATH];
		GetModuleFileName(NULL, buffer, MAX_PATH);
		std::string::size_type pos = std::string(buffer).find_last_of("\\/");
		return std::string(buffer).substr(0, pos);
#else
		return std::string(".");
#endif
	}
}