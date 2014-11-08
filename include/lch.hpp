#pragma once

// 1-st Level Headers
#include "Singleton.hpp"
#include "StringHelper.hpp"
#include "IOHelper.hpp"

// 2-nd Level Headers		// depends on:
#include "LogHelper.hpp"	// Singleton
#include "DirHelper.hpp"	// StringHelper
#include "ConfigHelper.hpp" // Singleton StringHelper IOHelper DirHelper

namespace helper {
using namespace StringHelper;
using namespace IOHelper;

using namespace LogHelper;
using namespace DirHelper;
using namespace ConfigHelper;
}
