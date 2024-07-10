#pragma once

#include <cstdlib>
#include <exception>

#include <dwg.h>
#include <argparse/argparse.hpp>
#include <rapidjson/rapidjson.h>



#define DNDS_MACRO_TO_STRING(V) __DNDS_str(V)
#define __DNDS_str(V) #V

#ifndef DWGSIM_CURRENT_COMMIT_HASH
#define DWGSIM_CURRENT_COMMIT_HASH SECTION_UNKNOWN
#endif