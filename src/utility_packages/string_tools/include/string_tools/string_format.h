#ifndef STRING_FORMAT_H
#define STRING_FORMAT_H

#include <string>

namespace string_tools
{

std::string formatString(const char* str, ...) __attribute__ ((format (printf, 1, 2) ));

}

#endif

