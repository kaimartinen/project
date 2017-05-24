#ifndef STRING_UTIL_H
#define STRING_UTIL_H

#include <vector>
#include <string>
#include <sstream>

namespace string_tools
{

   bool startsWith(const std::string & str, const std::string substr);
   bool endsWith(const std::string & str, const std::string substr);

   std::string toLower(const std::string & s);
   std::string toUpper(const std::string & s);

   std::string trim(const std::string & s);

   std::vector<std::string> split(const std::string & s, const char* delim);

   std::string createFromNumber(int value);
   std::string createFromNumber(double value);

};

#endif

