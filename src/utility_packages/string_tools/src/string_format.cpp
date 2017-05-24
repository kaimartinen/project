#include "string_tools/string_format.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

namespace string_tools
{

std::string formatString(const char* str, ...)
{
   va_list ap;
   va_start(ap, str);
   char* buf;
   if(vasprintf(&buf, str, ap) <= 0) {
      va_end(ap);
      return std::string();
   }
   va_end(ap);

   std::string ret = buf;
   free(buf);
   return ret;
}

}

