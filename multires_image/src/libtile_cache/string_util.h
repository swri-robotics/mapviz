/*
* StringUtil.h
* 
* Copyright (C) 2010 Southwest Research Institute <tbrown@swri.org>
*
* Created on: Feb 22, 2010
*     Author: Tucker Brown
*/

#ifndef STRING_UTIL_H
#define STRING_UTIL_H

// C++ standard libraries
#include <string>
#include <vector>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <functional>
#include <algorithm>

class StringUtil
{
public:

  // Conversion Methods
  static long ToLong(const std::string& string);
  static int ToInt(const std::string& string);
  static float ToFloat(const std::string& string);
  static double ToDouble(const std::string& string);

  // Split Methods
  static std::vector<std::string>& Split(const std::string &s, char delim, std::vector<std::string> &elems);
  static std::vector<std::string>& Split(const std::vector<std::string> &v, char delim, std::vector<std::string> &elems);
  static std::vector<std::string> Split(const std::string &s, char delim);
  static std::vector<std::string> Split(const std::vector<std::string> &v, char delim);
  static std::vector<std::string> Split(const std::string& s);

  // Trim from start
  static inline std::string& TrimLeft(std::string &s)
  {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
  }

  // Trim from end
  static inline std::string& TrimRight(std::string &s)
  {
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
  }

  // Trim from both ends
  static inline std::string& Trim(std::string &s)
  {
    return TrimLeft(TrimRight(s));
  }

private:
  StringUtil() {}
};

#endif /*STRING_UTIL_H*/
