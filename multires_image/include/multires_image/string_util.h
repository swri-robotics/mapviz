// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-58058A
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Steve Dellenback <sdellenback@swri.org> (210) 522-3914
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#ifndef MULTIRES_IMAGE_STRING_UTIL_H_
#define MULTIRES_IMAGE_STRING_UTIL_H_

// C++ standard libraries
#include <string>
#include <vector>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <functional>
#include <algorithm>

namespace multires_image
{

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

}

#endif  // MULTIRES_IMAGE_STRING_UTIL_H_
