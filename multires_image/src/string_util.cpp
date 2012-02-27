/*
* StringUtil.cpp
* 
* Copyright (C) 2010 Southwest Research Institute <tbrown@swri.org>
*
* Created on: Feb 22, 2010
*     Author: Tucker Brown
*/

#include <multires_image/string_util.h>

// C++ standard libraries
#include <exception>

namespace multires_image
{

long StringUtil::ToLong(const std::string& string)
{
  std::stringstream ss(string);

  // Throw exceptions on errors.
  ss.exceptions(std::ios_base::failbit | std::ios_base::badbit);

  long value;
  ss >> value;
  return value;
}

int StringUtil::ToInt(const std::string& string)
{
  std::stringstream ss(string);

  // Throw exceptions on errors.
  ss.exceptions(std::ios_base::failbit | std::ios_base::badbit);

  int value;
  ss >> value;
  return value;
}

float StringUtil::ToFloat(const std::string& string)
{
  std::stringstream ss(string);

  // Throw exceptions on errors.
  ss.exceptions(std::ios_base::failbit | std::ios_base::badbit);

  float value;
  ss >> value;
  return value;
}

double StringUtil::ToDouble(const std::string& string)
{
  std::stringstream ss(string);

  // Throw exceptions on errors.
  ss.exceptions(std::ios_base::failbit | std::ios_base::badbit);

  double value;
  ss >> value;
  return value;
}

std::vector<std::string>& StringUtil::Split(const std::string& s, char delim, std::vector<std::string>& elems)
{
  std::stringstream ss(s);
  std::string item;
  while(std::getline(ss, item, delim))
  {
    // TODO is this check neccessary?
    if (item.size() > 0)
    {
      elems.push_back(item);
    }
  }
  return elems;
}

std::vector<std::string>& StringUtil::Split(const std::vector<std::string>& v, char delim, std::vector<std::string>& elems)
{
  for(unsigned int x = 0; x < v.size(); x++)
  {
    std::string s = v[x];
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim))
    {
      // TODO is this check neccessary?
      if (item.size() > 0)
      {
        elems.push_back(item);
      }
    }
  }
  return elems;
}

std::vector<std::string> StringUtil::Split(const std::string &s, char delim)
{
  std::vector<std::string> elems;
  return Split(s, delim, elems);
}

std::vector<std::string> StringUtil::Split(const std::vector<std::string> &v, char delim)
{
  std::vector<std::string> elems;
  return Split(v, delim, elems);
}

std::vector<std::string> StringUtil::Split(const std::string &s)
{
  // Split on newlines.
  std::vector<std::string> elems1;
  Split(s, '\n', elems1);

  // Split on horizontal tabs.
  std::vector<std::string> elems2;
  Split(elems1, '\t', elems2);

  // Split on vertical tabs.
  std::vector<std::string> elems3;
  Split(elems2, '\v', elems3);

  // Split on feeds.
  std::vector<std::string> elems4;
  Split(elems3, '\f', elems4);

  // Split on carriage return.
  std::vector<std::string> elems5;
  Split(elems4, '\r', elems5);

  // Split on spaces
  std::vector<std::string> elems6;
  Split(elems5, ' ', elems6);

  return elems6;
}

}
