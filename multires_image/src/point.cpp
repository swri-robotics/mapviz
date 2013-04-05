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
#include <multires_image/point.h>

namespace multires_image
{

Point::Point() : Latitude(0), Longitude(0)
{
}

Point::Point(long lat, long lon) : Latitude(lat), Longitude(lon)
{
}

Point::Point(const Point& point) : Latitude(point.Latitude), Longitude(point.Longitude)
{

}

Point::Point(std::string lat, std::string lon)
{
  Latitude = convertStringToLong(lat);
  Longitude = convertStringToLong(lon);
}

std::string longToString(long i)
{
  std::stringstream ss;
  std::string s;
  ss << i;
  s = ss.str();
  return s;
}

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems)
{
  std::stringstream ss(s);
  std::string item;
  while(std::getline(ss, item, delim))
  {
    elems.push_back(item);
  }
  return elems;
}


std::vector<std::string> split(const std::string &s, char delim)
{
  std::vector<std::string> elems;
  return split(s, delim, elems);
}

std::string Point::FormattedLat(const PointT<long> refPoint)
{
  long newLat = refPoint.Y + Latitude;
  std::string lat = longToString(newLat);
      return lat.insert(lat.length() - 6, ".");
}

std::string Point::FormattedLon(const PointT<long> refPoint)
{
  long newLon = refPoint.X + Longitude;
  std::string lon = longToString(newLon);
      return lon.insert(lon.length() - 6, ".");
}

long Point::convertStringToLong(std::string number)
{
  std::vector<std::string> tmp = split(number, '.');
  long num = atol(tmp[0].c_str());
  num *= 1000000; //convert to microdegrees
  long tmpLong = atol(tmp[1].c_str());
  while(tmpLong < 99999)
    tmpLong *= 10;
  while(tmpLong > 1000000)
    tmpLong /= 10;

  if (num >= 0)
    num += tmpLong;
  else
    num -= tmpLong;
  return num;
}

}
