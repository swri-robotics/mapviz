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

#ifndef MULTIRES_IMAGE_POINT_H_
#define MULTIRES_IMAGE_POINT_H_

// C++ standard libraries
#include <string>
#include <sstream>
#include <vector>
#include <cstdio>
#include <cstdlib>

namespace multires_image
{
  template <class T> class PointT
  {
  public:
    T X;
    T Y;

    PointT() : X(0), Y(0) {}
    PointT(T x, T y) : X(x), Y(y) {}
    PointT(const PointT& point) : X(point.X), Y(point.Y) {}

    template < typename T2 >
    PointT & operator = (const PointT<T2> & other)
    {
      if (this != (PointT<T>*)&other) // protect against invalid self-assignment
      {
        X = (T)other.X;
        Y = (T)other.Y;
      }
      return *this;
    }

    template < typename T2 >
    bool operator == (const PointT<T2> & point) const
    {
      return X == point.X && Y == point.Y;
    }

    template < typename T2 >
    bool operator != (const PointT<T2>& point) const
    {
      return !(*this == point);
    }
  };

  class Point
  {
    long convertStringToLong(std::string number);

    public:
      long Latitude;
      long Longitude;

      Point();
      Point(long lat, long lon);
      Point(std::string lat, std::string lon);
      Point(const Point& point);

      //accessors
      std::string FormattedLat(const PointT<long> refPoint);
      std::string FormattedLon(const PointT<long> refPoint);
  };

  // Convenience typedefs
  typedef PointT<int32_t> Int32Point;
  typedef PointT<double>  DoublePoint;
}

#endif  // MULTIRES_IMAGE_POINT_H_
