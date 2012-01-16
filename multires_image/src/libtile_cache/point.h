/*
 * Point.h
 * 
 * Copyright (C) 2010 Southwest Research Institute <tbrown@swri.org>
 *
 * Created on: Feb 22, 2010
 *     Author: Tucker Brown
 */
#ifndef POINT_H
#define POINT_H

// C++ standard libraries
#include <string>
#include <sstream>
#include <vector>
#include <cstdio>
#include <cstdlib>

template <class T> class PointT
{
public:
  T X;
  T Y;

  PointT() : Y(0), X(0) {}
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

#endif /*POINT_H*/
