/*
 * BoundingBox.h
 * 
 * Copyright (C) 2010 Southwest Research Institute <tbrown@swri.org>
 *
 * Created on: Feb 22, 2010
 *     Author: Tucker Brown
 */

#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

// C++ standard libraries
#include <algorithm>

#include "point.h"

#include <ros/ros.h>

template <class T>
class BoundingBox
{
  public:
    BoundingBox() :
      Width(0),
      Height(0)
    {
      // default constructor
    }

    BoundingBox(PointT<T> _topLeft, PointT<T> _bottomRight) :
      topLeft(_topLeft),
      bottomRight(_bottomRight)
    {
      this->Update();
    }

    PointT<T> topLeft;
    PointT<T> bottomRight;
    T Width;
    T Height;
    PointT<T> Center;

    void Update()
    {
      Width = bottomRight.X - topLeft.X;
      Height = bottomRight.Y - topLeft.Y;
      Center = PointT<T>((bottomRight.X + topLeft.X) / 2, (bottomRight.Y + topLeft.Y) / 2);
    }

    bool operator== (const BoundingBox<T> & rhs) const
    {
      return topLeft == rhs.topLeft &&
        bottomRight == rhs.bottomRight &&
        Center == rhs.Center &&
        Width == rhs.Width &&
        Height == rhs.Height;
    }

    BoundingBox& operator= (const BoundingBox<T> & other)
    {
      if (this != &other) // protect against invalid self-assignment
      {
        topLeft     = other.topLeft;
        bottomRight = other.bottomRight;
        Center      = other.Center;
        Width       = other.Width;
        Height      = other.Height;
      }
      return *this;
    }

    bool operator!= (const BoundingBox<T>& rhs) const
    {
      return !(*this == rhs);
    }

    bool Contains(T x, T y)
    {
      if (x < std::min(topLeft.X, bottomRight.X) || x > std::max(topLeft.X, bottomRight.X))
      {
        return false;
      }
      else if (y < std::min(topLeft.Y, bottomRight.Y) || y > std::max(topLeft.Y, bottomRight.Y))
      {
        return false;
      }

      return true;
    }
};

#endif /*BOUNDING_BOX_H*/
