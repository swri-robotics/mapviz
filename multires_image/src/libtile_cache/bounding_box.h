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

template <class T> class BoundingBox
{
  public:
    BoundingBox() : Width(0), Height(0) {}
    BoundingBox(PointT<T> _topLeft, PointT<T> _bottomRight);

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

    BoundingBox & operator = (const BoundingBox<T> & other)
    {
      if (this != &other) // protect against invalid self-assignment
      {
        topLeft = other.topLeft;
        bottomRight = other.bottomRight;
        Center = other.Center;
        Width = other.Width;
        Height = other.Height;
      }
      return *this;
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

    bool operator == (const BoundingBox<T> & rhs) const;
    bool operator != (const BoundingBox<T>& rhs) const;
};

#endif /*BOUNDING_BOX_H*/
