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

#ifndef MULTIRES_IMAGE_BOUNDING_BOX_H
#define MULTIRES_IMAGE_BOUNDING_BOX_H

// C++ standard libraries
#include <algorithm>

#include <multires_image/point.h>

#include <ros/ros.h>

namespace multires_image
{

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

  // Convenience typedefs
  typedef BoundingBox<int32_t> Int32Box;
  typedef BoundingBox<double>  DoubleBox;
}

#endif  // MULTIRES_IMAGE_BOUNDING_BOX_H
