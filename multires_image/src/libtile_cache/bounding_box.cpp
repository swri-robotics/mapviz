/*
 * BoundingBox.cpp
 * 
 * Copyright (C) 2010 Southwest Research Institute <tbrown@swri.org>
 *
 * Created on: Feb 22, 2010
 *     Author: Tucker Brown
 */

#include "bounding_box.h"

template<class T>
BoundingBox<T>::BoundingBox(PointT<T> _topLeft, PointT<T> _bottomRight)
{
  topLeft = _topLeft;
  bottomRight = _bottomRight;
  Width = bottomRight.Longitude - topLeft.Longitude;
  Height = bottomRight.Latitude - topLeft.Latitude;
  Center = PointT<T>((bottomRight.Longitude + topLeft.Longitude) / 2, (bottomRight.Latitude + topLeft.Latitude) / 2);
}

template<class T>
bool BoundingBox<T>::operator == (const BoundingBox<T> & rhs) const
{
  return topLeft == rhs.topLeft &&
    bottomRight == rhs.bottomRight &&
    Center == rhs.Center &&
    Width == rhs.Width &&
    Height == rhs.Height;
}

template<class T>
bool BoundingBox<T>::operator != (const BoundingBox<T>& rhs) const
{
  return !(*this == rhs);
}
