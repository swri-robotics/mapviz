/*
* MathUtil.cpp
* 
* Copyright (C) 2010 Southwest Research Institute <tbrown@swri.org>
*
* Created on: Feb 28, 2010
*     Author: Marc Alban
*/

#ifndef MULTIRES_IMAGE_MATH_UTIL_H_
#define MULTIRES_IMAGE_MATH_UTIL_H_

namespace multires_image
{
  class MathUtil
  {
  public:
    static double Round(double value);
    static float Round(float value);
  };
}

#endif  // MULTIRES_IMAGE_MATH_UTIL_H_
