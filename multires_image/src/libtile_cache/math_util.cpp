/*
* MathUtil.cpp
* 
* Copyright (C) 2010 Southwest Research Institute <tbrown@swri.org>
*
* Created on: Feb 28, 2010
*     Author: Marc Alban
*/

// C++ standard libraries
#include <cmath>

#include "math_util.h"

double MathUtil::Round(double value)
{
  return (value > 0.0) ? floor(value + 0.5) : ceil(value - 0.5);
}

float MathUtil::Round(float value)
{
  return (value > 0.0f) ? floor(value + 0.5f) : ceil(value - 0.5f);
}
