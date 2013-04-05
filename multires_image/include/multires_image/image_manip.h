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

#ifndef MULTIRES_IMAGE_IMAGE_MANIP_H_
#define MULTIRES_IMAGE_IMAGE_MANIP_H_

// QT libraries
#include <QString>

namespace multires_image
{
  class ImageManip
  {
  public:
    static bool ResizeImage(QString src, QString dst, int percent);
    static bool ResizeImageWindows(QString src, QString dst, int percent);
    static bool ResizeImageLinux(QString src, QString dst, int percent);

    static bool TileImage(QString src, QString dst, int size);
    static bool TileImageWindows(QString src, QString dst, int size);
    static bool TileImageLinux(QString src, QString dst, int size);

    static bool GetDimensions(QString path, int& width, int& height);
    static bool GetDimensionsWindows(QString path, int& width, int& height);
    static bool GetDimensionsLinux(QString path, int& width, int& height);

  private:
    static bool knownPlatform;
    static bool isWindows;
    static bool isLinux;
  };
}

#endif  // MULTIRES_IMAGE_IMAGE_MANIP_H_
