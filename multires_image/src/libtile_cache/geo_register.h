/*
* GeoRegister.h
* 
* Copyright (C) 2010 Southwest Research Institute <tbrown@swri.org>
*
* Created on: Feb 26, 2010
*     Author: Marc Alban
*/

#ifndef GEO_REGISTER_H
#define GEO_REGISTER_H

// C++ standard libraries
#include <string>

#include "point.h"
#include "bounding_box.h"

class GeoRegister
{
public:
  GeoRegister(const std::string& path);
  ~GeoRegister(void);

  std::string Image() { return m_image; }
  int ImageWidth() { return m_imageWidth; }
  int ImageHeight() { return m_imageHeight; }
  const PointT<int>& ReferencePixel() const { return m_referencePixel; }
  const PointT<long>& ReferenceCoordinate() const { return m_referenceCoordinate; }
  double MicroDegreesPerPixelX() const { return m_microDegreesPerPixelX; }
  double MicroDegreesPerPixelY() const { return m_microDegreesPerPixelY; }
  const BoundingBox<double>& GetBoundingBox() { return m_boundingBox; }

  std::string LoadGeoRegisterFile();
  void SaveGeoRegisterFile(const std::string& filepath);

  void AdjustReferenceCoordinate(long latitude, long longitude);

private:
  const std::string   m_path;

  std::string         m_image;
  int                 m_imageWidth;
  int                 m_imageHeight;
  double              m_microDegreesPerPixelX;
  double              m_microDegreesPerPixelY;
  PointT<int>         m_referencePixel;
  PointT<long>        m_referenceCoordinate;
  BoundingBox<double> m_boundingBox;

  void ProcessLine(const std::string& line);
  bool IsValid();
  void CalculateReferencePoints();
};

#endif // GEO_REGISTER_H
