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

#ifndef MULTIRES_IMAGE_GEO_REGISTER_H_
#define MULTIRES_IMAGE_GEO_REGISTER_H_

// C++ standard libraries
#include <string>

#include <multires_image/point.h>
#include <multires_image/bounding_box.h>

namespace multires_image
{
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
}

#endif  // MULTIRES_IMAGE_GEO_REGISTER_H_
