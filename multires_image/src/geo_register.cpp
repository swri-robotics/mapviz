/*
* GeoRegister.cpp
* 
* Copyright (C) 2010 Southwest Research Institute <tbrown@swri.org>
*
* Created on: Feb 26, 2010
*     Author: Marc Alban
*/

#include <multires_image/geo_register.h>

// C++ standard libraries
#include <fstream>
#include <iostream>
#include <vector>
#include <exception>

#include <multires_image/math_util.h>
#include <multires_image/string_util.h>

namespace multires_image
{

GeoRegister::GeoRegister(const std::string& path) :
  m_path(path),
  m_image(""),
  m_imageWidth(-1),
  m_imageHeight(-1),
  m_microDegreesPerPixelX(0),
  m_microDegreesPerPixelY(0),
  m_referencePixel(-1,-1),
  m_referenceCoordinate(-1,-1)
{

}

GeoRegister::~GeoRegister(void)
{
}

std::string GeoRegister::LoadGeoRegisterFile()
{
  std::string errorString;

  try
  {
    std::ifstream infile(m_path.c_str());
    if (infile.is_open())
    {
      bool finished = false;
      bool commentOpen = false;
      std::string line;

      while (!infile.eof() && !finished)
      {
        getline(infile, line);

        // Check if a multiline comment is ended in this line.
        if (commentOpen)
        {
          std::string::size_type commentEnd = line.find("*/");
          if (commentEnd != std::string::npos)
          {
            commentOpen = false;
            line = line.substr(commentEnd + 2);
          }
          else
          {
            // This entire line is commented out.
            line = "";
          }
        }

        // Remove any comments.
        std::string::size_type commentStart = line.find("/*");
        while (!commentOpen && commentStart != std::string::npos)
        {
          std::string::size_type commentEnd = line.find("*/");
          if (commentEnd != std::string::npos)
          {
            line.erase(commentStart, commentEnd - commentStart);
            commentStart = line.find("/*");
          }
          else
          {
            commentOpen = true;
            line = line.substr(0, commentStart);
          }
        }

        if (line.length() > 0)
        {
          ProcessLine(line);
        }
      }
    }

    infile.close();

    if (IsValid())
    {
      CalculateReferencePoints();
    }
    else
    {
      errorString = "Invalid GEO register file.";
    }
  }
  catch (std::exception e)
  {
    errorString = "An exception occured loading the geo-register file: ";
    errorString += e.what();
    std::cout << errorString << std::endl;
  }

  return errorString;
}

void GeoRegister::ProcessLine(const std::string& line)
{
  std::vector<std::string> values = StringUtil::Split(line);

  if (values.size() > 1)
  {
    if (values[0].compare("ZoomImage") == 0)
    {
      // Reconstruct the file path.
      for (unsigned int i = 1; i < values.size() - 1; i++)
      {
        m_image += values[i] + ' ';
      }
      m_image += values[values.size() - 1];
    }
    else if (values[0].compare("ZpXcenter") == 0)
    {
      m_referencePixel.X = StringUtil::ToInt(values[1]);
    }
    else if (values[0].compare("ZpYcenter") == 0)
    {
      m_referencePixel.Y = StringUtil::ToInt(values[1]);
    }
    else if (values[0].compare("ZpXscale") == 0)
    {
      m_microDegreesPerPixelX = StringUtil::ToDouble(values[1]);
    }
    else if (values[0].compare("ZpYscale") == 0)
    {
      m_microDegreesPerPixelY = StringUtil::ToDouble(values[1]);
    }
    else if (values[0].compare("ZpWidth") == 0)
    {
      m_imageWidth = StringUtil::ToInt(values[1]);
    }
    else if (values[0].compare("ZpHeight") == 0)
    {
      m_imageHeight = StringUtil::ToInt(values[1]);
    }
    else if (values[0].compare("pLocalPoint") == 0 && values.size() > 2)
    {
      // Assumes degrees.
      double latitude = StringUtil::ToDouble(values[1]);
      double longitude = StringUtil::ToDouble(values[2]);
      m_referenceCoordinate.Y = (long) MathUtil::Round(latitude * 1000000);
      m_referenceCoordinate.X = (long) MathUtil::Round(longitude * 1000000);
    }
  }
}

void GeoRegister::SaveGeoRegisterFile(const std::string& filepath)
{
  try
  {
    std::ofstream outfile(filepath.c_str());
    if (outfile.is_open())
    {
      outfile.setf(std::ios::fixed,std::ios::floatfield);
      outfile.precision(6);
      outfile << "ZoomImage " << m_image << std::endl;
      outfile << "pLocalPoint " << ((double)ReferenceCoordinate().Y / 1000000.0) << " " << ((double)ReferenceCoordinate().X / 1000000.0) << std::endl;
      outfile << "ZpXcenter " << ReferencePixel().X << std::endl;
      outfile << "ZpYcenter " << ReferencePixel().Y << std::endl;
      outfile << "ZpXscale " << MicroDegreesPerPixelX() << std::endl;
      outfile << "ZpYscale " << MicroDegreesPerPixelY() << std::endl;
      outfile << "ZpWidth " << ImageWidth() << std::endl;
      outfile << "ZpHeight " << ImageHeight() << std::endl;
    }
  }
  catch(std::string s)
  {
    std::cout << s << std::endl;
  }
  catch(...)
  {
    std::cout << "Failed to generate GeoReference file." << std::endl;
  }
}

bool GeoRegister::IsValid()
{
  return m_image.length() > 0 && m_imageWidth != -1 && m_imageHeight != -1
    && m_microDegreesPerPixelX != 0 && m_microDegreesPerPixelY != 0
    && m_referencePixel.X != -1 && m_referencePixel.Y != -1
    && m_referenceCoordinate.X != -1 && m_referenceCoordinate.Y != -1;
}

void GeoRegister::CalculateReferencePoints()
{
  double changeX = m_referencePixel.X * m_microDegreesPerPixelX;
  double changeY = m_referencePixel.Y * m_microDegreesPerPixelY * -1;

  m_boundingBox.topLeft.X = m_referenceCoordinate.X - changeX;
  m_boundingBox.topLeft.Y = m_referenceCoordinate.Y - changeY;

  changeX = (m_imageWidth - m_referencePixel.X) * m_microDegreesPerPixelX;
  changeY = (m_imageHeight - m_referencePixel.Y) * m_microDegreesPerPixelY * -1;

  m_boundingBox.bottomRight.X = m_referenceCoordinate.X + changeX;
  m_boundingBox.bottomRight.Y = m_referenceCoordinate.Y + changeY;

  m_boundingBox.Update();
}

void GeoRegister::AdjustReferenceCoordinate(long latitude, long longitude)
{
  m_referenceCoordinate.Y += latitude;
  m_referenceCoordinate.X += longitude;
}

}
