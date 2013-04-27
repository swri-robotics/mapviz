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

#include <multires_image/tile.h>

// C++ standard libraries
#include <cmath>
#include <algorithm>
#include <exception>
#include <iostream>

// QT libraries
#include <QGLWidget>
#include <QFile>

#include <math_util/math_util.h>

namespace multires_image
{
  Tile::Tile(
         const std::string& path, int column, int row, int level,
         const tf::Point& topLeft, const tf::Point& topRight,
         const tf::Point& bottomLeft, const tf::Point& bottomRight) :
    m_path(path),
    m_column(column),
    m_row(row),
    m_level(level),
    m_top_left(topLeft),
    m_top_right(topRight),
    m_bottom_right(bottomRight),
    m_bottom_left(bottomLeft),
    m_transformed_top_left(topLeft),
    m_transformed_top_right(topRight),
    m_transformed_bottom_right(bottomRight),
    m_transformed_bottom_left(bottomLeft),
    m_failed(false),
    m_textureLoaded(false),
    m_dimension(0),
    m_textureId(0),
    m_tileId(1000000 * level + 1000 * column + row),
    m_memorySize(0)
  {
  }

  Tile::~Tile(void)
  {
  }

  bool Tile::Exists()
  {
    return QFile::exists(m_path.c_str());
  }

  bool Tile::LoadImageToMemory(bool gl)
  {
    if (!m_failed)
    {
      m_mutex.lock();

      try
      {
        QImage nullImage;
        m_image = nullImage;

        if (m_image.load(m_path.c_str()))
        {
          if (gl)
          {
            int width = m_image.width();
            int height = m_image.height();

            int maxDimension = std::max(width, height);
            m_dimension = (int32_t)math_util::Round(pow(2, ceil(log((float)maxDimension)/log(2.0f))));

            if (width != m_dimension || height != m_dimension)
            {
              m_image = m_image.scaled(m_dimension, m_dimension, Qt::IgnoreAspectRatio, Qt::FastTransformation);
            }

            m_memorySize = m_dimension * m_dimension * 4;

            m_image = QGLWidget::convertToGLFormat(m_image);
          }
        }
        else
        {
          m_failed = true;
        }
      }
      catch(std::exception& e)
      {
        std::cout << "An exception occurred loading image: " << e.what() << std::endl;
        m_failed = true;
      }

      m_mutex.unlock();
    }

    return !m_failed;
  }

  void Tile::UnloadImage()
  {
    m_mutex.lock();

    QImage nullImage;
    m_image = nullImage;

    m_mutex.unlock();
  }

  bool Tile::LoadTexture()
  {
    if (!m_textureLoaded && !m_failed)
    {
      m_mutex.lock();

      try
      {
        GLuint ids[1];
        glGenTextures(1, &ids[0]);
        m_textureId = ids[0];

        glBindTexture(GL_TEXTURE_2D, m_textureId);
        glTexImage2D(GL_TEXTURE_2D, 0, 3, m_dimension, m_dimension, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_image.bits());

        // TODO(malban): check for GL error

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        m_textureLoaded = true;
      }
      catch(std::exception& e)
      {
        std::cout << "An exception occured loading texture: " << e.what() << std::endl;
        m_failed = true;
      }

      m_mutex.unlock();
    }

    return m_textureLoaded;
  }

  void Tile::UnloadTexture()
  {
    m_mutex.lock();

    if (m_textureLoaded)
    {
      m_textureLoaded = false;
      GLuint ids[1];
      ids[0] = m_textureId;
      glDeleteTextures(1, &ids[0]);
    }

    m_mutex.unlock();
  }

  void Tile::Draw()
  {
    if (!m_failed)
    {
      if (m_textureLoaded)
      {
        glBindTexture(GL_TEXTURE_2D, m_textureId);

        glBegin(GL_QUADS);

        glTexCoord2f(0, 1); glVertex2f(m_transformed_top_left.x(), m_transformed_top_left.y());
        glTexCoord2f(1, 1); glVertex2f(m_transformed_top_right.x(), m_transformed_top_right.y());
        glTexCoord2f(1, 0); glVertex2f(m_transformed_bottom_right.x(), m_transformed_bottom_right.y());
        glTexCoord2f(0, 0); glVertex2f(m_transformed_bottom_left.x(), m_transformed_bottom_left.y());

        glEnd();
      }
    }
  }

  void Tile::Transform(const transform_util::Transform& transform)
  {
    m_transformed_top_left = transform * m_top_left;
    m_transformed_top_right = transform * m_top_right;
    m_transformed_bottom_left = transform * m_bottom_left;
    m_transformed_bottom_right = transform * m_bottom_right;
  }
}

