// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#include <swri_math_util/math_util.h>

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

            float max_dim = std::max(width, height);
            m_dimension = swri_math_util::Round(
              std::pow(2.0f, std::ceil(std::log(max_dim)/std::log(2.0f))));

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
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_dimension, m_dimension, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_image.bits());

        // TODO(malban): check for GL error

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        m_textureLoaded = true;
      }
      catch (const std::exception& e)
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

  void Tile::Transform(const swri_transform_util::Transform& transform)
  {
    m_transformed_top_left = transform * m_top_left;
    m_transformed_top_right = transform * m_top_right;
    m_transformed_bottom_left = transform * m_bottom_left;
    m_transformed_bottom_right = transform * m_bottom_right;
  }

  void Tile::Transform(const swri_transform_util::Transform& transform, const swri_transform_util::Transform& offset_tf)
  {
    m_transformed_top_left = offset_tf * (transform * m_top_left);
    m_transformed_top_right = offset_tf * (transform * m_top_right);
    m_transformed_bottom_left = offset_tf * (transform * m_bottom_left);
    m_transformed_bottom_right = offset_tf * (transform * m_bottom_right);
  }
}

