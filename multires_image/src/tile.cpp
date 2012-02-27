/*
* ImageManip.h
*
* Copyright (C) 2010 Southwest Research Institute <malban@swri.org>
*
* Created on: Mar 01, 2010
*     Author: Marc Alban
*/

#include <multires_image/tile.h>

// C++ standard libraries
#include <algorithm>
#include <exception>
#include <iostream>
#include <cmath>

// QT libraries
#include <QGLWidget>
#include <QFile>

#include <multires_image/math_util.h>

namespace multires_image
{

Tile::Tile(const georeference::GeoReference& geo, const geospatial_index::WGS84UTM& utm,
       const std::string& path, int column, int row, int level,  
		   const PointT<double>& topLeft, const PointT<double>& topRight, 
		   const PointT<double>& bottomLeft, const PointT<double>& bottomRight) :
	m_geo(geo),
	m_utm(utm),
	m_path(path),
	m_column(column),
	m_row(row),
	m_level(level),
	m_topLeft(topLeft),
	m_topRight(topRight),
	m_bottomRight(bottomRight),
	m_bottomLeft(bottomLeft),
	m_hasUtm(false),
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

void Tile::ConvertToUtm()
{
  if (!m_hasUtm && m_geo.Datum() == "wgs84")
  {
    if (m_geo.Projection() == "utm")
    {
      m_topLeftUtm = m_topLeft;
      m_topRightUtm = m_topRight;
      m_bottomRightUtm = m_bottomRight;
      m_bottomLeftUtm = m_bottomLeft;
      m_hasUtm = true;
    }
    else if (m_geo.Projection() == "geographic")
    {
      int zone;
      char band;
        
      m_utm.GetUtm(m_topLeft.Y, m_topLeft.X, zone, band, m_topLeftUtm.X, m_topLeftUtm.Y);
      m_utm.GetUtm(m_topRight.Y, m_topRight.X, zone, band, m_topRightUtm.X, m_topRightUtm.Y);
      m_utm.GetUtm(m_bottomRight.Y, m_bottomRight.X, zone, band, m_bottomRightUtm.X, m_bottomRightUtm.Y);
      m_utm.GetUtm(m_bottomLeft.Y, m_bottomLeft.X, zone, band, m_bottomLeftUtm.X, m_bottomLeftUtm.Y);
      m_hasUtm = true;
    }
  }
}

void Tile::Initialize()
{
  ConvertToUtm();
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

          ConvertToUtm();

					int maxDimension = std::max(width, height);
					m_dimension = (int)MathUtil::Round(pow(2, ceil(log((float)maxDimension)/log(2.0f))));

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
		catch(std::exception e)
		{
			std::cout << "An exception occured loading image: " << e.what() << std::endl;
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

			// TODO check for GL error

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

			m_textureLoaded = true;
		}
		catch(std::exception e)
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

			glTexCoord2f(0, 1); glVertex2f(m_topLeft.X, m_topLeft.Y);
			glTexCoord2f(1, 1); glVertex2f(m_topRight.X, m_topRight.Y);
			glTexCoord2f(1, 0); glVertex2f(m_bottomRight.X, m_bottomRight.Y);
			glTexCoord2f(0, 0); glVertex2f(m_bottomLeft.X, m_bottomLeft.Y);

			glEnd();
		}
	}
}

void Tile::DrawUtm()
{
	if (!m_failed)
	{
		if (m_textureLoaded)
		{
			glBindTexture(GL_TEXTURE_2D, m_textureId);

			glBegin(GL_QUADS);

			glTexCoord2f(0, 1); glVertex2f(m_topLeftUtm.X, m_topLeftUtm.Y);
			glTexCoord2f(1, 1); glVertex2f(m_topRightUtm.X, m_topRightUtm.Y);
			glTexCoord2f(1, 0); glVertex2f(m_bottomRightUtm.X, m_bottomRightUtm.Y);
			glTexCoord2f(0, 0); glVertex2f(m_bottomLeftUtm.X, m_bottomLeftUtm.Y);

			glEnd();
		}
	}
}

void Tile::DrawRelative()
{
	if (!m_failed)
	{
		if (m_textureLoaded)
		{
			glBindTexture(GL_TEXTURE_2D, m_textureId);

			glBegin(GL_QUADS);

			glTexCoord2f(0, 1); glVertex2f(m_topLeftRelative.X, m_topLeftRelative.Y);
			glTexCoord2f(1, 1); glVertex2f(m_topRightRelative.X, m_topRightRelative.Y);
			glTexCoord2f(1, 0); glVertex2f(m_bottomRightRelative.X, m_bottomRightRelative.Y);
			glTexCoord2f(0, 0); glVertex2f(m_bottomLeftRelative.X, m_bottomLeftRelative.Y);

			glEnd();
		}
	}
}

void Tile::GetUtmPosition(
  double& top_left_x, double& top_left_y,
  double& top_right_x, double& top_right_y,
  double& bottom_right_x, double& bottom_right_y,
  double& bottom_left_x, double& bottom_left_y)
{
  top_left_y = m_topLeftUtm.Y;
  top_left_x = m_topLeftUtm.X;
  top_right_y = m_topRightUtm.Y;
  top_right_x = m_topRightUtm.X;
  bottom_right_y = m_bottomRightUtm.Y;
  bottom_right_x = m_bottomRightUtm.X;
  bottom_left_y = m_bottomLeftUtm.Y;
  bottom_left_x = m_bottomLeftUtm.X;
}

void Tile::SetRelativePosition(
  double top_left_x, double top_left_y,
  double top_right_x, double top_right_y,
  double bottom_right_x, double bottom_right_y,
  double bottom_left_x, double bottom_left_y)
{
  m_topLeftRelative.Y = top_left_y;
  m_topLeftRelative.X = top_left_x;
  m_topRightRelative.Y = top_right_y;
  m_topRightRelative.X = top_right_x;
  m_bottomRightRelative.Y = bottom_right_y;
  m_bottomRightRelative.X = bottom_right_x;
  m_bottomLeftRelative.X = bottom_left_x;
  m_bottomLeftRelative.Y = bottom_left_y;
}

void Tile::AdjustGeoReference(double latitude, double longitude)
{
	m_topLeft.Y += latitude;
	m_topLeft.X += longitude;
  m_topRight.Y += latitude;
	m_topRight.X += longitude;
	m_bottomRight.Y += latitude;
	m_bottomRight.X += longitude;
	m_bottomLeft.Y += latitude;
	m_bottomLeft.X += longitude;

  // TODO also adjust UTM coords
}

}

