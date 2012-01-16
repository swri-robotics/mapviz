/*
* ImageManip.h
*
* Copyright (C) 2010 Southwest Research Institute <malban@swri.org>
*
* Created on: Mar 01, 2010
*     Author: Marc Alban
*/

#ifndef TILE_H
#define TILE_H

// C++ standard libraries
#include <string>

// QT libraries
#include <QImage>
#include <QMutex>

#include <georeference/georeference.h>
#include <geospatial_index/wgs84_utm.h>

#include "point.h"

#ifndef GL_CLAMP_TO_EDGE  
#define GL_CLAMP_TO_EDGE 0x812F 
#endif

class Tile
{
public:
	Tile(const georeference::GeoReference& geo, const geospatial_index::WGS84UTM& utm,
	  const std::string& path, int column, int row, int level, const PointT<double>& topLeft,
	  const PointT<double>& topRight, const PointT<double>& bottomLeft, const PointT<double>& bottomRight);
	~Tile(void);

  void Initialize();
	bool Exists();
	bool HasUtm() const { return m_hasUtm; }
	bool Failed() { return m_failed; }
	bool TextureLoaded() { return m_textureLoaded; }
	const QImage& Image() { return m_image; }
	long TileID() { return m_tileId; }
	int Layer() { return m_level; }
	int MemorySize() { return m_memorySize; }
	int Row() { return m_row; }
	int Column() { return m_column; }

	bool LoadImageToMemory(bool gl = true);
	void UnloadImage();

	bool LoadTexture();
	void UnloadTexture();

	void Draw();
	void DrawUtm();
	void DrawRelative();

  void GetUtmPosition(
    double& top_left_x, double& top_left_y,
    double& top_right_x, double& top_right_y,
    double& bottom_right_x, double& bottom_right_y,
    double& bottom_left_x, double& bottom_left_y);

	void SetRelativePosition(
	  double top_left_x, double top_left_y,
    double top_right_x, double top_right_y,
    double bottom_right_x, double bottom_right_y,
    double bottom_left_x, double bottom_left_y);

	void AdjustGeoReference(double latitude, double longitude);

private:

  void ConvertToUtm();

  const georeference::GeoReference& m_geo;
  const geospatial_index::WGS84UTM& m_utm;
	const std::string   m_path;
	const int           m_column;
	const int           m_row;
	const int           m_level;
	
  PointT<double>      m_topLeft;
  PointT<double>      m_topRight;
  PointT<double>      m_bottomRight;
  PointT<double>      m_bottomLeft;

  PointT<double>      m_topLeftUtm;
  PointT<double>      m_topRightUtm;
  PointT<double>      m_bottomRightUtm;
  PointT<double>      m_bottomLeftUtm;


  PointT<double>      m_topLeftRelative;
  PointT<double>      m_topRightRelative;
  PointT<double>      m_bottomRightRelative;
  PointT<double>      m_bottomLeftRelative;


  bool                m_hasUtm;
	bool                m_failed;
	bool                m_textureLoaded;
	int                 m_dimension;
	int                 m_textureId;
	long                m_tileId;
	int                 m_memorySize;
	QImage              m_image;
	QMutex              m_mutex;
};

#endif // TILE_H
