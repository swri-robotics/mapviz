#ifndef TILEVIEW_H
#define TILEVIEW_H

// QT libraries
#include <QGLWidget>

#include <multires_image/tile_set.h>
#include <multires_image/tile_cache.h>
#include <multires_image/point.h>
#include <multires_image/bounding_box.h>

namespace multires_image
{

class TileView
{
public:
	TileView(TileSet* tiles, QGLWidget* widget);
	~TileView(void);

	const TileCache* Cache() { return &m_cache; }

	void SetView(double x, double y, double radius, double scale);

	void Draw();

	void Exit() { m_cache.Exit(); }

private:
	TileSet*   m_tiles;
	TileCache  m_cache;
	int        m_currentLayer;
	int        m_startRow;
	int        m_startColumn;
	int        m_endRow;
	int        m_endColumn;
	double     min_scale_;
};

}

#endif // TILEVIEW_H
