#ifndef TILEVIEW_H
#define TILEVIEW_H

// QT libraries
#include <QGLWidget>

#include "tile_set.h"
#include "tile_cache.h"
#include "point.h"
#include "bounding_box.h"

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

#endif // TILEVIEW_H
