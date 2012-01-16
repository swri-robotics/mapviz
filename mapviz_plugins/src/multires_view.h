#ifndef MULTIRES_VIEW_H
#define MULTIRES_VIEW_H

// QT libraries
#include <QGLWidget>

#include <multires_image/tile_set.h>
#include <multires_image/tile_cache.h>

class MultiresView
{
public:
	MultiresView(TileSet* tiles, QGLWidget* widget);
	~MultiresView(void);

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

	double min_scale_;
};

#endif // MULTIRES_VIEW
