/*
* ImageManip.h
*
* Copyright (C) 2010 Southwest Research Institute <malban@swri.org>
*
* Created on: Mar 01, 2010
*     Author: Marc Alban
*/

#ifndef TILECACHE_H
#define TILECACHE_H

// C++ standard libraries
#include <vector>
#include <stack>
#include <queue>
#include <map>

// QT libraries
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QGLWidget>

#include <multires_image/point.h>
#include <multires_image/tile_set.h>
#include <multires_image/tile.h>

namespace multires_image
{

class TileCache : public QObject
{
Q_OBJECT

public:
	TileCache(TileSet* tileSet, QGLWidget* widget);
	~TileCache(void);

	void Load(Tile* tile);
	void Precache(const PointT<double>& position);
	void Precache(double x, double y);

	void SetCurrentLayer(int layer) { m_currentLayer = layer; }

	void Exit();

public Q_SLOTS:
  void LoadTextureSlot(Tile*);
  void DeleteTextureSlot(Tile*);

Q_SIGNALS:
	void SignalLoadTexture(Tile*);
	void SignalDeleteTexture(Tile*);
	void SignalMemorySize(long);

private:
	TileSet*                  m_tileSet;
	QGLWidget*                m_widget;
	int                       m_currentLayer;
	PointT<double> m_currentPosition;
	bool                      m_exit;
	long                      m_memorySize;

	std::vector<std::queue<Tile*> > m_precacheRequests;
	std::stack<Tile*>               m_renderRequests;
	std::map<long, Tile*>           m_textureLoaded;
	std::map<long, Tile*>           m_renderRequestSet;
	std::map<long, Tile*>           m_precacheRequestSet;

	void PrecacheLayer(int layer, const PointT<double>& position, int size);
	void LoadTexture(Tile* tile);
	void UnloadTexture(Tile* tile);

	class CacheThread : public QThread
	{
	public:
		CacheThread(TileCache* parent) : p(parent) {}
		virtual void run();

	private:
		TileCache* p;
	};
	friend class CacheThread;

	class FreeThread : public QThread
	{
	public:
		FreeThread(TileCache* parent) : p(parent) {}
		virtual void run();

	private:
		TileCache* p;
	};
	friend class FreeThread;
	
	CacheThread m_cacheThread;
	FreeThread  m_freeThread;

	QMutex      m_renderRequestsLock;
	QMutex      m_renderRequestSetLock;
	QMutex      m_precacheRequestsLock;
	QMutex      m_precacheRequestSetLock;
	QMutex      m_textureLoadedLock;
};

}

#endif // TILECACHE_H
