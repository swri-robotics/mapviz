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

#ifndef MULTIRES_IMAGE_TILE_CACHE_H_
#define MULTIRES_IMAGE_TILE_CACHE_H_

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

#include <tf/transform_datatypes.h>

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
    void Precache(const tf::Point& position);
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
    tf::Point                 m_currentPosition;
    bool                      m_exit;
    long                      m_memorySize;

    std::vector<std::queue<Tile*> > m_precacheRequests;
    std::stack<Tile*>               m_renderRequests;
    std::map<long, Tile*>           m_textureLoaded;
    std::map<long, Tile*>           m_renderRequestSet;
    std::map<long, Tile*>           m_precacheRequestSet;

    void PrecacheLayer(int layer, const tf::Point& position, int size);
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

#endif  // MULTIRES_IMAGE_TILE_CACHE_H_
