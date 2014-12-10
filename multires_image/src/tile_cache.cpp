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

#include <multires_image/tile_cache.h>

// C++ standard libraries
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <exception>

// QT libraries
#include <QApplication>
#include <QEvent>

#include <multires_image/tile_set_layer.h>

namespace multires_image
{
  TileCache::TileCache(TileSet* tileSet, QGLWidget* widget) :
    m_tileSet(tileSet),
    m_widget(widget),
    m_currentLayer(0),
    m_currentPosition(0, 0, 0),
    m_exit(false),
    m_memorySize(0),
    m_cacheThread(this),
    m_freeThread(this),
    m_renderRequestsLock(QMutex::Recursive),
    m_renderRequestSetLock(QMutex::Recursive),
    m_precacheRequestsLock(QMutex::Recursive),
    m_precacheRequestSetLock(QMutex::Recursive),
    m_textureLoadedLock(QMutex::Recursive)
  {
    connect(this, SIGNAL(SignalLoadTexture(Tile*)),
      SLOT(LoadTextureSlot(Tile*)), Qt::BlockingQueuedConnection);

    connect(this, SIGNAL(SignalDeleteTexture(Tile*)),
      SLOT(DeleteTextureSlot(Tile*)), Qt::BlockingQueuedConnection);

    m_cacheThread.setPriority(QThread::NormalPriority);
    m_cacheThread.start();

    m_freeThread.setPriority(QThread::LowPriority);
    m_freeThread.start();

    for (int i = 0; i < m_tileSet->LayerCount(); i++)
    {
      m_precacheRequests.push_back(std::queue<Tile*>());
    }
  }

  TileCache::~TileCache(void)
  {
    m_exit = true;
    m_cacheThread.wait();
    m_freeThread.wait();
  }

  void TileCache::LoadTextureSlot(Tile* tile)
  {
    tile->LoadTexture();
  }

  void TileCache::DeleteTextureSlot(Tile* tile)
  {
    tile->UnloadTexture();
  }

  void TileCache::Load(Tile* tile)
  {
    m_renderRequestsLock.lock();
    m_renderRequestSetLock.lock();

    try
    {
      if (m_renderRequestSet.count(tile->TileID()) == 0)
      {
        m_renderRequests.push(tile);
        m_renderRequestSet[tile->TileID()] = tile;
      }
    }
    catch(const std::exception& e)
    {
      std::cout << "An exception occurred queuing a tile to be cached: " << e.what() << std::endl;
    }

    m_renderRequestSetLock.unlock();
    m_renderRequestsLock.unlock();
  }

  void TileCache::Precache(double x, double y)
  {
    tf::Point point(x, y, 0);
    Precache(point);
  }

  void TileCache::Precache(const tf::Point& position)
  {
    m_currentPosition = position;

    int sizes[] = {3, 2, 2, 1, 1, 1};

    PrecacheLayer(m_currentLayer, m_currentPosition, sizes[0]);

    for (int i = 1; i <= 5; i++)
    {
      int layer = m_currentLayer + i;
      if (layer < m_tileSet->LayerCount())
      {
        PrecacheLayer(layer, m_currentPosition, sizes[i]);
      }

      layer = m_currentLayer - i;
      if (layer >= 0)
      {
        PrecacheLayer(layer, m_currentPosition, sizes[i]);
      }
    }
  }

  void TileCache::PrecacheLayer(int layerNum, const tf::Point& position, int size)
  {
    TileSetLayer* layer = m_tileSet->GetLayer(layerNum);

    int row, column;
    layer->GetTileIndex(position, row, column);

    int startRow = std::max(0, row - size);
    int endRow = std::min(layer->RowCount() - 1, row + size);
    int startColumn = std::max(0, column - size);
    int endColumn = std::min(layer->ColumnCount() - 1, column + size);

    for (int c = startColumn; c <= endColumn; c++)
    {
      for (int r = startRow; r <= endRow; r++)
      {
        Tile* tile = layer->GetTile(c, r);

        m_precacheRequestsLock.lock();
        m_precacheRequestSetLock.lock();

        try
        {
          if (m_precacheRequestSet.count(tile->TileID()) == 0)
          {
            m_precacheRequests[layerNum].push(tile);
            m_precacheRequestSet[tile->TileID()] = tile;
          }
        }
        catch (const std::exception& e)
        {
          std::cout << "An exception occurred queuing tiles for precaching: " << e.what() << std::endl;
        }

        m_precacheRequestSetLock.unlock();
        m_precacheRequestsLock.unlock();
      }
    }
  }

  void TileCache::Exit()
  {
    m_exit = true;
  }

  void TileCache::LoadTexture(Tile* tile)
  {
    Q_EMIT SignalLoadTexture(tile);

    m_memorySize += tile->MemorySize();
    Q_EMIT SignalMemorySize(m_memorySize);

    m_textureLoadedLock.lock();

    try
    {
      m_textureLoaded[tile->TileID()] = tile;
    }
    catch (const std::exception& e)
    {
      std::cout << "An exception occurred loading texture: " << e.what() << std::endl;
    }

    m_textureLoadedLock.unlock();

    if (tile->Layer() == m_currentLayer)
    {
      QApplication::postEvent(m_widget, new QEvent(QEvent::UpdateRequest));
    }
  }

  void TileCache::UnloadTexture(Tile* tile)
  {
    Q_EMIT SignalDeleteTexture(tile);

    m_memorySize -= tile->MemorySize();
    Q_EMIT SignalMemorySize(m_memorySize);

    m_textureLoadedLock.lock();

    try
    {
      m_textureLoaded.erase(tile->TileID());
    }
    catch (const std::exception& e)
    {
      std::cout << "An exception occurred unloading texture: " << e.what() << std::endl;
    }

    m_textureLoadedLock.unlock();
  }

  void TileCache::CacheThread::run()
  {
    while (!p->m_exit)
    {
      Tile* tile = NULL;
      p->m_renderRequestsLock.lock();

      if (p->m_renderRequests.size() > 0)
      {
        tile = p->m_renderRequests.top();
        p->m_renderRequests.pop();
      }

      p->m_renderRequestsLock.unlock();

      if (tile != NULL)
      {
        if (!tile->Failed())
        {
          if (tile->Layer() == p->m_currentLayer)
          {
            int row, column;
            p->m_tileSet->GetLayer(tile->Layer())->GetTileIndex(p->m_currentPosition, row, column);

            if (abs(tile->Row() - row) <= 3 || abs(tile->Column() - column) < 3)
            {
              if (!tile->TextureLoaded())
              {
                if (tile->LoadImageToMemory() == true)
                {
                  p->LoadTexture(tile);
                  tile->UnloadImage();
                }
                else
                {
                  printf("failed to load image\n");
                }
              }
            }
          }
          else
          {
            p->m_precacheRequests[tile->Layer()].push(tile);
          }

          p->m_renderRequestSetLock.lock();
          p->m_renderRequestSet.erase(tile->TileID());
          p->m_renderRequestSetLock.unlock();
        }
        else
        {
        }
      }
      else
      {
        p->m_precacheRequestsLock.lock();

        try
        {
          for (uint32_t i = 0; (i < p->m_precacheRequests.size()) && (tile == NULL); i++)
          {
            int32_t index = p->m_currentLayer + i;
            if ((index < (int64_t)p->m_precacheRequests.size()) &&
                (p->m_precacheRequests[index].size() > 0))
            {
              tile = p->m_precacheRequests[index].front();
              p->m_precacheRequests[index].pop();
            }
            else if (i != 0)
            {
              index = p->m_currentLayer - i;
              if (index >= 0 && p->m_precacheRequests[index].size() > 0)
              {
                tile = p->m_precacheRequests[index].front();
                p->m_precacheRequests[index].pop();
              }
            }
          }
        }
        catch (const std::exception& e)
        {
          std::cout << "An exception occurred precaching texture: " << e.what() << std::endl;
        }

        p->m_precacheRequestsLock.unlock();

        if (tile != NULL && !tile->Failed() && !tile->TextureLoaded())
        {
          int row, column;
          p->m_tileSet->GetLayer(tile->Layer())->GetTileIndex(p->m_currentPosition, row, column);
          if (abs(tile->Row() - row) <= 3 || abs(tile->Column() - column) <= 3)
          {
            if (tile->LoadImageToMemory() == true)
            {
              p->LoadTexture(tile);

              tile->UnloadImage();
            }
            else
            {
              printf("failed to precache load image\n");
            }
          }

          p->m_precacheRequestSetLock.lock();
          p->m_precacheRequestSet.erase(tile->TileID());
          p->m_precacheRequestSetLock.unlock();
        }
      }

      if (tile == NULL)
      {
        usleep(10);
      }
    }
  }

  void TileCache::FreeThread::run()
  {
    while (!p->m_exit)
    {
      std::map<int64_t, Tile*>* tiles;
      p->m_textureLoadedLock.lock();

      tiles = new std::map<int64_t, Tile*>(p->m_textureLoaded);

      p->m_textureLoadedLock.unlock();

      std::map<int64_t, Tile*>::iterator iter;

      for (iter = tiles->begin(); iter != tiles->end(); ++iter)
      {
        Tile* tile = iter->second;
        int row, column;
        p->m_tileSet->GetLayer(tile->Layer())->GetTileIndex(p->m_currentPosition, row, column);

        if (abs(tile->Row() - row) > 6 || abs(tile->Column() - column) > 6)
        {
          p->m_renderRequestSetLock.lock();
          p->m_renderRequestSet.erase(tile->TileID());
          p->m_renderRequestSetLock.unlock();

          p->m_precacheRequestSetLock.lock();
          p->m_precacheRequestSet.erase(tile->TileID());
          p->m_precacheRequestSetLock.unlock();

          p->UnloadTexture(tile);
        }
      }

      delete tiles;

      sleep(2);
    }
  }
}
