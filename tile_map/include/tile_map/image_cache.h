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

#ifndef TILE_MAP_IMAGE_CACHE_H_
#define TILE_MAP_IMAGE_CACHE_H_

#include <boost/atomic.hpp>
#include <boost/shared_ptr.hpp>

#include <QCache>
#include <QImage>
#include <QMap>
#include <QMutex>
#include <QNetworkReply>
#include <QObject>
#include <QThread>

namespace tile_map
{
  class Image
  {
  public:
    Image(const QString& uri, int32_t priority);
    ~Image();
  
    int32_t Priority() const { return priority_; }
    void SetPriority(int32_t priority);
  
    bool Loading() const { return loading_; }
    void SetLoading(bool loading) { loading_ = loading; }
    
    QString Uri() const { return uri_; }
  
    boost::shared_ptr<const QImage> GetImage() const { return image_; }
    boost::shared_ptr<QImage> GetImage() { return image_; }

  private:
    QString uri_;
    
    boost::atomic<bool> loading_;
    boost::atomic<int32_t> priority_;
    
    mutable boost::shared_ptr<QImage> image_;
  };
  typedef boost::shared_ptr<Image> ImagePtr;

  class ImageCache : public QObject
  {
    Q_OBJECT
    
  public:
    ImageCache(const QString& cache_dir);
    ~ImageCache();
    
    ImagePtr GetImage(const QString& uri, int32_t priority = 0);
  
  public Q_SLOTS:
    void ProcessReply(QNetworkReply* reply);
  
  private:
    QString cache_dir_;
  
    QCache<QString, ImagePtr> cache_;
    QMap<QString, ImagePtr> unprocessed_;
    
    QMutex cache_mutex_;
    QMutex unprocessed_mutex_;
    boost::atomic<bool> exit_;
    
    boost::atomic<int32_t> pending_;
    
    class CacheThread : public QThread
    {
    public:
      CacheThread(ImageCache* parent) : p(parent) {}
      
      virtual void run();

    private:
      ImageCache* p;
 
    };
    friend class CacheThread;
    
    CacheThread cache_thread_;
  };
  typedef boost::shared_ptr<ImageCache> ImageCachePtr;
}

#endif  // TILE_MAP_IMAGE_CACHE_H_
