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

#include <tile_map/image_cache.h>

#include <boost/make_shared.hpp>

#include <QtAlgorithms>
#include <QByteArray>
#include <QList>
#include <QNetworkAccessManager>
#include <QNetworkDiskCache>
#include <QUrl>

namespace tile_map
{
  bool ComparePriority(const ImagePtr left, const ImagePtr right)
  {
    return left->Priority() < right->Priority();
  }

  Image::Image(const QString& uri, int32_t priority) :
    uri_(uri),
    loading_(false),
    priority_(priority)
  {
  }
  
  Image::~Image()
  {
  }

  void Image::SetPriority(int32_t priority)
  {
    priority_ = priority;
  }

  ImageCache::ImageCache(const QString& cache_dir) :
    cache_dir_(cache_dir), 
    exit_(false),
    pending_(0),
    cache_thread_(this)
  {
    cache_thread_.setPriority(QThread::NormalPriority);
    cache_thread_.start();

  }
  
  ImageCache::~ImageCache()
  {
    exit_ = true;
    cache_thread_.wait();
  }

  ImagePtr ImageCache::GetImage(const QString& uri, int32_t priority)
  {
    ImagePtr image;
    
    // Retrieve the image reference from the cache, updating the freshness.
    cache_mutex_.lock();
    ImagePtr* image_ptr = cache_.object(uri);
    if (!image_ptr)
    {
      // If the image is not in the cache, create a new reference.
      image_ptr = new ImagePtr(boost::make_shared<Image>(uri, priority));
      if (!cache_.insert(uri, image_ptr))
      {
        image_ptr = 0;
      }
    }
    
    if (image_ptr)
    {
      image = *image_ptr;
    }
    
    cache_mutex_.unlock();
    
    unprocessed_mutex_.lock();
    if (image && !image->GetImage() && !image->Loading())
    {
      image->SetPriority(priority);
      unprocessed_[uri] = image;
    }
    unprocessed_mutex_.unlock();
    
    return image;
  }
  
  void ImageCache::ProcessReply(QNetworkReply* reply)
  {
    QString url = reply->url().toString();
    
    ImagePtr image;
    unprocessed_mutex_.lock();
    
    image = unprocessed_.take(url);
    if (image)
    {
      image->SetLoading(false);
    }
    
    if (image && reply->error() == QNetworkReply::NoError)
    {
      QByteArray data = reply->readAll();
      image->GetImage().reset(new QImage());
      if (!image->GetImage()->loadFromData(data))
      {
        image->GetImage().reset();
      }
    }
    
    unprocessed_mutex_.unlock();
    
    reply->deleteLater();
    
    pending_--;
    
    // Condition variable?
  }
  
  void ImageCache::CacheThread::run()
  {
    QNetworkAccessManager network_manager(p);
    QNetworkDiskCache* disk_cache = new QNetworkDiskCache(p);
    disk_cache->setCacheDirectory(p->cache_dir_);
    network_manager.setCache(disk_cache);
    
    connect(&network_manager, SIGNAL(finished(QNetworkReply*)), p, SLOT(ProcessReply(QNetworkReply*)));
    
    while (!p->exit_)
    {
      QList<ImagePtr> images;
      
      // TODO(malban): Condition variable to wait for pending < 6 ?
      
      p->unprocessed_mutex_.lock();
      
      QMap<QString,ImagePtr>::iterator iter;
      for (iter = p->unprocessed_.begin(); iter != p->unprocessed_.end(); ++iter)
      {
        p->cache_mutex_.lock();
        if (!p->cache_.contains(iter.key()))
        {
          iter = p->unprocessed_.erase(iter);
        }
        p->cache_mutex_.unlock();
      }
      
      images = p->unprocessed_.values();
      
      qSort(images.begin(), images.end(), ComparePriority);
    
      while (p->pending_ < 6 && !images.empty())
      {
        ImagePtr image = images.front();
        image->SetLoading(true);
        images.pop_front();
        
        QNetworkRequest request;
        request.setUrl(QUrl(image->Uri()));
        request.setAttribute(
          QNetworkRequest::CacheLoadControlAttribute,
          QNetworkRequest::PreferCache);
        
        network_manager.get(request);
        
        p->pending_++;
      }
      
      p->unprocessed_mutex_.unlock();
    
      usleep(10);
    }
  }
}
