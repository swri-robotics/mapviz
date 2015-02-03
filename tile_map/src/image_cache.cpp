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

#include <ros/ros.h>

namespace tile_map
{
  bool ComparePriority(const ImagePtr left, const ImagePtr right)
  {
    return left->Priority() > right->Priority();
  }

  Image::Image(const std::string& uri, size_t uri_hash, uint64_t priority) :
    uri_(uri),
    uri_hash_(uri_hash),
    loading_(false),
    priority_(priority),
    failures_(0)
  {
  }
  
  Image::~Image()
  {
  }

  void Image::InitializeImage()
  {
    image_ = boost::make_shared<QImage>();
  }
  
  void Image::ClearImage()
  {
    image_.reset();
  }

  ImageCache::ImageCache(const QString& cache_dir, size_t size) :
    network_manager_(this),
    cache_dir_(cache_dir),
    cache_(size),
    exit_(false),
    pending_(0),
    tick_(0),
    cache_thread_(new CacheThread(this))
  {
    QNetworkDiskCache* disk_cache = new QNetworkDiskCache(this);
    disk_cache->setCacheDirectory(cache_dir_);
    network_manager_.setCache(disk_cache);
  
    connect(&network_manager_, SIGNAL(finished(QNetworkReply*)), this, SLOT(ProcessReply(QNetworkReply*)));
    connect(cache_thread_, SIGNAL(RequestImage(QString)), this, SLOT(ProcessRequest(QString)));

    cache_thread_->start();
    cache_thread_->setPriority(QThread::NormalPriority);
  }
  
  ImageCache::~ImageCache()
  {
    exit_ = true;
    cache_thread_->wait();
    delete cache_thread_;
  }

  ImagePtr ImageCache::GetImage(size_t uri_hash, const std::string& uri, int32_t priority)
  {
    ImagePtr image;
    
    // Retrieve the image reference from the cache, updating the freshness.
    cache_mutex_.lock();
    ImagePtr* image_ptr = cache_.take(uri_hash);
    if (!image_ptr)
    {
      // If the image is not in the cache, create a new reference.
      image_ptr = new ImagePtr(boost::make_shared<Image>(uri, uri_hash, priority));
      image = *image_ptr;
      if (!cache_.insert(uri_hash, image_ptr))
      {
        ROS_ERROR("FAILED TO CREATE HANDLE: %s", uri.c_str());
        image_ptr = 0;
      }
    }
    else
    {
      image = *image_ptr;
      
      // Add raw pointer back to cache.
      cache_.insert(uri_hash, image_ptr);
    }
    
    cache_mutex_.unlock();
    
    unprocessed_mutex_.lock();
    if (image && !image->GetImage())
    {
      if (image->Failures() < 3)
      {
        if (!unprocessed_.contains(uri_hash))
        {
          image->SetPriority(tick_++);
          unprocessed_[uri_hash] = image;
        }
      }
      else
      {
        ROS_ERROR("To many failures for image: %s", uri.c_str());
      }
    }

    unprocessed_mutex_.unlock();
    
    return image;
  }
  
  void ImageCache::ProcessRequest(QString uri)
  {
    QNetworkRequest request;
    request.setUrl(QUrl(uri));
    request.setRawHeader("User-Agent", "mapviz-1.0");
    request.setAttribute(
    QNetworkRequest::CacheLoadControlAttribute,
    QNetworkRequest::PreferCache);
        
    QNetworkReply *reply = network_manager_.get(request);
    connect(reply, SIGNAL(error(QNetworkReply::NetworkError)), this, SLOT(NetworkError(QNetworkReply::NetworkError)));
  } 
  
  void ImageCache::ProcessReply(QNetworkReply* reply)
  {
    std::string url = reply->url().toString().toStdString();
    size_t hash = hash_function_(url);
    
    ImagePtr image;
    unprocessed_mutex_.lock();
    
    image = unprocessed_[hash];
    if (image)
    {
      if (reply->error() == QNetworkReply::NoError)
      {
        QByteArray data = reply->readAll();
        image->InitializeImage();
        if (!image->GetImage()->loadFromData(data))
        {
          ROS_ERROR("FAILED TO CREATE IMAGE FROM REPLY: %s", url.c_str());
          image->ClearImage();
          image->AddFailure();
        }
      }
      else
      {
        ROS_ERROR("============ AN ERROR OCCURRED ==============: %s", url.c_str());
        image->AddFailure();
      }
    }
    
    unprocessed_.remove(hash);
    if (image)
    {
      image->SetLoading(false);
    }
    
    pending_--;

    unprocessed_mutex_.unlock();
    
    reply->deleteLater();
    
    

    // Condition variable?
  }
  
  void ImageCache::NetworkError(QNetworkReply::NetworkError error)
  {
    ROS_ERROR("NETWORK ERROR");
    // TODO add failure
  }
  
  void CacheThread::run()
  {    
    while (!p->exit_)
    {
      // TODO: Condition variable to wait for pending < 6 ?
      
      p->unprocessed_mutex_.lock();
      
      QList<ImagePtr> images = p->unprocessed_.values();
      
      p->unprocessed_mutex_.unlock();
      
      qSort(images.begin(), images.end(), ComparePriority);
    
      int32_t count = 0;
      while (p->pending_ < 6 && !images.empty() && count < 12)
      {
        ImagePtr image = images.front();
        if (!image->Loading())
        {
          image->SetLoading(true);
          images.pop_front();
        
          Q_EMIT RequestImage(QString::fromStdString(image->Uri()));
        
          p->unprocessed_mutex_.lock();
          p->pending_++;
          p->unprocessed_mutex_.unlock();

          count++;
        }
        else
        {
          images.pop_front();
        }
      }
      
      p->unprocessed_mutex_.lock();
      // Remove the oldest images from the unprocessed list.
      while (images.size() > 100)
      {
        ImagePtr image = images.back();
        images.pop_back();
        p->unprocessed_.remove(image->UriHash());
      }
      p->unprocessed_mutex_.unlock();
    
      usleep(1000);
    }
  }
}
