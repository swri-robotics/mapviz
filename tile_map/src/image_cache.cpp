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

  const int Image::MAXIMUM_FAILURES = 5;

  Image::Image(const QString& uri, size_t uri_hash, uint64_t priority) :
    uri_(uri),
    uri_hash_(uri_hash),
    loading_(false),
    failures_(0),
    failed_(false),
    priority_(priority)
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

  void Image::AddFailure()
  {
    failures_++;
    failed_ = failures_ > MAXIMUM_FAILURES;
  }

  const int ImageCache::MAXIMUM_NETWORK_REQUESTS = 6;

  ImageCache::ImageCache(const QString& cache_dir, size_t size) :
    network_manager_(this),
    cache_dir_(cache_dir),
    cache_(size),
    exit_(false),
    tick_(0),
    cache_thread_(new CacheThread(this)),
    network_request_semaphore_(MAXIMUM_NETWORK_REQUESTS)
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
    // After setting our exit flag to true, release any conditions the cache thread
    // might be waiting on so that it will exit.
    exit_ = true;
    cache_thread_->notify();
    network_request_semaphore_.release(MAXIMUM_NETWORK_REQUESTS);
    cache_thread_->wait();
    delete cache_thread_;
  }

  void ImageCache::Clear()
  {
    cache_.clear();
    network_manager_.cache()->clear();
  }

  ImagePtr ImageCache::GetImage(size_t uri_hash, const QString& uri, int32_t priority)
  {
    ImagePtr image;

    // Retrieve the image reference from the cache, updating the freshness.
    cache_mutex_.lock();

    if (failed_.contains(uri_hash))
    {
      cache_mutex_.unlock();
      return image;
    }

    ImagePtr* image_ptr = cache_.take(uri_hash);
    if (!image_ptr)
    {
      // If the image is not in the cache, create a new reference.
      image_ptr = new ImagePtr(boost::make_shared<Image>(uri, uri_hash));
      image = *image_ptr;
      if (!cache_.insert(uri_hash, image_ptr))
      {
        ROS_ERROR("FAILED TO CREATE HANDLE: %s", uri.toStdString().c_str());
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
      if (!image->Failed())
      {
        if (!unprocessed_.contains(uri_hash))
        {
          // Set an image's starting priority so that it's higher than the
          // starting priority of every other image we've requested so
          // far; that ensures that, all other things being equal, the
          // most recently requested images will be loaded first.
          image->SetPriority(priority + tick_++);
          unprocessed_[uri_hash] = image;
          uri_to_hash_map_[uri] = uri_hash;
          cache_thread_->notify();
        }
        else
        {
          // Every time an image is requested but hasn't been loaded yet,
          // increase its priority.  Tiles within the visible area will
          // be requested more frequently, so this will make them load faster
          // than tiles the user can't see.
          image->SetPriority(priority + tick_++);
        }
      }
      else
      {
        failed_.insert(uri_hash);
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
    request.setAttribute(
        QNetworkRequest::HttpPipeliningAllowedAttribute,
        true);

    QNetworkReply *reply = network_manager_.get(request);
    connect(reply, SIGNAL(error(QNetworkReply::NetworkError)),
            this, SLOT(NetworkError(QNetworkReply::NetworkError)));
  }

  void ImageCache::ProcessReply(QNetworkReply* reply)
  {
    QString url = reply->url().toString();

    ImagePtr image;
    unprocessed_mutex_.lock();

    size_t hash = uri_to_hash_map_[url];
    image = unprocessed_[hash];
    if (image)
    {
      if (reply->error() == QNetworkReply::NoError)
      {
        QByteArray data = reply->readAll();
        image->InitializeImage();
        if (!image->GetImage()->loadFromData(data))
        {
          image->ClearImage();
          image->AddFailure();
        }
      }
      else
      {
        image->AddFailure();
      }
    }

    unprocessed_.remove(hash);
    uri_to_hash_map_.remove(url);
    if (image)
    {
      image->SetLoading(false);
    }
    network_request_semaphore_.release();

    unprocessed_mutex_.unlock();

    reply->deleteLater();
  }

  void ImageCache::NetworkError(QNetworkReply::NetworkError error)
  {
    ROS_ERROR("NETWORK ERROR: %d", error);
    // TODO add failure
  }

  const int CacheThread::MAXIMUM_SEQUENTIAL_REQUESTS = 12;

  CacheThread::CacheThread(ImageCache* parent) :
    image_cache_(parent),
    waiting_mutex_()
  {
    waiting_mutex_.lock();
  }

  void CacheThread::notify()
  {
    waiting_mutex_.unlock();
  }

  void CacheThread::run()
  {
    while (!image_cache_->exit_)
    {
      // Wait until we're told there are images we need to request.
      waiting_mutex_.lock();

      // Next, get all of them and sort them by priority.
      image_cache_->unprocessed_mutex_.lock();
      QList<ImagePtr> images = image_cache_->unprocessed_.values();
      image_cache_->unprocessed_mutex_.unlock();

      qSort(images.begin(), images.end(), ComparePriority);

      // Go through all of them and request them.  Qt's network manager will
      // only handle six simultaneous requests at once, so we use a semaphore
      // to limit ourselves to that many.
      // Each individual image will release the semaphore when it is done loading.
      // Also, only load up to a certain number at a time in this loop.  If there
      // are more left afterward, we'll start over.  This ensures that we
      // concentrate on processing the highest-priority images.
      int count = 0;
      while (!image_cache_->exit_ && !images.empty() && count < MAXIMUM_SEQUENTIAL_REQUESTS)
      {
        image_cache_->network_request_semaphore_.acquire();

        ImagePtr image = images.front();
        image_cache_->unprocessed_mutex_.lock();
        if (!image->Loading() && !image->Failed())
        {
          count++;
          image->SetLoading(true);
          images.pop_front();

          QString uri = image->Uri();
          size_t hash = image_cache_->uri_to_hash_map_[uri];
          if (uri.startsWith(QString("file:///")))
          {
            image->InitializeImage();
            QString filepath = uri.replace(QString("file:///"), QString("/"));
            if (!image->GetImage()->load(filepath))
            {
              image->ClearImage();
              image->AddFailure();
            }

            image_cache_->unprocessed_.remove(hash);
            image_cache_->uri_to_hash_map_.remove(uri);
            image->SetLoading(false);
            image_cache_->network_request_semaphore_.release();
          }
          else
          {
            Q_EMIT RequestImage(image->Uri());
          }
        }
        else
        {
          images.pop_front();
        }
        image_cache_->unprocessed_mutex_.unlock();

      }
      if (!images.empty())
      {
        waiting_mutex_.unlock();
      }
    }
  }
}
