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

#include <tile_map/image_cache.hpp>

#include <QtAlgorithms>
#include <QByteArray>
#include <QList>
#include <QNetworkAccessManager>
#include <QNetworkDiskCache>
#include <QUrl>

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

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

  void Image::InitializeImage()
  {
    image_ = std::make_shared<QImage>();
  }

  void Image::SetPendingData(const QByteArray& data)
  {
    pending_data_ = data;
    has_pending_data_ = true;
  }

  QByteArray Image::TakePendingData()
  {
    QByteArray result;
    std::swap(result, pending_data_);
    has_pending_data_ = false;
    return result;
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

  ImageCache::ImageCache(const QString& cache_dir,
      size_t size,
      rclcpp::Logger logger) :
    network_manager_(this),
    cache_dir_(cache_dir),
    cache_(size),
    exit_(false),
    frame_(0),
    tick_(0),
    cache_thread_(new CacheThread(this)),
    network_request_semaphore_(MAXIMUM_NETWORK_REQUESTS),
    logger_(logger)
  {
    QNetworkDiskCache* disk_cache = new QNetworkDiskCache(this);
    disk_cache->setCacheDirectory(cache_dir_);
    network_manager_.setCache(disk_cache);

    connect(&network_manager_, SIGNAL(finished(QNetworkReply*)), this, SLOT(ProcessReply(QNetworkReply*)));
    connect(cache_thread_, SIGNAL(RequestImage(QString)), this, SLOT(ProcessRequest(QString)));

    cache_thread_->start();
    cache_thread_->setPriority(QThread::LowPriority);
  }

  ImageCache::~ImageCache()
  {
    // Disconnect signals before touching the thread or network manager in
    // case of any cleanup race conditions
    disconnect(&network_manager_, SIGNAL(finished(QNetworkReply*)),
        this, SLOT(ProcessReply(QNetworkReply*)));
    disconnect(cache_thread_, SIGNAL(RequestImage(QString)),
        this, SLOT(ProcessRequest(QString)));

    // After setting our exit flag to true, release any conditions the cache thread
    // might be waiting on so that it will exit.
    exit_ = true;
    cache_thread_->notify();
    network_request_semaphore_.release(MAXIMUM_NETWORK_REQUESTS);
    cache_thread_->wait();
    delete cache_thread_;
  }

  void ImageCache::IncrementFrame()
  {
    // As the user scrolls through many viewports, thousands of tile requests can accumulate.
    // Remove any unprocessed tile requests that aren't visible in the
    // current viewport frame request (unless we've already received a server reply and are actively
    // decoding the image)
    unprocessed_mutex_.lock();
    for (auto it = unprocessed_.begin(); it != unprocessed_.end(); ) {
      if (!it.value()->Loading() && !it.value()->HasPendingData() &&
          it.value()->LastRequestedFrame() < frame_) {
        uri_to_hash_map_.remove(it.value()->Uri());
        it = unprocessed_.erase(it);
      } else {
        ++it;
      }
    }
    unprocessed_mutex_.unlock();
    frame_++;
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
      image_ptr = new ImagePtr(std::make_shared<Image>(uri, uri_hash));
      image = *image_ptr;
      if (!cache_.insert(uri_hash, image_ptr))
      {
        RCLCPP_ERROR(logger_, "FAILED TO CREATE HANDLE: %s", uri.toStdString().c_str());
        image_ptr = nullptr;
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
          image->SetLastRequestedFrame(frame_);
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
          image->SetLastRequestedFrame(frame_);
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

  void  ImageCache::SetLogger(rclcpp::Logger logger)
  {
    logger_ = logger;
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

    network_manager_.get(request);
  }

  void ImageCache::ProcessReply(QNetworkReply* reply)
  {
    QString url = reply->url().toString();

    // Store raw bytes under the mutex
    // CacheThread decodes instead of the higher priority main thread.
    unprocessed_mutex_.lock();

    size_t hash = uri_to_hash_map_.value(url, 0);
    ImagePtr image;
    if (uri_to_hash_map_.contains(url)) {
      image = unprocessed_.value(hash);
    }

    bool keep_in_queue = false;
    if (image)
    {
      if (reply->error() == QNetworkReply::NoError)
      {
        image->SetPendingData(reply->readAll());
        keep_in_queue = true;  // CacheThread will decode then remove
      }
      else
      {
        auto steady_clock = rclcpp::Clock();
        RCLCPP_ERROR_THROTTLE(logger_, steady_clock, 1000,
          "tile_map: network error for %s: %s",
          url.toStdString().c_str(), reply->errorString().toStdString().c_str());
        image->AddFailure();
      }
      image->SetLoading(false);
    }
    else
    {
      RCLCPP_WARN(logger_, "tile_map: reply for unknown URL (already evicted?): %s",
        url.toStdString().c_str());
    }

    if (!keep_in_queue && uri_to_hash_map_.contains(url)) {
      unprocessed_.remove(hash);
      uri_to_hash_map_.remove(url);
    }

    network_request_semaphore_.release();

    unprocessed_mutex_.unlock();

    cache_thread_->notify();

    reply->deleteLater();
  }

  const int CacheThread::MAXIMUM_SEQUENTIAL_REQUESTS = 12;

  CacheThread::CacheThread(ImageCache* parent) :
    image_cache_(parent),
    waiting_semaphore_(0)
  {
  }

  void CacheThread::notify()
  {
    // Using a semaphore here to safely call release from any thread. There is a race
    // here where multiple threads can see available as 0, but this just causes extra
    // wakeups and is effectively harmless
    if (waiting_semaphore_.available() == 0) {
      waiting_semaphore_.release();
    }
  }

  void CacheThread::run()
  {
    while (!image_cache_->exit_)
    {
      // Wait until we're told there are images we need to request.
      waiting_semaphore_.acquire();

      // Decode any tiles whose network bytes have arrived (set by ProcessReply).
      // This keeps JPEG/PNG decode off the main thread so it cannot block the
      // Qt event loop and delay render frames.
      while (!image_cache_->exit_)
      {
        image_cache_->unprocessed_mutex_.lock();
        ImagePtr to_decode;
        for (auto& img : image_cache_->unprocessed_) {
          if (img->HasPendingData()) { to_decode = img; break; }
        }
        QByteArray raw_data;
        if (to_decode) {
          raw_data = to_decode->TakePendingData();
        }
        image_cache_->unprocessed_mutex_.unlock();

        if (!to_decode) { break; }

        auto decoded = std::make_shared<QImage>();
        if (!decoded->loadFromData(raw_data))
        {
          decoded.reset();
          RCLCPP_WARN(image_cache_->logger_, "tile_map: failed to decode image from %s",
            to_decode->Uri().toStdString().c_str());
        }

        image_cache_->unprocessed_mutex_.lock();
        if (image_cache_->unprocessed_.value(to_decode->UriHash()) == to_decode)
        {
          if (decoded) {
            to_decode->SetDecodedImage(decoded);
          } else {
            to_decode->AddFailure();
          }
          image_cache_->unprocessed_.remove(to_decode->UriHash());
          image_cache_->uri_to_hash_map_.remove(to_decode->Uri());
        }
        image_cache_->unprocessed_mutex_.unlock();
      }

      // Next, get all of them and sort them by priority.
      // Sort under the mutex guard to prevent seg faults from occurring
      // during comparisons in other threads
      image_cache_->unprocessed_mutex_.lock();
      QList<ImagePtr> images = image_cache_->unprocessed_.values();
      std::sort(images.begin(), images.end(), ComparePriority);
      image_cache_->unprocessed_mutex_.unlock();

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
        // Request images only if they aren't currently pending, loading, or previously failed
        if (!image->Loading() && !image->Failed() && !image->HasPendingData() &&
            image_cache_->unprocessed_.value(image->UriHash()) == image)
        {
          count++;
          image->SetLoading(true);
          images.pop_front();

          QString uri = image->Uri();
          size_t hash = image_cache_->uri_to_hash_map_.value(uri);
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
            image_cache_->cache_thread_->notify();
          }
          else
          {
            Q_EMIT RequestImage(image->Uri());
          }
        }
        else
        {
          images.pop_front();
          image_cache_->network_request_semaphore_.release();
        }
        image_cache_->unprocessed_mutex_.unlock();
      }
    }
  }
}
