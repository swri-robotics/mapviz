// *****************************************************************************
//
// Copyright (c) 2014-2020, Southwest Research Institute® (SwRI®)
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

#ifndef TILE_MAP_IMAGE_CACHE_HPP_
#define TILE_MAP_IMAGE_CACHE_HPP_

#include <string>

#include <rclcpp/logger.hpp>

#include <QByteArray>
#include <QCache>
#include <QImage>
#include <QMap>
#include <QMutex>
#include <QNetworkReply>
#include <QObject>
#include <QSemaphore>
#include <QSet>
#include <QThread>
#include <set>

namespace tile_map
{
  class CacheThread;

  class Image
  {
  public:
    Image(const QString& uri, size_t uri_hash, uint64_t priority = 0);
    ~Image() = default;

    QString Uri() const { return uri_; }
    size_t UriHash() const { return uri_hash_; }

    std::shared_ptr<QImage> GetImage() { return image_; }

    void InitializeImage();
    void ClearImage();

    void AddFailure();
    bool Failed() const { return failed_; }

    void SetLastRequestedFrame(uint64_t frame) { last_requested_frame_ = frame; }
    uint64_t LastRequestedFrame() const { return last_requested_frame_; }

    void SetPriority(uint64_t priority) { priority_ = priority; }
    uint64_t Priority() const { return priority_; }

    bool Loading() const { return loading_; }
    void SetLoading(bool loading) { loading_ = loading; }

    void SetDecodedImage(std::shared_ptr<QImage> img) { image_ = img; }

    bool HasPendingData() const { return has_pending_data_; }
    void SetPendingData(const QByteArray& data);
    QByteArray TakePendingData();

  private:
    QString uri_;
    size_t uri_hash_;
    bool loading_;
    int32_t failures_;
    bool failed_;
    uint64_t last_requested_frame_ = 0;
    uint64_t priority_ = 0;

    mutable std::shared_ptr<QImage> image_;
    QByteArray pending_data_;
    bool has_pending_data_ = false;

    static const int MAXIMUM_FAILURES;
  };
  typedef std::shared_ptr<Image> ImagePtr;

  class ImageCache : public QObject
  {
    Q_OBJECT

  public:
    explicit ImageCache(const QString& cache_dir,
        size_t size = 4096,
        rclcpp::Logger logger = rclcpp::get_logger("tile_map::ImageCache"));
    ~ImageCache() override;

    ImagePtr GetImage(size_t uri_hash, const QString& uri, int32_t priority = 0);

    void IncrementFrame();

    void SetLogger(rclcpp::Logger logger);

  public Q_SLOTS:
    void ProcessRequest(QString uri);
    void ProcessReply(QNetworkReply* reply);
    void Clear();

  private:
    QNetworkAccessManager network_manager_;

    QString cache_dir_;

    QCache<size_t, ImagePtr> cache_;
    QMap<size_t, ImagePtr> unprocessed_;
    QSet<size_t> failed_;
    QMap<QString, size_t> uri_to_hash_map_;

    QMutex cache_mutex_;
    QMutex unprocessed_mutex_;
    bool exit_;

    uint64_t frame_;
    uint64_t tick_;

    CacheThread* cache_thread_;

    QSemaphore network_request_semaphore_;

    rclcpp::Logger logger_;

    friend class CacheThread;

    static const int MAXIMUM_NETWORK_REQUESTS;
  };

  class CacheThread : public QThread
  {
    Q_OBJECT
    public:
      explicit CacheThread(ImageCache* parent);

      void run() override;

      void notify();

    Q_SIGNALS:
      void RequestImage(QString);

    private:
      ImageCache* image_cache_;
      QSemaphore waiting_semaphore_;

      static const int MAXIMUM_SEQUENTIAL_REQUESTS;
  };


  typedef std::shared_ptr<ImageCache> ImageCachePtr;
}

#endif  // TILE_MAP_IMAGE_CACHE_HPP_
