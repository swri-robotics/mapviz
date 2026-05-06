// *****************************************************************************
//
// Copyright (c) 2014-2025, Southwest Research Institute® (SwRI®)
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

#include <tile_map/texture_cache.hpp>

#include <cmath>

#include <rclcpp/logging.hpp>

#include <QImage>

#include <swri_math_util/math_util.h>

namespace tile_map
{
  Texture::Texture(std::unique_ptr<QOpenGLTexture> texture, size_t hash) :
    texture_(std::move(texture)),
    url_hash(hash)
  {
  }

  Texture::~Texture() = default;

  TextureCache::TextureCache(ImageCachePtr image_cache,
      size_t size,
      rclcpp::Logger logger) :
    cache_(size),
    image_cache_(image_cache),
    logger_(logger)
  {

  }

  TexturePtr TextureCache::GetTexture(size_t url_hash, const QString& url, bool& failed, int priority)
  {
    TexturePtr texture;

    failed = false;

    TexturePtr* texture_ptr = cache_.take(url_hash);
    if (texture_ptr)
    {
      texture = *texture_ptr;
      delete texture_ptr;
    }

    if (!texture)
    {
      ImagePtr image = image_cache_->GetImage(url_hash, url, priority);

      if (image)
      {
        failed = image->Failed();
        std::shared_ptr<QImage> image_ptr = image->GetImage();
        if (image_ptr)
        {
          // All of the OpenGL calls need to occur on the main thread and so
          // can't be done in the background.  The QImage calls could
          // potentially be done in a background thread by the image cache.
          QImage qimage = *image_ptr;

          float max_dim = std::max(qimage.width(), qimage.height());
          int32_t dimension = swri_math_util::Round(
            std::pow(2, std::ceil(std::log(max_dim) / std::log(2.0f))));

          if (qimage.width() != dimension || qimage.height() != dimension)
          {
            qimage = qimage.scaled(dimension, dimension, Qt::IgnoreAspectRatio, Qt::FastTransformation);
          }

          const QImage gl_image = qimage.convertToFormat(QImage::Format_RGBA8888).mirrored();

          auto open_gl_texture = std::make_unique<QOpenGLTexture>(QOpenGLTexture::Target2D);
          open_gl_texture->setFormat(QOpenGLTexture::RGBA8_UNorm);
          open_gl_texture->setSize(dimension, dimension);
          open_gl_texture->allocateStorage(QOpenGLTexture::RGBA, QOpenGLTexture::UInt8);
          open_gl_texture->setData(QOpenGLTexture::RGBA, QOpenGLTexture::UInt8, gl_image.constBits());
          open_gl_texture->setMinificationFilter(QOpenGLTexture::Linear);
          open_gl_texture->setMagnificationFilter(QOpenGLTexture::Linear);
          open_gl_texture->setWrapMode(QOpenGLTexture::ClampToEdge);

          if (!open_gl_texture->isCreated())
          {
            RCLCPP_ERROR(logger_, "Failed to create tile texture for %s", url.toStdString().c_str());
            return texture;
          }

          texture_ptr = new TexturePtr(std::make_shared<Texture>(std::move(open_gl_texture), url_hash));
          texture = *texture_ptr;

          cache_.insert(url_hash, texture_ptr);
        }
      }
    }

    return texture;
  }

  void TextureCache::AddTexture(const TexturePtr& texture)
  {
    if (texture)
    {
      TexturePtr* texture_ptr = new TexturePtr(texture);
      cache_.insert(texture->url_hash, texture_ptr);
    }
  }

  void TextureCache::SetLogger(rclcpp::Logger logger)
  {
    logger_ = logger;
    image_cache_->SetLogger(logger_);
  }

  void TextureCache::Clear()
  {
    image_cache_->Clear();
    cache_.clear();
  }
}
