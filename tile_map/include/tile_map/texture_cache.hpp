// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef TILE_MAP__TEXTURE_CACHE_HPP_
#define TILE_MAP__TEXTURE_CACHE_HPP_

#include <QCache>
#include <QImage>
#include <QOpenGLTexture>
#include <QString>

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tile_map/image_cache.hpp>

namespace tile_map
{
class Texture
{
public:
  Texture(std::unique_ptr<QOpenGLTexture> texture, size_t hash);
  ~Texture();

  const std::unique_ptr<QOpenGLTexture> & GetTexture() const {return texture_;}
  const size_t url_hash;

private:
  std::unique_ptr<QOpenGLTexture> texture_;
};
typedef std::shared_ptr<Texture> TexturePtr;

class TextureCache
{
public:
  explicit TextureCache(
    ImageCachePtr image_cache,
    size_t size = 512,
    rclcpp::Logger logger = rclcpp::get_logger("tile_map::TextureCache"));

  TexturePtr GetTexture(size_t url_hash, const QString & url, bool & failed, int priority);
  void AddTexture(const TexturePtr & texture);

  /// Turns a downloaded tile image into the image that is uploaded as its
  /// texture, e.g. elevation data into shading.  Gets the tile's URL too.
  using ImageFilter = std::function<QImage(const QImage & image, const QString & url)>;

  /// Filters every image before upload; an empty filter uploads them as they
  /// are.  Textures already made are not refiltered; see ClearTextures().
  void SetImageFilter(ImageFilter filter);

  /// Drops the textures but keeps the downloaded images, so the next request
  /// for each tile rebuilds its texture (with the current image filter)
  /// without downloading it again.
  void ClearTextures();

  void IncrementFrame();

  void SetLogger(rclcpp::Logger logger);

  void Clear();

  /// The image cache lives for as long as this texture cache does, so callers
  /// may connect to its signals once and keep the connection.
  ImageCachePtr GetImageCache() {return image_cache_;}

private:
  QCache<size_t, TexturePtr> cache_;

  ImageCachePtr image_cache_;

  ImageFilter image_filter_;

  rclcpp::Logger logger_;
};
typedef std::shared_ptr<TextureCache> TextureCachePtr;
}  // namespace tile_map

#endif  // TILE_MAP__TEXTURE_CACHE_HPP_
