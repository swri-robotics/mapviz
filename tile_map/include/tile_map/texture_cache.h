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

#ifndef TILE_MAP_TEXTURE_CACHE_H_
#define TILE_MAP_TEXTURE_CACHE_H_

#include <QCache>

#include <tile_map/image_cache.h>

namespace tile_map
{
  class Texture
  {
  public:
    Texture(int32_t texture_id, size_t hash);
    ~Texture();

    const int32_t id;
    const size_t url_hash;

    bool failed;
  };
  typedef boost::shared_ptr<Texture> TexturePtr;

  class TextureCache
  {
  public:
    explicit TextureCache(ImageCachePtr image_cache, size_t size = 512);

    TexturePtr GetTexture(size_t url_hash, const QString& url, bool& failed, int priority);
    void AddTexture(const TexturePtr& texture);

    void Clear();

  private:
    QCache<size_t, TexturePtr> cache_;

    ImageCachePtr image_cache_;
  };
  typedef boost::shared_ptr<TextureCache> TextureCachePtr;
}

#endif  // TILE_MAP_TEXTURE_CACHE_H_
