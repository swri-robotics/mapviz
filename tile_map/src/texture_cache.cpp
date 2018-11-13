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

#include <tile_map/texture_cache.h>

#include <cmath>

#include <boost/make_shared.hpp>

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <ros/ros.h>

#include <QGLWidget>
#include <QImage>

#include <swri_math_util/math_util.h>

namespace tile_map
{
  Texture::Texture(int32_t texture_id, size_t hash) :
    id(texture_id),
    url_hash(hash)
  {
  }

  Texture::~Texture()
  {
    //ROS_ERROR("==== DELETING TEXTURE: %d ====", id);
    // The texture will automatically be freed from the GPU memory when it goes
    // out of scope.  This is effectively when it is no longer in the texture
    // cache or being referenced for a render.
    GLuint ids[1];
    ids[0] = id;
    glDeleteTextures(1, &ids[0]);
  }

  TextureCache::TextureCache(ImageCachePtr image_cache, size_t size) :
    cache_(size),
    image_cache_(image_cache)
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
        boost::shared_ptr<QImage> image_ptr = image->GetImage();
        if (image_ptr)
        {
          // All of the OpenGL calls need to occur on the main thread and so
          // can't be done in the background.  The QImage calls could
          // potentially be done in a background thread by the image cache.
          QImage qimage = *image_ptr;

          GLuint ids[1];
          uint32_t check = 9999999;
          ids[0] = check;

          glGenTextures(1, &ids[0]);

          if (check == ids[0])
          {
            ROS_ERROR("FAILED TO CREATE TEXTURE");

            GLenum err = glGetError();
            const GLubyte *errString = gluErrorString(err);
            ROS_ERROR("GL ERROR(%u): %s", err, errString);
            return texture;
          }

          texture_ptr = new TexturePtr(boost::make_shared<Texture>(ids[0], url_hash));
          texture = *texture_ptr;

          float max_dim = std::max(qimage.width(), qimage.height());
          int32_t dimension = swri_math_util::Round(
            std::pow(2, std::ceil(std::log(max_dim) / std::log(2.0f))));

          if (qimage.width() != dimension || qimage.height() != dimension)
          {
            qimage = qimage.scaled(dimension, dimension, Qt::IgnoreAspectRatio, Qt::FastTransformation);
          }

          glBindTexture(GL_TEXTURE_2D, texture->id);
          glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGBA,
            dimension,
            dimension,
            0,
            GL_RGBA,
            GL_UNSIGNED_BYTE,
            QGLWidget::convertToGLFormat(qimage).bits());

          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

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

  void TextureCache::Clear()
  {
    image_cache_->Clear();
    cache_.clear();
  }
}
