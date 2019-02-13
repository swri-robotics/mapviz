// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************

#include <tile_map/bing_source.h>
#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>

#include <QRegExp>
#include <QString>

#include <json/json.h>

namespace tile_map
{
  const QString BingSource::BING_TYPE = "bing";
  const std::string BingSource::BING_IMAGE_URL_KEY = "imageUrl";
  const std::string BingSource::BING_IMAGE_URL_SUBDOMAIN_KEY = "imageUrlSubdomains";
  const std::string BingSource::BING_RESOURCE_SET_KEY = "resourceSets";
  const std::string BingSource::BING_RESOURCE_KEY = "resources";
  const std::string BingSource::BING_STATUS_CODE_KEY = "statusCode";

  BingSource::BingSource(const QString& name) :
    TileSource(),
    network_manager_(this)
  {
    name_ = name;
    is_custom_ = false;
    max_zoom_ = 19;
    base_url_ = "https://dev.virtualearth.net/REST/v1/Imagery/Metadata/Aerial?uriScheme=https&include=ImageryProviders&key={api_key}";
    tile_url_ = "";
    min_zoom_ = 2;

    QObject::connect(&network_manager_, SIGNAL(finished(QNetworkReply*)),
                     this, SLOT(ReplyFinished(QNetworkReply*)));
  }

  size_t BingSource::GenerateTileHash(int32_t level, int64_t x, int64_t y)
  {
    size_t hash = hash_((base_url_ + api_key_ + GenerateQuadKey(level, x, y)).toStdString());
    return hash;
  }

  QString BingSource::GenerateTileUrl(int32_t level, int64_t x, int64_t y)
  {
    QString url = tile_url_;
    if (!subdomains_.empty())
    {
      boost::random::uniform_int_distribution<> random(0, (int) subdomains_.size() - 1);
      url.replace(QString::fromStdString("{subdomain}"), subdomains_[random(rng_)]);
    }
    url.replace(QString::fromStdString("{quadkey}"), GenerateQuadKey(level, x, y));
    return url;
  }

  QString BingSource::GetType() const
  {
    return BING_TYPE;
  }

  QString BingSource::GetApiKey() const
  {
    return api_key_;
  }

  void BingSource::SetApiKey(const QString& api_key)
  {
    api_key_ = api_key.trimmed();
    if (!api_key_.isEmpty())
    {
      QString url(base_url_);
      url.replace(QString::fromStdString("{api_key}"), api_key_);
      // Changing the API key will result in the tile URL changing; go ahead
      // and blank it out so we don't make requests using the old one.
      tile_url_= "";
      subdomains_.clear();
      network_manager_.get(QNetworkRequest(QUrl(url)));
    }
  }

  QString BingSource::GenerateQuadKey(int32_t level, int64_t x, int64_t y) const
  {
    QString quadkey;
    for (int32_t i = level; i > 0; i--)
    {
      int32_t bitmask = 1 << (i-1);
      int32_t digit = 0;
      if ((x & bitmask) != 0)
      {
        digit |= 1;
      }
      if ((y & bitmask) != 0)
      {
        digit |= 2;
      }
      quadkey.append(QString::number(digit));
    }

    return quadkey;
  }

  void BingSource::ReplyFinished(QNetworkReply* reply)
  {
    QString reply_string(reply->readAll());
    Json::Reader reader;
    Json::Value root;
    reader.parse(reply_string.toStdString(), root);

    int status = root[BING_STATUS_CODE_KEY].asInt();
    if (status != 200)
    {
      Q_EMIT ErrorMessage("Bing authorization error: " + boost::lexical_cast<std::string>(status));
    }
    else
    {
      if (!root[BING_RESOURCE_SET_KEY].isArray() ||
          root[BING_RESOURCE_SET_KEY].size() == 0)
      {
        Q_EMIT ErrorMessage("No Bing resource sets found.");
        return;
      }
      Json::Value firstResourceSet = root[BING_RESOURCE_SET_KEY][0];

      if (!firstResourceSet[BING_RESOURCE_KEY].isArray() ||
          firstResourceSet[BING_RESOURCE_KEY].size() == 0)
      {
        Q_EMIT ErrorMessage("No Bing resources found.");
        return;
      }

      Json::Value first_resource = firstResourceSet[BING_RESOURCE_KEY][0];

      std::string image_url = first_resource[BING_IMAGE_URL_KEY].asString();

      if (image_url.empty())
      {
        Q_EMIT ErrorMessage("No Bing image URL found.");
        return;
      }

      tile_url_ = QString::fromStdString(image_url);
      SetMaxZoom(19);


      if (!first_resource[BING_IMAGE_URL_SUBDOMAIN_KEY].isArray() ||
          first_resource[BING_IMAGE_URL_SUBDOMAIN_KEY].size() == 0)
      {
        Q_EMIT ErrorMessage("No image URL subdomains; maybe that's ok sometimes?");
      }

      for (int i = 0; i < first_resource[BING_IMAGE_URL_SUBDOMAIN_KEY].size(); i++)
      {
        Json::Value subdomain = first_resource[BING_IMAGE_URL_SUBDOMAIN_KEY][i];
        subdomains_.push_back(QString::fromStdString(subdomain.asString()));
      }

      Q_EMIT InfoMessage("API Key Set.");

      is_ready_ = true;
    }
  }
}
