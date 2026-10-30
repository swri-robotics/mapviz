// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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

#include <tile_map/bing_source.hpp>

#include <QNetworkRequest>
#include <QString>
#include <QStringList>
#include <QUrl>
#include <json/json.h>

#include <random>

namespace tile_map
{
const QString BingSource::BING_TYPE = "bing";
const std::string BingSource::BING_ATTRIBUTION_KEY = "attribution";  // NOLINT(runtime/string)
const std::string BingSource::BING_BBOX_KEY = "bbox";  // NOLINT(runtime/string)
const std::string BingSource::BING_BRAND_LOGO_URI_KEY = "brandLogoUri";  // NOLINT(runtime/string)
const std::string BingSource::BING_COVERAGE_AREAS_KEY = "coverageAreas";  // NOLINT(runtime/string)
const std::string BingSource::BING_IMAGE_URL_KEY = "imageUrl";  // NOLINT(runtime/string)
const std::string BingSource::BING_IMAGERY_PROVIDERS_KEY =  // NOLINT(runtime/string)
  "imageryProviders";
const std::string BingSource::BING_IMAGE_URL_SUBDOMAIN_KEY =  // NOLINT(runtime/string)
  "imageUrlSubdomains";
const std::string BingSource::BING_RESOURCE_SET_KEY = "resourceSets";  // NOLINT(runtime/string)
const std::string BingSource::BING_RESOURCE_KEY = "resources";  // NOLINT(runtime/string)
const std::string BingSource::BING_STATUS_CODE_KEY = "statusCode";  // NOLINT(runtime/string)
const std::string BingSource::BING_ZOOM_MAX_KEY = "zoomMax";  // NOLINT(runtime/string)
const std::string BingSource::BING_ZOOM_MIN_KEY = "zoomMin";  // NOLINT(runtime/string)

namespace
{
/// Marks the logo request, so ReplyFinished() can tell it from the metadata.
const char LOGO_REQUEST[] = "logo";
}  // namespace

BingSource::BingSource(const QString & name)
: TileSource(),
  network_manager_(this)
{
  name_ = name;
  is_ready_ = false;
  is_custom_ = false;
  max_zoom_ = 19;
  base_url_ =
    "https://dev.virtualearth.net/REST/v1/Imagery/Metadata/Aerial"
    "?uriScheme=https&include=ImageryProviders&key={api_key}";
  tile_url_ = "";
  min_zoom_ = 2;

  QObject::connect(
    &network_manager_, SIGNAL(finished(QNetworkReply*)),
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
  if (!subdomains_.empty()) {
    std::uniform_int_distribution<int> random(0, static_cast<int>(subdomains_.size()) - 1);
    url.replace(QString::fromStdString("{subdomain}"), subdomains_[random(rng_)]);
  }
  url.replace(QString::fromStdString("{quadkey}"), GenerateQuadKey(level, x, y));
  return url;
}

QString BingSource::GetType() const
{
  return BING_TYPE;
}

QString BingSource::GetAttribution(const ViewBounds & view) const
{
  QStringList credits;
  for (const ImageryProvider & provider : providers_) {
    for (const CoverageArea & area : provider.coverage_areas) {
      if (view.level >= area.zoom_min && view.level <= area.zoom_max &&
        view.south <= area.north && view.north >= area.south &&
        view.west <= area.east && view.east >= area.west)
      {
        if (!credits.contains(provider.attribution)) {
          credits.append(provider.attribution);
        }
        break;
      }
    }
  }
  return credits.join(", ");
}

QImage BingSource::GetAttributionLogo() const
{
  return logo_;
}

QString BingSource::GetApiKey() const
{
  return api_key_;
}

void BingSource::SetApiKey(const QString & api_key)
{
  api_key_ = api_key.trimmed();
  if (!api_key_.isEmpty()) {
    QString url(base_url_);
    url.replace(QString::fromStdString("{api_key}"), api_key_);
    // Changing the API key will result in the tile URL changing; go ahead
    // and blank it out so we don't make requests using the old one.
    tile_url_ = "";
    subdomains_.clear();
    providers_.clear();
    logo_ = QImage();
    network_manager_.get(QNetworkRequest(QUrl(url)));
  }
}

QString BingSource::GenerateQuadKey(int32_t level, int64_t x, int64_t y) const
{
  QString quadkey;
  for (int32_t i = level; i > 0; i--) {
    int32_t bitmask = 1 << (i - 1);
    int32_t digit = 0;
    if ((x & bitmask) != 0) {
      digit |= 1;
    }
    if ((y & bitmask) != 0) {
      digit |= 2;
    }
    quadkey.append(QString::number(digit));
  }

  return quadkey;
}

void BingSource::ReplyFinished(QNetworkReply * reply)
{
  reply->deleteLater();

  if (reply->request().attribute(QNetworkRequest::User).toString() == LOGO_REQUEST) {
    QImage logo;
    if (reply->error() == QNetworkReply::NoError && logo.loadFromData(reply->readAll())) {
      logo_ = logo;
    } else {
      Q_EMIT ErrorMessage(
        "Unable to download the Bing logo: " + reply->errorString().toStdString());
    }
    return;
  }

  HandleMetadata(reply->readAll());
}

void BingSource::HandleMetadata(const QByteArray & reply)
{
  Json::Reader reader;
  Json::Value root;
  reader.parse(reply.toStdString(), root);

  int status = root[BING_STATUS_CODE_KEY].asInt();
  if (status != 200) {
    Q_EMIT ErrorMessage("Bing authorization error: " + std::to_string(status));
  } else {
    if (!root[BING_RESOURCE_SET_KEY].isArray() ||
      root[BING_RESOURCE_SET_KEY].empty())
    {
      Q_EMIT ErrorMessage("No Bing resource sets found.");
      return;
    }
    Json::Value firstResourceSet = root[BING_RESOURCE_SET_KEY][0];

    if (!firstResourceSet[BING_RESOURCE_KEY].isArray() ||
      firstResourceSet[BING_RESOURCE_KEY].empty())
    {
      Q_EMIT ErrorMessage("No Bing resources found.");
      return;
    }

    Json::Value first_resource = firstResourceSet[BING_RESOURCE_KEY][0];

    std::string image_url = first_resource[BING_IMAGE_URL_KEY].asString();

    if (image_url.empty()) {
      Q_EMIT ErrorMessage("No Bing image URL found.");
      return;
    }

    tile_url_ = QString::fromStdString(image_url);
    SetMaxZoom(19);


    if (!first_resource[BING_IMAGE_URL_SUBDOMAIN_KEY].isArray() ||
      first_resource[BING_IMAGE_URL_SUBDOMAIN_KEY].empty())
    {
      Q_EMIT ErrorMessage("No image URL subdomains; maybe that's ok sometimes?");
    }

    for (const auto & subdomain : first_resource[BING_IMAGE_URL_SUBDOMAIN_KEY]) {
      subdomains_.push_back(QString::fromStdString(subdomain.asString()));
    }

    // The terms of use require crediting the providers whose imagery is on
    // screen, so keep each provider's coverage to match against the view.
    providers_.clear();
    for (const auto & provider_json : first_resource[BING_IMAGERY_PROVIDERS_KEY]) {
      ImageryProvider provider;
      provider.attribution =
        QString::fromStdString(provider_json[BING_ATTRIBUTION_KEY].asString());
      for (const auto & area_json : provider_json[BING_COVERAGE_AREAS_KEY]) {
        // bbox is [south, west, north, east] in degrees.
        const Json::Value & bbox = area_json[BING_BBOX_KEY];
        if (!bbox.isArray() || bbox.size() != 4) {
          continue;
        }
        CoverageArea area;
        area.zoom_min = area_json[BING_ZOOM_MIN_KEY].asInt();
        area.zoom_max = area_json[BING_ZOOM_MAX_KEY].asInt();
        area.south = bbox[0].asDouble();
        area.west = bbox[1].asDouble();
        area.north = bbox[2].asDouble();
        area.east = bbox[3].asDouble();
        provider.coverage_areas.push_back(area);
      }
      if (!provider.attribution.isEmpty()) {
        providers_.push_back(provider);
      }
    }

    std::string logo_uri = root[BING_BRAND_LOGO_URI_KEY].asString();
    if (!logo_uri.empty()) {
      QNetworkRequest request{QUrl(QString::fromStdString(logo_uri))};
      request.setAttribute(QNetworkRequest::User, QString(LOGO_REQUEST));
      network_manager_.get(request);
    }

    Q_EMIT InfoMessage("API Key Set.");

    is_ready_ = true;
  }
}
}  // namespace tile_map
