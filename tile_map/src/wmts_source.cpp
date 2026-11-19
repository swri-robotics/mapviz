// *****************************************************************************
//
// Copyright (c) 2015-2025, Southwest Research Institute® (SwRI®)
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

#include <tile_map/wmts_source.hpp>

#include <QList>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QRegularExpression>
#include <QStringList>
#include <QUrl>
#include <QtGlobal>
#include <json/json.h>

#include <algorithm>

#include <tile_map/tile_source.hpp>

namespace tile_map
{
const QString WmtsSource::WMTS_TYPE = "wmts";

WmtsSource::WmtsSource(
  const QString & name,
  const QString & base_url,
  bool is_custom,
  int32_t max_zoom)
: TileSource()
{
  name_ = name;
  base_url_ = base_url;
  is_custom_ = is_custom;
  max_zoom_ = max_zoom;
  min_zoom_ = 1;
}

QString WmtsSource::GetType() const
{
  return WMTS_TYPE;
}

size_t WmtsSource::GenerateTileHash(int32_t level, int64_t x, int64_t y)
{
  return hash_(GenerateTileUrl(level, x, y).toStdString());
}

QString WmtsSource::GenerateTileUrl(int32_t level, int64_t x, int64_t y)
{
  QString url(base_url_);
  url.replace(QString::fromStdString("{level}"), QString::number(level));
  // "{z}" is what most tile servers call this in their own documentation, so
  // accept it rather than requesting a URL with a literal "{z}" in it.
  url.replace(QString::fromStdString("{z}"), QString::number(level));
  url.replace(QString::fromStdString("{x}"), QString::number(x));
  url.replace(QString::fromStdString("{y}"), QString::number(y));

  return url;
}

QString WmtsSource::ValidateBaseUrl(const QString & base_url)
{
  if (base_url.trimmed().isEmpty()) {
    return "The base URL is empty.";
  }

  QStringList missing;
  if (!base_url.contains("{level}") && !base_url.contains("{z}")) {
    missing << "{level}";
  }
  if (!base_url.contains("{x}")) {
    missing << "{x}";
  }
  if (!base_url.contains("{y}")) {
    missing << "{y}";
  }

  if (missing.isEmpty()) {
    return "";
  }

  return "The base URL is missing " + missing.join(", ") +
         ".  Without them every tile resolves to the same image.  A URL looks "
         "like http://localhost/osm_tiles/{level}/{x}/{y}.png ({z} also works "
         "in place of {level}).";
}

void WmtsSource::RequestServiceLimits()
{
  if (is_custom_ || limits_requested_) {
    return;
  }

  const QString url = ArcGisServiceInfoUrl(base_url_);
  if (url.isEmpty()) {
    return;
  }

  limits_requested_ = true;
  if (network_manager_ == nullptr) {
    network_manager_ = new QNetworkAccessManager(this);
  }

  QNetworkReply * reply = network_manager_->get(QNetworkRequest(QUrl(url)));
  QObject::connect(
    reply, &QNetworkReply::finished, this, [this, reply]() {
      reply->deleteLater();
      if (reply->error() != QNetworkReply::NoError) {
        // Keep the built-in max zoom, and try again the next time the
        // source is selected.
        limits_requested_ = false;
        return;
      }
      HandleServiceInfo(reply->readAll());
    });
}

void WmtsSource::HandleServiceInfo(const QByteArray & service_info)
{
  // A level outside the range we can draw means the description is wrong,
  // so keep the current value rather than clamping to a number that came
  // from it.
  const int32_t level = ParseArcGisMaxLevel(service_info);
  if (level < min_zoom_ || level > MAX_SUPPORTED_ZOOM || level == max_zoom_) {
    return;
  }

  max_zoom_ = level;
  Q_EMIT MaxZoomChanged();
}

QString WmtsSource::ArcGisServiceInfoUrl(const QString & base_url)
{
  static const QRegularExpression map_server(
    "^(https?://[^?#]*/arcgis/rest/services/[^?#]+?/MapServer)(?:[/?#]|$)",
    QRegularExpression::CaseInsensitiveOption);

  const QRegularExpressionMatch match = map_server.match(base_url.trimmed());
  if (!match.hasMatch()) {
    return "";
  }
  return match.captured(1) + "?f=json";
}

int32_t WmtsSource::ParseArcGisMaxLevel(const QByteArray & service_info)
{
  Json::Reader reader;
  Json::Value root;
  if (!reader.parse(service_info.toStdString(), root) || !root.isObject()) {
    return -1;
  }

  const Json::Value & lods = root["tileInfo"]["lods"];
  if (!lods.isArray()) {
    return -1;
  }

  if (root["maxLOD"].isIntegral()) {
    const int32_t max_lod = root["maxLOD"].asInt();
    for (const Json::Value & lod : lods) {
      if (lod["level"].isIntegral() && lod["level"].asInt() == max_lod) {
        return max_lod;
      }
    }
    return -1;
  }

  // maxScale is the most detailed scale the service draws; 0 means it sets
  // no limit, which says nothing about how deep its cache goes.  The scales
  // in the description are rounded, so allow a little slack when comparing.
  const double max_scale = root["maxScale"].isNumeric() ? root["maxScale"].asDouble() : 0.0;
  if (max_scale <= 0.0) {
    return -1;
  }

  int32_t deepest = -1;
  for (const Json::Value & lod : lods) {
    if (!lod["level"].isIntegral() || !lod["scale"].isNumeric()) {
      continue;
    }
    if (lod["scale"].asDouble() >= max_scale * (1.0 - 1e-6)) {
      deepest = std::max(deepest, static_cast<int32_t>(lod["level"].asInt()));
    }
  }
  return deepest;
}
}  // namespace tile_map
