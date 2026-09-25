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

#ifndef TILE_MAP__STADIA_SOURCE_HPP_
#define TILE_MAP__STADIA_SOURCE_HPP_

#include <QString>

#include <string>

#include <tile_map/tile_source.hpp>

namespace tile_map
{
class StadiaSource : public TileSource
{
  Q_OBJECT

public:
  explicit StadiaSource(
    const QString & name,
    const QString & base_url,
    bool is_custom,
    int32_t max_zoom);

  size_t GenerateTileHash(int32_t level, int64_t x, int64_t y) override;
  QString GenerateTileUrl(int32_t level, int64_t x, int64_t y) override;
  QString GetType() const override;

  QString GetApiKey() const;
  void SetApiKey(const QString & api_key);

  static const QString STADIA_TYPE;

private:
  std::hash<std::string> hash_;
  QString api_key_;
};
}  // namespace tile_map

#endif  // TILE_MAP__STADIA_SOURCE_HPP_
