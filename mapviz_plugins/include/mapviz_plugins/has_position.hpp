// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
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
#ifndef MAPVIZ_PLUGINS__HAS_POSITION_HPP_
#define MAPVIZ_PLUGINS__HAS_POSITION_HPP_

#include <cmath>

namespace mapviz_plugins
{
/// Whether a gps_msgs::msg::GPSFix or sensor_msgs::msg::NavSatFix carries a
/// position that can be drawn.
///
/// A fix status alone is not enough: a receiver can report a fix before it has
/// a position, and both messages use NaN for a position they do not have.
/// STATUS_NO_FIX comes from the message's own status type, so the same check
/// serves GPSStatus and NavSatStatus. A NaN altitude is only a 2D fix, which
/// still has a place on the map.
template<typename FixT>
bool HasPosition(const FixT & fix)
{
  using StatusT = decltype(fix.status);
  return fix.status.status != StatusT::STATUS_NO_FIX &&
         std::isfinite(fix.latitude) && std::isfinite(fix.longitude);
}
}  // namespace mapviz_plugins

#endif  // MAPVIZ_PLUGINS__HAS_POSITION_HPP_
