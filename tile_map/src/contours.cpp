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

#include <tile_map/contours.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <unordered_map>
#include <utility>

namespace tile_map
{
namespace
{
/// A piece of a contour line inside one cell.  Its ends are on two of the
/// cell's edges, named so that the cells on either side of an edge agree.
struct Segment
{
  uint64_t edge_a;
  uint64_t edge_b;
  GridPoint a;
  GridPoint b;
};

/// How much shorter than the path along the line the straight distance
/// across a label may be, for the label to sit on it.
constexpr double MIN_STRAIGHTNESS = 0.9;

/// Joins the segments at one height into lines.
void JoinSegments(
  const std::vector<Segment> & segments, double height, bool index,
  std::vector<ContourLine> & lines)
{
  // Each edge is crossed at most once per height, so at most two segments
  // (one from each cell beside it) end on it.
  std::unordered_map<uint64_t, std::pair<int64_t, int64_t>> at_edge;
  at_edge.reserve(segments.size() * 2);
  for (size_t i = 0; i < segments.size(); ++i) {
    for (uint64_t edge : {segments[i].edge_a, segments[i].edge_b}) {
      auto inserted = at_edge.emplace(edge, std::make_pair(static_cast<int64_t>(i), int64_t{-1}));
      if (!inserted.second) {
        inserted.first->second.second = static_cast<int64_t>(i);
      }
    }
  }

  std::vector<bool> used(segments.size(), false);
  // Follows the line from `edge`, leaving segment `current`, and adds the
  // points it passes to `add`.  Returns true if it came back to `start`.
  auto follow = [&](size_t start, size_t current, uint64_t edge, auto add) {
      while (true) {
        const std::pair<int64_t, int64_t> & ends = at_edge[edge];
        const int64_t next = ends.first == static_cast<int64_t>(current) ? ends.second : ends.first;
        if (next < 0) {
          return false;
        }
        if (static_cast<size_t>(next) == start) {
          return true;
        }
        if (used[next]) {
          return false;
        }
        used[next] = true;
        const Segment & segment = segments[next];
        if (segment.edge_a == edge) {
          add(segment.b);
          edge = segment.edge_b;
        } else {
          add(segment.a);
          edge = segment.edge_a;
        }
        current = static_cast<size_t>(next);
      }
    };

  for (size_t s = 0; s < segments.size(); ++s) {
    if (used[s]) {
      continue;
    }
    used[s] = true;
    std::deque<GridPoint> points{segments[s].a, segments[s].b};
    const bool closed = follow(
      s, s, segments[s].edge_b, [&points](const GridPoint & p) {points.push_back(p);});
    if (closed) {
      // The walk ended on the first point again.
      points.pop_back();
    } else {
      follow(s, s, segments[s].edge_a, [&points](const GridPoint & p) {points.push_front(p);});
    }
    lines.push_back({height, index, closed, std::vector<GridPoint>(points.begin(), points.end())});
  }
}

double Distance(const GridPoint & a, const GridPoint & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}
}  // namespace

double AutoContourInterval(double meters_per_pixel)
{
  if (!std::isfinite(meters_per_pixel) || meters_per_pixel <= 0.0) {
    return 1.0;
  }
  const double target = std::max(1.0, 1.5 * meters_per_pixel);
  const double power = std::pow(10.0, std::floor(std::log10(target)));
  for (double factor : {1.0, 2.0, 5.0}) {
    // Allow for rounding in the power, so 2 m per pixel gives 5, not 10.
    if (factor * power >= target * (1.0 - 1e-9)) {
      return factor * power;
    }
  }
  return 10.0 * power;
}

bool TraceContours(
  const std::vector<float> & heights,
  int width,
  int height,
  double interval,
  size_t max_segments,
  std::vector<ContourLine> & lines)
{
  lines.clear();
  if (width < 2 || height < 2 || heights.size() != static_cast<size_t>(width) * height ||
    !(interval > 0.0))
  {
    return false;
  }

  const uint64_t w = static_cast<uint64_t>(width);
  // Edges are numbered from the cell corner they start at: a horizontal edge
  // runs east from it, a vertical edge south.
  auto horizontal = [w](int row, int col) {return (row * w + col) * 2;};
  auto vertical = [w](int row, int col) {return (row * w + col) * 2 + 1;};
  auto at = [&heights, w](int row, int col) {
      return static_cast<double>(heights[row * w + col]);
    };

  // Segments for each multiple of the interval.
  std::map<int64_t, std::vector<Segment>> levels;
  size_t count = 0;
  for (int row = 0; row + 1 < height; ++row) {
    for (int col = 0; col + 1 < width; ++col) {
      const double tl = at(row, col);
      const double tr = at(row, col + 1);
      const double br = at(row + 1, col + 1);
      const double bl = at(row + 1, col);
      if (std::isnan(tl) || std::isnan(tr) || std::isnan(br) || std::isnan(bl)) {
        continue;
      }
      const double low = std::min({tl, tr, br, bl});
      const double high = std::max({tl, tr, br, bl});
      const int64_t first = static_cast<int64_t>(std::ceil(low / interval));
      const int64_t last = static_cast<int64_t>(std::floor(high / interval));
      for (int64_t k = first; k <= last; ++k) {
        const double h = static_cast<double>(k) * interval;
        // A corner counts as above the line when it is at its height, so
        // ground that is level at the height gets a line around it, not
        // through it.
        const int shape = (tl >= h ? 8 : 0) | (tr >= h ? 4 : 0) | (br >= h ? 2 : 0) |
          (bl >= h ? 1 : 0);
        if (shape == 0 || shape == 15) {
          continue;
        }
        // Where the line crosses each edge, worked out the same way in the
        // cells on both sides of it.
        auto top = [&]() {
            return std::make_pair(
              horizontal(row, col),
              GridPoint{col + (h - tl) / (tr - tl), static_cast<double>(row)});
          };
        auto bottom = [&]() {
            return std::make_pair(
              horizontal(row + 1, col),
              GridPoint{col + (h - bl) / (br - bl), static_cast<double>(row + 1)});
          };
        auto left = [&]() {
            return std::make_pair(
              vertical(row, col), GridPoint{static_cast<double>(col), row + (h - tl) / (bl - tl)});
          };
        auto right = [&]() {
            return std::make_pair(
              vertical(row, col + 1),
              GridPoint{static_cast<double>(col + 1), row + (h - tr) / (br - tr)});
          };
        std::vector<Segment> & segments = levels[k];
        auto add = [&segments, &count](
          const std::pair<uint64_t, GridPoint> & a, const std::pair<uint64_t, GridPoint> & b) {
            segments.push_back({a.first, b.first, a.second, b.second});
            ++count;
          };
        const bool center_above = (tl + tr + br + bl) / 4.0 >= h;
        switch (shape) {
          case 1: case 14: add(left(), bottom()); break;
          case 2: case 13: add(bottom(), right()); break;
          case 3: case 12: add(left(), right()); break;
          case 4: case 11: add(top(), right()); break;
          case 6: case 9: add(top(), bottom()); break;
          case 7: case 8: add(left(), top()); break;
          case 5:
            // Northeast and southwest corners above.
            if (center_above) {
              add(left(), top());
              add(bottom(), right());
            } else {
              add(top(), right());
              add(left(), bottom());
            }
            break;
          case 10:
            // Northwest and southeast corners above.
            if (center_above) {
              add(top(), right());
              add(left(), bottom());
            } else {
              add(left(), top());
              add(bottom(), right());
            }
            break;
          default:
            break;
        }
        if (count > max_segments) {
          return false;
        }
      }
    }
  }

  for (const auto & level : levels) {
    const bool index = level.first % INDEX_CONTOUR_EVERY == 0;
    JoinSegments(level.second, static_cast<double>(level.first) * interval, index, lines);
  }
  return true;
}

std::vector<LabelSpot> LabelSpots(
  const std::vector<ContourLine> & lines, double spacing, double half_length)
{
  std::vector<LabelSpot> spots;
  if (!(spacing > 0.0) || !(half_length > 0.0)) {
    return spots;
  }
  for (const ContourLine & line : lines) {
    if (!line.index || line.points.size() < 2) {
      continue;
    }
    // Distance along the line to each point; a closed line ends back at its
    // first point.
    std::vector<GridPoint> points = line.points;
    if (line.closed) {
      points.push_back(points.front());
    }
    std::vector<double> along(points.size(), 0.0);
    for (size_t i = 1; i < points.size(); ++i) {
      along[i] = along[i - 1] + Distance(points[i - 1], points[i]);
    }
    const double length = along.back();
    if (length < 2.5 * half_length) {
      continue;
    }
    auto point_at = [&](double s) {
        if (line.closed) {
          s = std::fmod(s, length);
          if (s < 0.0) {
            s += length;
          }
        }
        s = std::clamp(s, 0.0, length);
        const size_t i = std::max<size_t>(
          1, std::lower_bound(along.begin(), along.end(), s) - along.begin());
        const size_t j = std::min(i, points.size() - 1);
        const double piece = along[j] - along[j - 1];
        const double t = piece > 0.0 ? (s - along[j - 1]) / piece : 0.0;
        return GridPoint{
        points[j - 1].x + t * (points[j].x - points[j - 1].x),
        points[j - 1].y + t * (points[j].y - points[j - 1].y)};
      };

    // Where labels would go if the line were straight, then the nearest
    // straight enough place to each.
    std::vector<double> wanted;
    if (length < spacing) {
      wanted.push_back(0.5 * length);
    } else {
      for (double s = 0.5 * spacing; s < length; s += spacing) {
        wanted.push_back(s);
      }
    }
    const double step = 0.5 * half_length;
    const int tries = std::max(1, static_cast<int>(0.25 * spacing / step));
    for (double s0 : wanted) {
      for (int t = 0; t <= 2 * tries; ++t) {
        // 0, +1, -1, +2, -2, ... steps away.
        const double offset = (t % 2 == 1 ? 1.0 : -1.0) * ((t + 1) / 2) * step;
        const double s = s0 + offset;
        if (!line.closed && (s < half_length || s > length - half_length)) {
          continue;
        }
        const GridPoint before = point_at(s - half_length);
        const GridPoint after = point_at(s + half_length);
        if (Distance(before, after) >= MIN_STRAIGHTNESS * 2.0 * half_length) {
          spots.push_back({point_at(s), before, after, line.height});
          break;
        }
      }
    }
  }
  return spots;
}
}  // namespace tile_map
