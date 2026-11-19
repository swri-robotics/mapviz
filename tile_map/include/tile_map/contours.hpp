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

#ifndef TILE_MAP__CONTOURS_HPP_
#define TILE_MAP__CONTOURS_HPP_

#include <cstddef>
#include <vector>

namespace tile_map
{
/// Every this many contour lines is an index contour, drawn heavier and
/// labeled with its height.
constexpr int INDEX_CONTOUR_EVERY = 5;

/// A point on a grid of heights, in cells: x counts columns and y counts rows,
/// and the middle of the cell in row r and column c is at (c, r).
struct GridPoint
{
  double x;
  double y;
};

/// One contour line: a run of points where the ground is at `height`.
struct ContourLine
{
  /// Height in meters.
  double height;
  /// True for index contours (see INDEX_CONTOUR_EVERY).
  bool index;
  /// True when the line comes back to its start.  The first point is not
  /// repeated at the end.
  bool closed;
  std::vector<GridPoint> points;
};

/**
 * The contour interval used when none is set: the smallest of 1, 2 or 5 times
 * a power of ten that is at least 1.5 pixels' worth of ground, and never less
 * than 1 m.  That keeps lines apart on moderate slopes at any zoom.
 *
 * @param meters_per_pixel  Scale of the view
 */
double AutoContourInterval(double meters_per_pixel);

/**
 * Traces contour lines through a grid of heights (marching squares).
 *
 * There is a line at every multiple of `interval`.  A cell with a corner that
 * isn't a number (missing data) has no lines through it, so lines stop at the
 * edge of the data.  Where a cell is a saddle, the average of its corners
 * decides which way the lines turn.
 *
 * @param heights       Heights in meters, row by row
 * @param width         Number of columns
 * @param height        Number of rows
 * @param interval      Height between lines, in meters
 * @param max_segments  The most line segments to trace (one per cell a line
 *   crosses)
 * @param lines         Filled with the lines found
 * @return False, with `lines` empty, if the lines need more than
 *   `max_segments` segments or the input is the wrong size
 */
bool TraceContours(
  const std::vector<float> & heights,
  int width,
  int height,
  double interval,
  size_t max_segments,
  std::vector<ContourLine> & lines);

/// Where a height label fits on an index contour.
struct LabelSpot
{
  /// The middle of the label.
  GridPoint position;
  /// The points on the line `half_length` before and after the middle, which
  /// give the label its direction.
  GridPoint before;
  GridPoint after;
  double height;
};

/**
 * Places for height labels on the index contours, about every `spacing` cells
 * along each line, where the line runs nearly straight for `half_length` cells
 * on either side.  Lines too short for a label get none.
 */
std::vector<LabelSpot> LabelSpots(
  const std::vector<ContourLine> & lines, double spacing, double half_length);
}  // namespace tile_map

#endif  // TILE_MAP__CONTOURS_HPP_
