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
#include <gtest/gtest.h>
#include <QColor>
#include <QImage>

#include <cstdio>
#include <string>

#include <mapviz/video_writer.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace
{
/// The average BGR color of a region of @p frame.
cv::Scalar Mean(const cv::Mat & frame, int top, int bottom)
{
  return cv::mean(frame.rowRange(top, bottom));
}
}  // namespace

TEST(VideoWriter, WritesFramesTheRightWayUp)
{
  const std::string path = ::testing::TempDir() + "test_video_writer.avi";
  const int width = 64;
  const int height = 48;

  // Frames arrive the way QOpenGLWidget::grabFramebuffer() returns them: top
  // row first, in ARGB32, which is BGRA in memory.
  QImage frame(width, height, QImage::Format_ARGB32);
  frame.fill(QColor(255, 0, 0));
  for (int row = height / 2; row < height; row++) {
    for (int col = 0; col < width; col++) {
      frame.setPixelColor(col, row, QColor(0, 0, 255));
    }
  }

  mapviz::VideoWriter writer;
  ASSERT_TRUE(writer.initializeWriter(path, width, height));
  for (int i = 0; i < 3; i++) {
    writer.processFrame(frame);
  }
  writer.stop();

  cv::VideoCapture video(path);
  cv::Mat written;
  ASSERT_TRUE(video.read(written));
  std::remove(path.c_str());

  // Red on top and blue below, not flipped, and not with red and blue
  // swapped.
  const cv::Scalar top = Mean(written, 0, height / 2 - 4);
  const cv::Scalar bottom = Mean(written, height / 2 + 4, height);
  EXPECT_GT(top[2], 200) << "top: " << top;
  EXPECT_LT(top[0], 60) << "top: " << top;
  EXPECT_GT(bottom[0], 200) << "bottom: " << bottom;
  EXPECT_LT(bottom[2], 60) << "bottom: " << bottom;
}
