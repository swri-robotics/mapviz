// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
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

#include <mapviz/video_writer.h>

#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace mapviz
{
  bool VideoWriter::initializeWriter(const std::string& directory, int width, int height)
  {
    QMutexLocker locker(&video_mutex_);
    if (!video_writer_)
    {
      width_ = width;
      height_ = height;

      ROS_INFO("Initializing recording:\nWidth/Height/Filename: %d / %d / %s", width_, height_, directory.c_str() );
      video_writer_ = boost::make_shared<cv::VideoWriter>(
          directory,
          CV_FOURCC('M', 'J', 'P', 'G'),
          30,
          cv::Size(width_, height_));

      if (!video_writer_->isOpened())
      {
        ROS_ERROR("Failed to open video file for writing.");
        stop();
        return false;
      }
    }

    return true;
  }

  bool VideoWriter::isRecording()
  {
    return video_writer_.get() != NULL;
  }

  void VideoWriter::processFrame(QImage frame)
  {
    try
    {
      ROS_DEBUG_THROTTLE(1.0, "VideoWriter::processFrame()");
      {
        QMutexLocker locker(&video_mutex_);
        if (!video_writer_)
        {
          ROS_WARN_THROTTLE(1.0, "Got frame, but video writer wasn't open.");
          return;
        }
      }

      cv::Mat image;
      cv::Mat temp_image;
      switch (frame.format())
      {
        case QImage::Format_ARGB32:
          // The image received should have its format set to ARGB32, but it's
          // actually BGRA.  Need to convert it to BGR and flip it vertically
          // before giving it to the cv::VideoWriter.
          image = cv::Mat(frame.height(), frame.width(), CV_8UC4, frame.bits());
          cv::cvtColor(image, temp_image, CV_BGRA2BGR);
          cv::flip(temp_image, image, 0);
          break;
        default:
          ROS_WARN_THROTTLE(1.0, "Unexpected image format: %d", frame.format());
          return;
      }

      {
        QMutexLocker locker(&video_mutex_);
        if (video_writer_)
        {
          ROS_DEBUG_THROTTLE(1.0, "Writing frame.");
          video_writer_->write(image);
        }
      }
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_THROTTLE(1.0, "Error when processing video frame: %s", e.what());
    }
  }

  void VideoWriter::stop()
  {
    ROS_INFO("Stopping video recording.");
    QMutexLocker locker(&video_mutex_);
    video_writer_.reset();
  }
}