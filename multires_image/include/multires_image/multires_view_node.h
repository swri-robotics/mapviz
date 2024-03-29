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

#ifndef MULTIRES_IMAGE_MULTIRES_VIEW_NODE_H_
#define MULTIRES_IMAGE_MULTIRES_VIEW_NODE_H_

// C++ standard libraries
#include <string>

// Boost libraries
#include <boost/thread.hpp>

// QT libraries
#include <QMainWindow>
#include <QMouseEvent>
#include <QLabel>
#include <QShowEvent>

// ROS libraries
#include <rclcpp/rclcpp.hpp>

#include <multires_image/QGLMap.h>
#include <multires_image/tile_set.h>

namespace multires_image
{
  class MultiresViewNode : public QMainWindow
  {
    Q_OBJECT

  public:
    MultiresViewNode(int argc, char **argv, QWidget *parent = 0, Qt::WindowFlags flags = Qt::WindowFlags());
    ~MultiresViewNode() override = default;

    virtual void showEvent(QShowEvent* event) override;

    void Initialize();

    void Spin();

  private:
    void SpinLoop();

    int argc_;
    char** argv_;

    rclcpp::Node::SharedPtr node_;
    boost::thread*  thread_;

    bool initialized_;

    std::string image_path_;

    TileSet* tile_set_;
  };
}

#endif  // MULTIRES_IMAGE_MULTIRES_VIEW_NODE_H_
