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

/**
 * \file
 *
 * multires_image::MultiresViewNode. Description.
 *   - \b Parameters
 *      - \e "node_name"/image_path <tt>[string]</tt> - Description. [""]
 */

#include <string>

// QT libraries
#include <QString>
#include <QApplication>
#include <QMessageBox>
#include <QImage>
#include <QFileInfo>

#include <multires_image/multires_view_node.h>

namespace multires_image
{
  MultiresViewNode::MultiresViewNode(int argc, char **argv, QWidget *parent, Qt::WindowFlags flags) :
    QMainWindow(parent, flags),
    argc_(argc),
    argv_(argv),
    node_(NULL),
    thread_(NULL),
    initialized_(false)
  {
    setCentralWidget(new QGLMap());
    this->setMinimumSize(640, 480);
  }

  MultiresViewNode::~MultiresViewNode()
  {
    delete node_;
  }

  void MultiresViewNode::Spin()
  {
    if (thread_ == NULL)
    {
      thread_ = new boost::thread(&MultiresViewNode::SpinLoop, this);
    }
  }

  void MultiresViewNode::SpinLoop()
  {
    while (ros::ok())
    {
      ros::spinOnce();

      usleep(10);
    }
  }

  void MultiresViewNode::showEvent(QShowEvent* event)
  {
    Initialize();
  }

  void MultiresViewNode::Initialize()
  {
    if (!initialized_)
    {
      ros::init(argc_, argv_, "multires_view_node");

      node_ = new ros::NodeHandle();

      node_->param(ros::this_node::getName() + "/image_path", image_path_, std::string(""));


      tile_set_ = new TileSet(image_path_);

      if (tile_set_->Load())
      {
        QGLMap* glMap = reinterpret_cast<QGLMap*>(centralWidget());
        glMap->SetTiles(tile_set_);
        glMap->UpdateView();
      }
      else
      {
        QMessageBox::warning(this, "Error", "Failed to load tiles.");
      }

      Spin();

      initialized_ = true;
    }
  }
}

int main(int argc, char **argv)
{
  // Initialize QT
  QApplication app(argc, argv);

  multires_image::MultiresViewNode node(argc, argv);
  node.show();

  return app.exec();
}
