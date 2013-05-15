// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-58058A
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Steve Dellenback <sdellenback@swri.org> (210) 522-3914
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
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
#include <QtGui/QApplication>
#include <QString>
#include <QtGui/QMessageBox>
#include <QImage>
#include <QFileInfo>

#include <multires_image/multires_view_node.h>

namespace multires_image
{
  MultiresViewNode::MultiresViewNode(int argc, char **argv, QWidget *parent, Qt::WFlags flags) :
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
        QGLMap* glMap = (QGLMap*)centralWidget();
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

int main (int argc, char **argv)
{
  // Initialize QT
  QApplication app(argc, argv);

  multires_image::MultiresViewNode node(argc, argv);
  node.show();

  return app.exec();
}
