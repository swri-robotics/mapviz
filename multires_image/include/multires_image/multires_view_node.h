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

#ifndef MULTIRES_IMAGE_MULTIRES_VIEW_NODE_H_
#define MULTIRES_IMAGE_MULTIRES_VIEW_NODE_H_

// C++ standard libraries
#include <string>

// Boost libraries
#include <boost/thread.hpp>

// QT libraries
#include <QtGui/QMainWindow>
#include <QMouseEvent>
#include <QLabel>
#include <QShowEvent>

// ROS libraries
#include <ros/ros.h>

#include <multires_image/QGLMap.h>
#include <multires_image/tile_set.h>

namespace multires_image
{
  class MultiresViewNode : public QMainWindow
  {
    Q_OBJECT

  public:
    MultiresViewNode(int argc, char **argv, QWidget *parent = 0, Qt::WFlags flags = 0);
    ~MultiresViewNode();

    virtual void showEvent(QShowEvent* event);

    void Initialize();

    void Spin();

  private:
    void SpinLoop();

    int argc_;
    char** argv_;

    ros::NodeHandle* node_;
    boost::thread*  thread_;

    bool initialized_;

    std::string image_path_;

    TileSet* tile_set_;
  };
}

#endif  // MULTIRES_IMAGE_MULTIRES_VIEW_NODE_H_
