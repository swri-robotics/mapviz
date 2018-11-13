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

#include <mapviz_plugins/image_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDialog>
#include <QGLWidget>

// ROS libraries
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <mapviz/select_topic_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::ImagePlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  ImagePlugin::ImagePlugin() :
    config_widget_(new QWidget()),
    anchor_(TOP_LEFT),
    units_(PIXELS),
    offset_x_(0),
    offset_y_(0),
    width_(320),
    height_(240),
    transport_("default"),
    has_image_(false),
    last_width_(0),
    last_height_(0),
    original_aspect_ratio_(1.0)
  {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
    QObject::connect(ui_.anchor, SIGNAL(activated(QString)), this, SLOT(SetAnchor(QString)));
    QObject::connect(ui_.units, SIGNAL(activated(QString)), this, SLOT(SetUnits(QString)));
    QObject::connect(ui_.offsetx, SIGNAL(valueChanged(int)), this, SLOT(SetOffsetX(int)));
    QObject::connect(ui_.offsety, SIGNAL(valueChanged(int)), this, SLOT(SetOffsetY(int)));
    QObject::connect(ui_.width, SIGNAL(valueChanged(double)), this, SLOT(SetWidth(double)));
    QObject::connect(ui_.height, SIGNAL(valueChanged(double)), this, SLOT(SetHeight(double)));
    QObject::connect(this,SIGNAL(VisibleChanged(bool)),this,SLOT(SetSubscription(bool)));
    QObject::connect(ui_.keep_ratio, SIGNAL(toggled(bool)), this, SLOT(KeepRatioChanged(bool)));
    QObject::connect(ui_.transport_combo_box, SIGNAL(activated(const QString&)),
                     this, SLOT(SetTransport(const QString&)));

    ui_.width->setKeyboardTracking(false);
    ui_.height->setKeyboardTracking(false);
  }

  ImagePlugin::~ImagePlugin()
  {
  }

  void ImagePlugin::SetOffsetX(int offset)
  {
    offset_x_ = offset;
  }

  void ImagePlugin::SetOffsetY(int offset)
  {
    offset_y_ = offset;
  }

  void ImagePlugin::SetWidth(double width)
  {
    width_ = width;
  }

  void ImagePlugin::SetHeight(double height)
  {
    height_ = height;
  }

  void ImagePlugin::SetAnchor(QString anchor)
  {
    if (anchor == "top left")
    {
      anchor_ = TOP_LEFT;
    }
    else if (anchor == "top center")
    {
      anchor_ = TOP_CENTER;
    }
    else if (anchor == "top right")
    {
      anchor_ = TOP_RIGHT;
    }
    else if (anchor == "center left")
    {
      anchor_ = CENTER_LEFT;
    }
    else if (anchor == "center")
    {
      anchor_ = CENTER;
    }
    else if (anchor == "center right")
    {
      anchor_ = CENTER_RIGHT;
    }
    else if (anchor == "bottom left")
    {
      anchor_ = BOTTOM_LEFT;
    }
    else if (anchor == "bottom center")
    {
      anchor_ = BOTTOM_CENTER;
    }
    else if (anchor == "bottom right")
    {
      anchor_ = BOTTOM_RIGHT;
    }
  }

  void ImagePlugin::SetUnits(QString units)
  {
    // do this in both cases to avoid image clamping
    ui_.width->setMaximum(10000);
    ui_.height->setMaximum(10000);

    if (units == "pixels")
    {
      ui_.width->setDecimals(0);
      ui_.height->setDecimals(0);
      units_ = PIXELS;
      width_  = width_ * double(canvas_->width()) / 100.0;
      height_ = height_ * double(canvas_->height()) / 100.0;
      ui_.width->setSuffix(" px");
      ui_.height->setSuffix(" px");
    }
    else if (units == "percent")
    {
      ui_.width->setDecimals(1);
      ui_.height->setDecimals(1);
      units_ = PERCENT;
      width_ = width_ * 100.0 / double(canvas_->width());
      height_ =  height_ * 100.0 / double(canvas_->height());
      ui_.width->setSuffix(" %");
      ui_.height->setSuffix(" %");
    }
    ui_.width->setValue( width_ );
    ui_.height->setValue( height_ );

    if( units_ == PERCENT)
    {
      ui_.width->setMaximum(100);
      ui_.height->setMaximum(100);
    }

  }
  void ImagePlugin::SetSubscription(bool visible)
  {
    if(topic_.empty())
    {
      return;
    }
    else if(!visible)
    {
      image_sub_.shutdown();
      ROS_INFO("Dropped subscription to %s", topic_.c_str());
    }
    else
    {
      Resubscribe();
    }
  }

  void ImagePlugin::SetTransport(const QString& transport)
  {
    transport_ = transport.toStdString();
    ROS_INFO("Changing image_transport to %s.", transport_.c_str());
    TopicEdited();
  }

  void ImagePlugin::KeepRatioChanged(bool checked)
  {
    ui_.height->setEnabled( !checked );
    if( checked )
    {
      ui_.height->setValue( width_ * original_aspect_ratio_ );
    }
  }

  void ImagePlugin::Resubscribe()
  {
    if (transport_ == "default")
    {
      force_resubscribe_ = true;
      TopicEdited();
    }
  }

  void ImagePlugin::SelectTopic()
  {
    ros::master::TopicInfo topic = mapviz::SelectTopicDialog::selectTopic(
      "sensor_msgs/Image");

    if(topic.name.empty())
    {
      topic.name.clear();
      TopicEdited();

    }
    if (!topic.name.empty())
    {
      ui_.topic->setText(QString::fromStdString(topic.name));
      TopicEdited();
    }
  }

  void ImagePlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    if(!this->Visible())
    {
      PrintWarning("Topic is Hidden");
      initialized_ = false;
      has_message_ = false;
      if (!topic.empty())
      {
        topic_ = topic;
      }
      image_sub_.shutdown();
      return;
    }
    // Re-subscribe if either the topic or the image transport
    // have changed.
    if (force_resubscribe_ ||
        topic != topic_ ||
        image_sub_.getTransport() != transport_)
    {
      force_resubscribe_ = false;
      initialized_ = false;
      has_message_ = false;
      topic_ = topic;
      PrintWarning("No messages received.");

      image_sub_.shutdown();

      if (!topic_.empty())
      {
        boost::shared_ptr<image_transport::ImageTransport> it;
        if (transport_ == "default")
        {
          ROS_DEBUG("Using default transport.");
          image_transport::ImageTransport it(node_);
          image_sub_ = it.subscribe(topic_, 1, &ImagePlugin::imageCallback, this);
        }
        else
        {
          ROS_DEBUG("Setting transport to %s on %s.",
                   transport_.c_str(), local_node_.getNamespace().c_str());

          local_node_.setParam("image_transport", transport_);
          image_transport::ImageTransport it(local_node_);
          image_sub_ = it.subscribe(topic_, 1, &ImagePlugin::imageCallback, this,
                                    image_transport::TransportHints(transport_,
                                                                    ros::TransportHints(),
                                                                    local_node_));
        }

        ROS_INFO("Subscribing to %s", topic_.c_str());
      }
    }
  }

  void ImagePlugin::imageCallback(const sensor_msgs::ImageConstPtr& image)
  {
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }

    image_ = *image;

    try
    {
      cv_image_ = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (const cv_bridge::Exception& e)
    {
      PrintError(e.what());
      return;
    }

    last_width_ = 0;
    last_height_ = 0;
    original_aspect_ratio_ = (double)image->height / (double)image->width;

    if( ui_.keep_ratio->isChecked() )
    {
      double height =  width_ * original_aspect_ratio_;
      if (units_ == PERCENT)
      {
        height *= (double)canvas_->width() / (double)canvas_->height();
      }
      ui_.height->setValue(height);
    }

    has_image_ = true;
  }

  void ImagePlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void ImagePlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void ImagePlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  QWidget* ImagePlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool ImagePlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;

    return true;
  }

  void ImagePlugin::ScaleImage(double width, double height)
  {
    if (!has_image_)
    {
      return;
    }

    cv::resize(cv_image_->image, scaled_image_, cvSize2D32f(width, height), 0, 0, CV_INTER_AREA);
  }

  void ImagePlugin::DrawIplImage(cv::Mat *image)
  {
    // TODO(malban) glTexture2D may be more efficient than glDrawPixels

    if (image == NULL || image->cols == 0 || image->rows == 0)
    {
      return;
    }

    GLenum format;
    switch (image->channels())
    {
      case 1:
        format = GL_LUMINANCE;
        break;
      case 2:
        format = GL_LUMINANCE_ALPHA;
        break;
      case 3:
        format = GL_BGR;
        break;
      default:
        return;
    }

    glPixelZoom(1.0f, -1.0f);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glDrawPixels(image->cols, image->rows, format, GL_UNSIGNED_BYTE, image->ptr());

    PrintInfo("OK");
  }

  void ImagePlugin::Draw(double x, double y, double scale)
  {
    // Calculate the correct offsets and dimensions
    double x_offset = offset_x_;
    double y_offset = offset_y_;
    double width = width_;
    double height = height_;

    if (units_ == PERCENT)
    {
      x_offset = offset_x_ * canvas_->width() / 100.0;
      y_offset = offset_y_ * canvas_->height() / 100.0;
      width = width_ * canvas_->width() / 100.0;
      height = height_ * canvas_->height() / 100.0;
    }

    if( ui_.keep_ratio->isChecked() )
    {
      height = original_aspect_ratio_ * width;
    }

    // Scale the source image if necessary
    if (width != last_width_ || height != last_height_)
    {
      ScaleImage(width, height);
    }

    // Calculate the correct render position
    double x_pos = 0;
    double y_pos = 0;
    if (anchor_ == TOP_LEFT)
    {
      x_pos = x_offset;
      y_pos = y_offset;
    }
    else if (anchor_ == TOP_CENTER)
    {
      x_pos = (canvas_->width() - width) / 2.0 + x_offset;
      y_pos = y_offset;
    }
    else if (anchor_ == TOP_RIGHT)
    {
      x_pos = canvas_->width() - width - x_offset;
      y_pos = y_offset;
    }
    else if (anchor_ == CENTER_LEFT)
    {
      x_pos = x_offset;
      y_pos = (canvas_->height() - height) / 2.0 + y_offset;
    }
    else if (anchor_ == CENTER)
    {
      x_pos = (canvas_->width() - width) / 2.0 + x_offset;
      y_pos = (canvas_->height() - height) / 2.0 + y_offset;
    }
    else if (anchor_ == CENTER_RIGHT)
    {
      x_pos = canvas_->width() - width - x_offset;
      y_pos = (canvas_->height() - height) / 2.0 + y_offset;
    }
    else if (anchor_ == BOTTOM_LEFT)
    {
      x_pos = x_offset;
      y_pos = canvas_->height() - height - y_offset;
    }
    else if (anchor_ == BOTTOM_CENTER)
    {
      x_pos = (canvas_->width() - width) / 2.0 + x_offset;
      y_pos = canvas_->height() - height - y_offset;
    }
    else if (anchor_ == BOTTOM_RIGHT)
    {
      x_pos = canvas_->width() - width - x_offset;
      y_pos = canvas_->height() - height - y_offset;
    }

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, canvas_->width(), canvas_->height(), 0, -0.5f, 0.5f);

    glRasterPos2d(x_pos, y_pos);

    DrawIplImage(&scaled_image_);

    glPopMatrix();

    last_width_ = width;
    last_height_ = height;
  }

  void ImagePlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    // Note that image_transport should be loaded before the
    // topic to make sure the transport is set appropriately before we
    // subscribe.
    if (node["image_transport"])
    {
      node["image_transport"] >> transport_;
      int index = ui_.transport_combo_box->findText( QString::fromStdString(transport_) );
      if (index != -1)
      {
        ui_.transport_combo_box->setCurrentIndex(index);
      }
      else
      {
        ROS_WARN("Saved image transport %s is unavailable.",
                 transport_.c_str());
      }
    }

    if (node["topic"])
    {
      std::string topic;
      node["topic"] >> topic;
      ui_.topic->setText(topic.c_str());
      TopicEdited();
    }

    if (node["anchor"])
    {
      std::string anchor;
      node["anchor"] >> anchor;
      ui_.anchor->setCurrentIndex(ui_.anchor->findText(anchor.c_str()));
      SetAnchor(anchor.c_str());
    }

    if (node["units"])
    {
      std::string units;
      node["units"] >> units;
      ui_.units->setCurrentIndex(ui_.units->findText(units.c_str()));
      SetUnits(units.c_str());
    }

    if (node["offset_x"])
    {
      node["offset_x"] >> offset_x_;
      ui_.offsetx->setValue(offset_x_);
    }

    if (node["offset_y"])
    {
      node["offset_y"] >> offset_y_;
      ui_.offsety->setValue(offset_y_);
    }

    if (node["width"])
    {
      node["width"] >> width_;
      ui_.width->setValue(width_);
    }

    if (node["height"])
    {
      node["height"] >> height_;
      ui_.height->setValue(height_);
    }

    if (node["keep_ratio"])
    {
      bool keep;
      node["keep_ratio"] >> keep;
      ui_.keep_ratio->setChecked( keep );
    }
  }

  void ImagePlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "topic" << YAML::Value << ui_.topic->text().toStdString();
    emitter << YAML::Key << "anchor" << YAML::Value << AnchorToString(anchor_);
    emitter << YAML::Key << "units" << YAML::Value << UnitsToString(units_);
    emitter << YAML::Key << "offset_x" << YAML::Value << offset_x_;
    emitter << YAML::Key << "offset_y" << YAML::Value << offset_y_;
    emitter << YAML::Key << "width" << YAML::Value << width_;
    emitter << YAML::Key << "height" << YAML::Value << height_;
    emitter << YAML::Key << "keep_ratio" << YAML::Value << ui_.keep_ratio->isChecked();
    emitter << YAML::Key << "image_transport" << YAML::Value << transport_;
  }

  std::string ImagePlugin::AnchorToString(Anchor anchor)
  {
    std::string anchor_string = "top left";

    if (anchor == TOP_LEFT)
    {
      anchor_string = "top left";
    }
    else if (anchor == TOP_CENTER)
    {
      anchor_string = "top center";
    }
    else if (anchor == TOP_RIGHT)
    {
      anchor_string = "top right";
    }
    else if (anchor == CENTER_LEFT)
    {
      anchor_string = "center left";
    }
    else if (anchor == CENTER)
    {
      anchor_string = "center";
    }
    else if (anchor == CENTER_RIGHT)
    {
      anchor_string = "center right";
    }
    else if (anchor == BOTTOM_LEFT)
    {
      anchor_string = "bottom left";
    }
    else if (anchor == BOTTOM_CENTER)
    {
      anchor_string = "bottom center";
    }
    else if (anchor == BOTTOM_RIGHT)
    {
      anchor_string = "bottom right";
    }

    return anchor_string;
  }

  std::string ImagePlugin::UnitsToString(Units units)
  {
    std::string units_string = "pixels";

    if (units == PIXELS)
    {
      units_string = "pixels";
    }
    else if (units == PERCENT)
    {
      units_string = "percent";
    }

    return units_string;
  }

  void ImagePlugin::CreateLocalNode()
  {
    // This is the same way ROS generates anonymous node names.
    // See http://docs.ros.org/api/roscpp/html/this__node_8cpp_source.html
    // Giving each image plugin a unique node means that we can control
    // its image transport individually.
    char buf[200];
    snprintf(buf, sizeof(buf), "image_%llu", (unsigned long long)ros::WallTime::now().toNSec());
    local_node_ = ros::NodeHandle(node_, buf);
  }

  void ImagePlugin::SetNode(const ros::NodeHandle& node)
  {
    node_ = node;

    // As soon as we have a node, we can find the available image transports
    // and add them to our combo box.
    image_transport::ImageTransport it(node_);
    std::vector<std::string> transports = it.getLoadableTransports();
    Q_FOREACH (const std::string& transport, transports)
    {
      QString qtransport = QString::fromStdString(transport).replace("image_transport/", "");
      ui_.transport_combo_box->addItem(qtransport);
    }

    CreateLocalNode();
  }
}

