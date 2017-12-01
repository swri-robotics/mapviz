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
    offset_(0,0),
    size_(320, 240),
    transport_("default"),
    has_image_(false),
    last_size_(0,0),
    original_img_size_(1,1)
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
    offset_.setX( offset );
  }

  void ImagePlugin::SetOffsetY(int offset)
  {
    offset_.setY( offset );
  }

  void ImagePlugin::SetWidth(double width)
  {
    size_.setWidth( width );
  }

  void ImagePlugin::SetHeight(double height)
  {
    size_.setHeight( height );
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
    if (units == "pixels")
    {
      ui_.width->setDecimals(0);
      ui_.height->setDecimals(0);
      units_ = PIXELS;
      // convert percent to pixels
      double scale_ratio = (size_.width() / 100.0);
      size_.setWidth(  original_img_size_.width() * scale_ratio );
      size_.setHeight( original_img_size_.height() * scale_ratio );
      ui_.width->setSuffix(" px");
      ui_.height->setSuffix(" px");
      ui_.offsetx->setSuffix(" px");
      ui_.offsety->setSuffix(" px");
    }
    else if (units == "percent")
    {
      ui_.width->setDecimals(1);
      ui_.height->setDecimals(1);
      units_ = PERCENT;
      size_.setWidth(  100.0 * size_.width() /  original_img_size_.width() );
      size_.setHeight( 100.0 * size_.height() /  original_img_size_.height() );
      ui_.width->setSuffix(" % (original)");
      ui_.height->setSuffix(" % (original)");
      ui_.offsetx->setSuffix(" % (canvas)");
      ui_.offsety->setSuffix(" % (canvas)");
    }
    ui_.width->setValue( size_.width() );
    ui_.height->setValue( size_.height() );
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
      image_transport::ImageTransport it(local_node_);
      image_sub_ = it.subscribe(topic_, 1, &ImagePlugin::imageCallback, this);

      ROS_INFO("Subscribing to %s", topic_.c_str());
    }
  }

  void ImagePlugin::SetTransport(const QString& transport)
  {
    ROS_INFO("Changing image_transport to %s.", transport.toStdString().c_str());
    transport_ = transport;
    TopicEdited();
  }

  void ImagePlugin::KeepRatioChanged(bool checked)
  {
    ui_.height->setEnabled( !checked );
    if( checked )
    {
      ui_.height->setValue( size_.width() * original_img_size_.height() / original_img_size_.width() );
    }
  }

  void ImagePlugin::Resubscribe()
  {
    if (transport_ == QString::fromStdString("default"))
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
        image_sub_.getTransport() != transport_.toStdString())
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
        if (transport_ == QString::fromStdString("default"))
        {
          ROS_DEBUG("Using default transport.");
          it = boost::make_shared<image_transport::ImageTransport>(node_);
          image_sub_ = it->subscribe(topic_, 1, &ImagePlugin::imageCallback, this);
        }
        else
        {
          ROS_DEBUG("Setting transport to %s on %s.",
                   transport_.toStdString().c_str(), local_node_.getNamespace().c_str());

          local_node_.setParam("image_transport", transport_.toStdString());
          it = boost::make_shared<image_transport::ImageTransport>(local_node_);
          image_sub_ = it->subscribe(topic_, 1, &ImagePlugin::imageCallback, this,
                                     image_transport::TransportHints(transport_.toStdString(),
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

    last_size_ = QSizeF(0,0);
    original_img_size_ = QSizeF(image_.width, image_.height);

    if( ui_.keep_ratio->isChecked() )
    {
      const double ratio =  original_img_size_.height() / original_img_size_.width();
      if( units_ == PERCENT){
        ui_.height->setValue( size_.width() );
      }
      else{
        ui_.height->setValue( size_.width() * ratio );
      }
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
    double x_offset = offset_.x();
    double y_offset = offset_.y();
    double width = size_.width();
    double height = size_.height();

    if (units_ == PERCENT)
    {
      x_offset *= canvas_->width() / 100.0;
      y_offset *= canvas_->height() / 100.0;
      width  *= original_img_size_.width() / 100.0;
      height *= original_img_size_.height() / 100.0;
    }

    if( ui_.keep_ratio->isChecked() )
    {
      height = (original_img_size_.height() / original_img_size_.width()) * width;
    }

    // Scale the source image if necessary
    if (width != last_size_.width() || height != last_size_.height())
    {
      ScaleImage(width, height);
      last_size_ = QSizeF(width,height);
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
  }

  void ImagePlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    // Note that image_transport should be loaded before the
    // topic to make sure the transport is set appropriately before we
    // subscribe.
    if (node["image_transport"])
    {
      std::string transport;
      node["image_transport"] >> transport;
      transport_ = QString::fromStdString(transport);
      int index = ui_.transport_combo_box->findText(transport_);
      if (index != -1)
      {
        ui_.transport_combo_box->setCurrentIndex(index);
      }
      else
      {
        ROS_WARN("Saved image transport %s is unavailable.",
                 transport_.toStdString().c_str());
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
      double offset;
      node["offset_x"] >> offset;
      ui_.offsetx->setValue(offset);
    }

    if (node["offset_y"])
    {
      double offset;
      node["offset_y"] >> offset;
      ui_.offsety->setValue(offset);
    }

    if (node["width"])
    {
      double width;
      node["width"] >> width;
      ui_.width->setValue(width);
    }

    if (node["height"])
    {
      double height;
      node["height"] >> height;
      ui_.height->setValue(height);
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
    emitter << YAML::Key << "offset_x" << YAML::Value << offset_.x();
    emitter << YAML::Key << "offset_y" << YAML::Value << offset_.y();
    emitter << YAML::Key << "width" << YAML::Value << size_.width();
    emitter << YAML::Key << "height" << YAML::Value << size_.height();
    emitter << YAML::Key << "keep_ratio" << YAML::Value << ui_.keep_ratio->isChecked();
    emitter << YAML::Key << "image_transport" << YAML::Value << transport_.toStdString();
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

