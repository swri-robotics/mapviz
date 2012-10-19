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

#include "disparity_plugin.h"

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(mapviz_plugins, disparity, mapviz_plugins::DisparityPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{

  DisparityPlugin::DisparityPlugin() :
    config_widget_(new QWidget()),
    anchor_(TOP_LEFT),
    units_(PIXELS),
    offset_x_(0),
    offset_y_(0),
    width_(320),
    height_(240),
    has_image_(false),
    last_width_(0),
    last_height_(0)
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
    QObject::connect(ui_.width, SIGNAL(valueChanged(int)), this, SLOT(SetWidth(int)));
    QObject::connect(ui_.height, SIGNAL(valueChanged(int)), this, SLOT(SetHeight(int)));
  }

  DisparityPlugin::~DisparityPlugin()
  {

  }

  void DisparityPlugin::SetOffsetX(int offset)
  {
    offset_x_ = offset;
    canvas_->update();
  }

  void DisparityPlugin::SetOffsetY(int offset)
  {
    offset_y_ = offset;
    canvas_->update();
  }

  void DisparityPlugin::SetWidth(int width)
  {
    width_ = width;

    canvas_->update();
  }

  void DisparityPlugin::SetHeight(int height)
  {
    height_ = height;

    canvas_->update();
  }

  void DisparityPlugin::SetAnchor(QString anchor)
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

    canvas_->update();
  }

  void DisparityPlugin::SetUnits(QString units)
  {
    if (units == "pixels")
    {
      units_ = PIXELS;
    }
    else if (units == "percent")
    {
      units_ = PERCENT;
    }

    canvas_->update();
  }

  void DisparityPlugin::SelectTopic()
  {
    QDialog dialog;
    Ui::topicselect ui;
    ui.setupUi(&dialog);

    std::vector<ros::master::TopicInfo> topics;
    ros::master::getTopics(topics);

    for (unsigned int i = 0; i < topics.size(); i++)
    {
      if (topics[i].datatype == "stereo_msgs/DisparityImage")
      {
        ui.displaylist->addItem(topics[i].name.c_str());
      }
    }
    ui.displaylist->setCurrentRow(0);

    dialog.exec();

    if (dialog.result() == QDialog::Accepted && ui.displaylist->selectedItems().count() == 1)
    {
      ui_.topic->setText(ui.displaylist->selectedItems().first()->text());
      TopicEdited();
    }
  }

  void DisparityPlugin::TopicEdited()
  {
    if (ui_.topic->text().toStdString() != topic_)
    {
      initialized_ = false;
      has_message_ = false;
      topic_ = ui_.topic->text().toStdString();
      PrintWarning("No messages received.");

      disparity_sub_.shutdown();
      disparity_sub_ = node_.subscribe(topic_, 1, &DisparityPlugin::disparityCallback, this);

      ROS_INFO("Subscribing to %s", topic_.c_str());
    }
  }

  void DisparityPlugin::disparityCallback(const stereo_msgs::DisparityImageConstPtr disparity)
  {
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }

    if (disparity->min_disparity == 0.0 && disparity->max_disparity == 0.0)
    {
      PrintError("Min and max disparity not set.");
      has_image_ = false;
      return;
    }

    if (disparity->image.encoding != sensor_msgs::image_encodings::TYPE_32FC1)
    {
      PrintError("Invalid encoding.");
      has_image_ = false;
      return;
    }

    disparity_ = *disparity;

    // Colormap and display the disparity image
    float min_disparity = disparity->min_disparity;
    float max_disparity = disparity->max_disparity;
    float multiplier = 255.0f / (max_disparity - min_disparity);

    const cv::Mat_<float> dmat(
        disparity->image.height,
        disparity->image.width,
        (float*)&disparity->image.data[0],
        disparity->image.step);

    disparity_color_.create(disparity->image.height, disparity->image.width);

    for (int row = 0; row < disparity_color_.rows; ++row)
    {
      const float* d = dmat[row];
      for (int col = 0; col < disparity_color_.cols; ++col)
      {
        int index = (d[col] - min_disparity) * multiplier + 0.5;
        index = std::min(255, std::max(0, index));
        // Fill as BGR
        disparity_color_(row, col)[2] = colormap[3*index + 0];
        disparity_color_(row, col)[1] = colormap[3*index + 1];
        disparity_color_(row, col)[0] = colormap[3*index + 2];
      }
    }

    last_width_ = 0;
    last_height_ = 0;

    has_image_ = true;

    canvas_->update();
  }

  void DisparityPlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_ERROR("Error: %s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void DisparityPlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_INFO("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void DisparityPlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  QWidget* DisparityPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool DisparityPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;

    return true;
  }

  void DisparityPlugin::ScaleImage(int width, int height)
  {
    if (!has_image_)
      return;

    cv::resize(disparity_color_, scaled_image_, cvSize(width, height), 0, 0, CV_INTER_AREA);
  }

  void DisparityPlugin::DrawIplImage(cv::Mat *image)
  {
    // TODO glTexture2D may be more efficient than glDrawPixels

    if (!has_image_)
      return;

    if (image == NULL)
      return;

    if (image->cols == 0 || image->rows == 0)
      return;

    GLenum format;
    switch(image->channels())
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

    glPixelZoom( 1.0, -1.0 );
    glDrawPixels(image->cols, image->rows, format, GL_UNSIGNED_BYTE, image->ptr());

    PrintInfo("OK");
  }

  void DisparityPlugin::Draw(double x, double y, double scale)
  {
    // Calculate the correct offsets and dimensions
    int x_offset = offset_x_;
    int y_offset = offset_y_;
    int width = width_;
    int height = height_;
    if (units_ == PERCENT)
    {
      x_offset = offset_x_ * canvas_->width() / 100.0;
      y_offset = offset_y_ * canvas_->height() / 100.0;
      width = width_ * canvas_->width() / 100.0;
      height = height_ * canvas_->height() / 100.0;
    }

    // Scale the source image if necessary
    if (width != last_width_ || height != last_height_)
    {
      ScaleImage(width, height);
    }

    // Calculate the correct render position
    int x_pos = 0;
    int y_pos = 0;
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

    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, canvas_->width(), canvas_->height(), 0, -0.5f, 0.5f);

    glRasterPos2f(x_pos, y_pos);

    DrawIplImage(&scaled_image_);

    glPopMatrix();

    last_width_ = width;
    last_height_ = height;
  }

  void DisparityPlugin::LoadConfiguration(const YAML::Node& node, const std::string& config_path)
  {
    std::string topic;
    node["topic"] >> topic;
    ui_.topic->setText(topic.c_str());

    TopicEdited();

    std::string anchor;
    node["anchor"] >> anchor;
    SetAnchor(anchor.c_str());

    std::string units;
    node["units"] >> units;
    SetAnchor(units.c_str());

    node["offset_x"] >> offset_x_;
    node["offset_y"] >> offset_y_;
    node["width"] >> width_;
    node["height"] >> height_;
  }

  void DisparityPlugin::SaveConfiguration(YAML::Emitter& emitter, const std::string& config_path)
  {
    emitter << YAML::Key << "topic" << YAML::Value << ui_.topic->text().toStdString();
    emitter << YAML::Key << "anchor" << YAML::Value << AnchorToString(anchor_);
    emitter << YAML::Key << "units" << YAML::Value << UnitsToString(units_);
    emitter << YAML::Key << "offset_x" << YAML::Value << offset_x_;
    emitter << YAML::Key << "offset_y" << YAML::Value << offset_y_;
    emitter << YAML::Key << "width" << YAML::Value << width_;
    emitter << YAML::Key << "height" << YAML::Value << height_;
  }

  std::string DisparityPlugin::AnchorToString(Anchor anchor)
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

  std::string DisparityPlugin::UnitsToString(Units units)
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

  unsigned char DisparityPlugin::colormap[768] =
    { 150, 150, 150,
      107, 0, 12,
      106, 0, 18,
      105, 0, 24,
      103, 0, 30,
      102, 0, 36,
      101, 0, 42,
      99, 0, 48,
      98, 0, 54,
      97, 0, 60,
      96, 0, 66,
      94, 0, 72,
      93, 0, 78,
      92, 0, 84,
      91, 0, 90,
      89, 0, 96,
      88, 0, 102,
      87, 0, 108,
      85, 0, 114,
      84, 0, 120,
      83, 0, 126,
      82, 0, 131,
      80, 0, 137,
      79, 0, 143,
      78, 0, 149,
      77, 0, 155,
      75, 0, 161,
      74, 0, 167,
      73, 0, 173,
      71, 0, 179,
      70, 0, 185,
      69, 0, 191,
      68, 0, 197,
      66, 0, 203,
      65, 0, 209,
      64, 0, 215,
      62, 0, 221,
      61, 0, 227,
      60, 0, 233,
      59, 0, 239,
      57, 0, 245,
      56, 0, 251,
      55, 0, 255,
      54, 0, 255,
      52, 0, 255,
      51, 0, 255,
      50, 0, 255,
      48, 0, 255,
      47, 0, 255,
      46, 0, 255,
      45, 0, 255,
      43, 0, 255,
      42, 0, 255,
      41, 0, 255,
      40, 0, 255,
      38, 0, 255,
      37, 0, 255,
      36, 0, 255,
      34, 0, 255,
      33, 0, 255,
      32, 0, 255,
      31, 0, 255,
      29, 0, 255,
      28, 0, 255,
      27, 0, 255,
      26, 0, 255,
      24, 0, 255,
      23, 0, 255,
      22, 0, 255,
      20, 0, 255,
      19, 0, 255,
      18, 0, 255,
      17, 0, 255,
      15, 0, 255,
      14, 0, 255,
      13, 0, 255,
      11, 0, 255,
      10, 0, 255,
      9, 0, 255,
      8, 0, 255,
      6, 0, 255,
      5, 0, 255,
      4, 0, 255,
      3, 0, 255,
      1, 0, 255,
      0, 4, 255,
      0, 10, 255,
      0, 16, 255,
      0, 22, 255,
      0, 28, 255,
      0, 34, 255,
      0, 40, 255,
      0, 46, 255,
      0, 52, 255,
      0, 58, 255,
      0, 64, 255,
      0, 70, 255,
      0, 76, 255,
      0, 82, 255,
      0, 88, 255,
      0, 94, 255,
      0, 100, 255,
      0, 106, 255,
      0, 112, 255,
      0, 118, 255,
      0, 124, 255,
      0, 129, 255,
      0, 135, 255,
      0, 141, 255,
      0, 147, 255,
      0, 153, 255,
      0, 159, 255,
      0, 165, 255,
      0, 171, 255,
      0, 177, 255,
      0, 183, 255,
      0, 189, 255,
      0, 195, 255,
      0, 201, 255,
      0, 207, 255,
      0, 213, 255,
      0, 219, 255,
      0, 225, 255,
      0, 231, 255,
      0, 237, 255,
      0, 243, 255,
      0, 249, 255,
      0, 255, 255,
      0, 255, 249,
      0, 255, 243,
      0, 255, 237,
      0, 255, 231,
      0, 255, 225,
      0, 255, 219,
      0, 255, 213,
      0, 255, 207,
      0, 255, 201,
      0, 255, 195,
      0, 255, 189,
      0, 255, 183,
      0, 255, 177,
      0, 255, 171,
      0, 255, 165,
      0, 255, 159,
      0, 255, 153,
      0, 255, 147,
      0, 255, 141,
      0, 255, 135,
      0, 255, 129,
      0, 255, 124,
      0, 255, 118,
      0, 255, 112,
      0, 255, 106,
      0, 255, 100,
      0, 255, 94,
      0, 255, 88,
      0, 255, 82,
      0, 255, 76,
      0, 255, 70,
      0, 255, 64,
      0, 255, 58,
      0, 255, 52,
      0, 255, 46,
      0, 255, 40,
      0, 255, 34,
      0, 255, 28,
      0, 255, 22,
      0, 255, 16,
      0, 255, 10,
      0, 255, 4,
      2, 255, 0,
      8, 255, 0,
      14, 255, 0,
      20, 255, 0,
      26, 255, 0,
      32, 255, 0,
      38, 255, 0,
      44, 255, 0,
      50, 255, 0,
      56, 255, 0,
      62, 255, 0,
      68, 255, 0,
      74, 255, 0,
      80, 255, 0,
      86, 255, 0,
      92, 255, 0,
      98, 255, 0,
      104, 255, 0,
      110, 255, 0,
      116, 255, 0,
      122, 255, 0,
      128, 255, 0,
      133, 255, 0,
      139, 255, 0,
      145, 255, 0,
      151, 255, 0,
      157, 255, 0,
      163, 255, 0,
      169, 255, 0,
      175, 255, 0,
      181, 255, 0,
      187, 255, 0,
      193, 255, 0,
      199, 255, 0,
      205, 255, 0,
      211, 255, 0,
      217, 255, 0,
      223, 255, 0,
      229, 255, 0,
      235, 255, 0,
      241, 255, 0,
      247, 255, 0,
      253, 255, 0,
      255, 251, 0,
      255, 245, 0,
      255, 239, 0,
      255, 233, 0,
      255, 227, 0,
      255, 221, 0,
      255, 215, 0,
      255, 209, 0,
      255, 203, 0,
      255, 197, 0,
      255, 191, 0,
      255, 185, 0,
      255, 179, 0,
      255, 173, 0,
      255, 167, 0,
      255, 161, 0,
      255, 155, 0,
      255, 149, 0,
      255, 143, 0,
      255, 137, 0,
      255, 131, 0,
      255, 126, 0,
      255, 120, 0,
      255, 114, 0,
      255, 108, 0,
      255, 102, 0,
      255, 96, 0,
      255, 90, 0,
      255, 84, 0,
      255, 78, 0,
      255, 72, 0,
      255, 66, 0,
      255, 60, 0,
      255, 54, 0,
      255, 48, 0,
      255, 42, 0,
      255, 36, 0,
      255, 30, 0,
      255, 24, 0,
      255, 18, 0,
      255, 12, 0,
      255,  6, 0,
      255,  0, 0,
    };
}

