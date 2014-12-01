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

#include <mapviz/mapviz.h>

// C++ standard libraries
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

// Boost libraries
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// QT libraries
#include <QtGui/QApplication>
#include <QFileDialog>
#include <QActionGroup>
#include <QColorDialog>
#include <QLabel>

#include <math_util/constants.h>
#include <transform_util/frames.h>
#include <yaml_util/yaml_util.h>

#include <mapviz/config_item.h>

namespace mapviz
{
Mapviz::Mapviz(int argc, char **argv, QWidget *parent, Qt::WFlags flags) :
    QMainWindow(parent, flags),
    xy_pos_label_(new QLabel("fixed: 0.0,0.0")),
    lat_lon_pos_label_(new QLabel("lat/lon: 0.0,0.0")),
    argc_(argc),
    argv_(argv),
    initialized_(false),
    force_720p_(false),
    force_480p_(false),
    resizable_(true),
    background_(Qt::gray),
    capture_directory_("~"),
    updating_frames_(false),
    node_(NULL),
    canvas_(NULL)
{
  ui_.setupUi(this);

  xy_pos_label_->setVisible(false);
  lat_lon_pos_label_->setVisible(false);

  ui_.statusbar->addPermanentWidget(xy_pos_label_);
  ui_.statusbar->addPermanentWidget(lat_lon_pos_label_);
  
  spacer1_ = new QWidget(ui_.statusbar);
  spacer1_->setMaximumSize(22,22);
  spacer1_->setMinimumSize(22,22);
  ui_.statusbar->addPermanentWidget(spacer1_);
  
  screenshot_button_ = new QPushButton();
  screenshot_button_->setMaximumSize(22,22);
  screenshot_button_->setIcon(QIcon(":/images/image-x-generic.png"));
  screenshot_button_->setFlat(true);
  ui_.statusbar->addPermanentWidget(screenshot_button_);
  
  spacer2_ = new QWidget(ui_.statusbar);
  spacer2_->setMaximumSize(22,22);
  spacer2_->setMinimumSize(22,22);
  ui_.statusbar->addPermanentWidget(spacer2_);
  
  rec_button_ = new QPushButton();
  rec_button_->setMaximumSize(22,22);
  rec_button_->setIcon(QIcon(":/images/media-record.png"));
  rec_button_->setCheckable(true);
  rec_button_->setFlat(true);
  ui_.statusbar->addPermanentWidget(rec_button_);
  
  stop_button_ = new QPushButton();
  stop_button_->setMaximumSize(22,22);
  stop_button_->setIcon(QIcon(":/images/media-playback-stop.png"));
  stop_button_->setEnabled(false);
  stop_button_->setFlat(true);
  ui_.statusbar->addPermanentWidget(stop_button_);
  
  spacer3_ = new QWidget(ui_.statusbar);
  spacer3_->setMaximumSize(22,22);
  spacer3_->setMinimumSize(22,22);
  ui_.statusbar->addPermanentWidget(spacer3_);
  
  ui_.statusbar->setVisible(true);

  QActionGroup* group = new QActionGroup(this);

  ui_.actionForce_720p->setActionGroup(group);
  ui_.actionForce_480p->setActionGroup(group);
  ui_.actionResizable->setActionGroup(group);

  ui_.targetframe->addItem("<none>");

  canvas_ = new MapCanvas(this);
  setCentralWidget(canvas_);

  connect(canvas_, SIGNAL(Hover(double,double,double)), this, SLOT(Hover(double,double,double)));
  connect(ui_.configs, SIGNAL(ItemsMoved()), this, SLOT(ReorderDisplays()));
  connect(ui_.actionExit, SIGNAL(triggered()), this, SLOT(close()));

  connect(rec_button_, SIGNAL(toggled(bool)), this, SLOT(ToggleRecord(bool)));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(StopRecord()));
  connect(screenshot_button_, SIGNAL(clicked()), this, SLOT(Screenshot()));

  connect(ui_.blackbox_trigger_btn, SIGNAL(clicked()),
          this, SLOT(BlackBoxTrigger()));
  ui_.blackbox_trigger_btn->setEnabled(false);

  ui_.bg_color->setStyleSheet("background: " + background_.name() + ";");
  canvas_->SetBackground(background_);
}

Mapviz::~Mapviz()
{
  delete node_;
}

void Mapviz::showEvent(QShowEvent* event)
{
  Initialize();
}

void Mapviz::closeEvent(QCloseEvent* event)
{
  AutoSave();
}

void Mapviz::Initialize()
{
  if (!initialized_)
  {
    ros::init(argc_, argv_, "mapviz");

    spin_timer_.start(30);
    connect(&spin_timer_, SIGNAL(timeout()), this, SLOT(SpinOnce()));

    node_ = new ros::NodeHandle();
    tf_ = boost::make_shared<tf::TransformListener>();
    tf_manager_.Initialize(tf_);

    blackbox_monitor_timer_ = node_->createTimer(ros::Duration(0.1),
                                                 &Mapviz::BlackBoxTimer,
                                                 this);
    blackbox_trigger_srv_ = node_->serviceClient<std_srvs::Empty>(
      "/black_box/trigger_record");

    loader_ = new pluginlib::ClassLoader<MapvizPlugin>(
        "mapviz", "mapviz::MapvizPlugin");

    std::vector<std::string> plugins = loader_->getDeclaredClasses();
    for (unsigned int i = 0; i < plugins.size(); i++)
    {
      ROS_INFO("Found mapviz plugin: %s", plugins[i].c_str());
    }

    canvas_->InitializeTf(tf_);
    canvas_->SetFixedFrame(ui_.fixedframe->currentText().toStdString());
    canvas_->SetTargetFrame(ui_.targetframe->currentText().toStdString());

    ros::NodeHandle priv("~");

    std::string config;
    priv.param("config", config, QDir::homePath().toStdString() + "/.mapviz_config");

    Open(config);

    UpdateFrames();
    frame_timer_.start(1000);
    connect(&frame_timer_, SIGNAL(timeout()), this, SLOT(UpdateFrames()));

    save_timer_.start(10000);
    connect(&save_timer_, SIGNAL(timeout()), this, SLOT(AutoSave()));
    
    connect(&record_timer_, SIGNAL(timeout()), this, SLOT(CaptureVideoFrame()));

    initialized_ = true;
  }
}

void Mapviz::SpinOnce()
{
  if (ros::ok())
  {
    ros::spinOnce();
  }
  else
  {
    QApplication::exit();
  }
}

void Mapviz::UpdateFrames()
{
  std::vector<std::string> frames;
  tf_->getFrameStrings(frames);

  if ((int)frames.size() == ui_.fixedframe->count())
  {
    bool changed = false;
    for (unsigned int i = 0; i < frames.size(); i++)
    {
      if (frames[i] != ui_.fixedframe->itemText(i).toStdString())
      {
        changed = true;
      }
    }

    if (!changed)
      return;
  }

  updating_frames_ = true;
  ROS_INFO("Updating frames...");

  std::string current_fixed = ui_.fixedframe->currentText().toStdString();

  ui_.fixedframe->clear();
  for (unsigned int i = 0; i < frames.size(); i++)
  {
    ui_.fixedframe->addItem(frames[i].c_str());
  }

  if (current_fixed != "")
  {
    int index = ui_.fixedframe->findText(current_fixed.c_str());
    if (index < 0)
    {
      ui_.fixedframe->addItem(current_fixed.c_str());
    }

    index = ui_.fixedframe->findText(current_fixed.c_str());
    ui_.fixedframe->setCurrentIndex(index);
  }

  std::string current_target = ui_.targetframe->currentText().toStdString();

  ui_.targetframe->clear();
  ui_.targetframe->addItem("<none>");
  for (unsigned int i = 0; i < frames.size(); i++)
  {
    ui_.targetframe->addItem(frames[i].c_str());
  }

  if (current_target != "")
  {
    int index = ui_.targetframe->findText(current_target.c_str());
    if (index < 0)
    {
      ui_.targetframe->addItem(current_target.c_str());
    }

    index = ui_.targetframe->findText(current_target.c_str());
    ui_.targetframe->setCurrentIndex(index);
  }
  
  updating_frames_ = false;

  if (current_target != ui_.targetframe->currentText().toStdString())
  {
    TargetFrameSelected(ui_.targetframe->currentText());
  }

  if (current_fixed != ui_.fixedframe->currentText().toStdString())
  {
    FixedFrameSelected(ui_.fixedframe->currentText());
  }
}

void Mapviz::Force720p(bool on)
{
  if (force_720p_ != on)
  {
    force_720p_ = on;

    if (force_720p_)
    {
      force_480p_ = false;
      resizable_ = false;
    }

    AdjustWindowSize();
  }
}

void Mapviz::Force480p(bool on)
{
  if (force_480p_ != on)
  {
    force_480p_ = on;

    if (force_480p_)
    {
      force_720p_ = false;
      resizable_ = false;
    }

    AdjustWindowSize();
  }
}

void Mapviz::SetResizable(bool on)
{
  if (resizable_ != on)
  {
    resizable_ = on;

    if (resizable_)
    {
      force_720p_ = false;
      force_480p_ = false;
    }

    AdjustWindowSize();
  }
}

void Mapviz::AdjustWindowSize()
{
  canvas_->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
  setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));

  this->setMinimumSize(QSize(100, 100));
  this->setMaximumSize(QSize(10000, 10000));

  if (force_720p_)
  {
    canvas_->setMinimumSize(1280, 720);
    canvas_->setMaximumSize(1280, 720);
    canvas_->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
    adjustSize();
    this->setMaximumSize(this->sizeHint());
    this->setMinimumSize(this->sizeHint());
    setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  }
  else if (force_480p_)
  {
    canvas_->setMinimumSize(640, 480);
    canvas_->setMaximumSize(640, 480);
    canvas_->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
    adjustSize();
    this->setMaximumSize(this->sizeHint());
    this->setMinimumSize(this->sizeHint());
    setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  }
  else if (stop_button_->isEnabled())
  {
    canvas_->setMinimumSize(canvas_->width(), canvas_->height());
    canvas_->setMaximumSize(canvas_->width(), canvas_->height());
    canvas_->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
    adjustSize();
    this->setMaximumSize(this->sizeHint());
    this->setMinimumSize(this->sizeHint());
    setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  }
  else
  {
    canvas_->setMinimumSize(100, 100);
    canvas_->setMaximumSize(10000, 10000);
  }
}

void Mapviz::Open(const std::string& filename)
{
  ROS_INFO("Loading configuration from: %s", filename.c_str());

  YAML::Node doc;
  if (!yaml_util::LoadFile(filename, doc))
  {
    ROS_ERROR("Failed to load file: %s", filename.c_str());
    return;
  }

  try
  {
    boost::filesystem::path filepath(filename);
    std::string config_path = filepath.parent_path().string();

    ClearDisplays();

    if (yaml_util::FindValue(doc, "capture_directory"))
    {
      doc["capture_directory"] >> capture_directory_;
    }

    if (yaml_util::FindValue(doc, "fixed_frame"))
    {
      std::string fixed_frame;
      doc["fixed_frame"] >> fixed_frame;
      ui_.fixedframe->setEditText(fixed_frame.c_str());
    }

    if (yaml_util::FindValue(doc, "target_frame"))
    {
      std::string target_frame;
      doc["target_frame"] >> target_frame;
      ui_.targetframe->setEditText(target_frame.c_str());
    }

    if (yaml_util::FindValue(doc, "fix_orientation"))
    {
      bool fix_orientation = false;
      doc["fix_orientation"] >> fix_orientation;
      ui_.actionFix_Orientation->setChecked(fix_orientation);
    }

    if (yaml_util::FindValue(doc, "show_displays"))
    {
      bool show_displays = false;
      doc["show_displays"] >> show_displays;
      ui_.actionConfig_Dock->setChecked(show_displays);
    }

    if (yaml_util::FindValue(doc, "show_capture_tools"))
    {
      bool show_capture_tools = false;
      doc["show_capture_tools"] >> show_capture_tools;
      ui_.actionShow_Capture_Tools->setChecked(show_capture_tools);
    }
    
    if (yaml_util::FindValue(doc, "show_status_bar"))
    {
      bool show_status_bar = false;
      doc["show_status_bar"] >> show_status_bar;
      ui_.actionShow_Status_Bar->setChecked(show_status_bar);
    }

    if (yaml_util::FindValue(doc, "show_capture_tools"))
    {
      bool show_capture_tools = false;
      doc["show_capture_tools"] >> show_capture_tools;
      ui_.actionShow_Capture_Tools->setChecked(show_capture_tools);
    }

    if (yaml_util::FindValue(doc, "window_width"))
    {
      int window_width = 0;
      doc["window_width"] >> window_width;
      resize(window_width, height());
    }

    if (yaml_util::FindValue(doc, "window_height"))
    {
      int window_height = 0;
      doc["window_height"] >> window_height;
      resize(width(), window_height);
    }

    if (yaml_util::FindValue(doc, "view_scale"))
    {
      float scale = 0;
      doc["view_scale"] >> scale;
      canvas_->SetViewScale(scale);
    }

    if (yaml_util::FindValue(doc, "offset_x"))
    {
      float x = 0;
      doc["offset_x"] >> x;
      canvas_->SetOffsetX(x);
    }

    if (yaml_util::FindValue(doc, "offset_x"))
    {
      float y = 0;
      doc["offset_y"] >> y;
      canvas_->SetOffsetY(y);
    }

    if (yaml_util::FindValue(doc, "force_720p"))
    {
      bool force_720p;
      doc["force_720p"] >> force_720p;

      if (force_720p)
      {
        ui_.actionForce_720p->setChecked(true);
      }
    }
    
    if (yaml_util::FindValue(doc, "force_480p"))
    {
      bool force_480p;
      doc["force_480p"] >> force_480p;
      
      if (force_480p)
      {
        ui_.actionForce_480p->setChecked(true);
      }
    }

    bool use_latest_transforms = true;
    if (yaml_util::FindValue(doc, "use_latest_transforms"))
    {
      doc["use_latest_transforms"] >> use_latest_transforms;
    }
    ui_.uselatesttransforms->setChecked(use_latest_transforms);
    canvas_->ToggleUseLatestTransforms(use_latest_transforms);

    if (yaml_util::FindValue(doc, "background"))
    {
      std::string color;
      doc["background"] >> color;
      background_ = QColor(color.c_str());
      ui_.bg_color->setStyleSheet("background: " + background_.name() + ";");
      canvas_->SetBackground(background_);
    }

    if (yaml_util::FindValue(doc, "displays"))
    {
      const YAML::Node& displays = doc["displays"];
      for (uint32_t i = 0; i < displays.size(); i++)
      {
        std::string type, name;
        displays[i]["type"] >> type;
        displays[i]["name"] >> name;

        const YAML::Node& config = displays[i]["config"];

        bool visible = false;
        config["visible"] >> visible;

        bool collapsed = false;
        config["collapsed"] >> collapsed;

        MapvizPluginPtr plugin =
            CreateNewDisplay(name, type, visible, collapsed);
        plugin->LoadConfig(config, config_path);
      }
    }
  }
  catch (const YAML::ParserException& e)
  {
    ROS_ERROR("%s", e.what());
    return;
  }
  catch (const YAML::Exception& e)
  {
    ROS_ERROR("%s", e.what());
    return;
  }
}

void Mapviz::Save(const std::string& filename)
{
  ROS_INFO("Saving configuration to: %s", filename.c_str());
  std::ofstream fout(filename.c_str());
  if (fout.fail())
  {
    ROS_ERROR("Failed to open file: %s", filename.c_str());
    return;
  }

  boost::filesystem::path filepath(filename);
  std::string config_path = filepath.parent_path().string();

  YAML::Emitter out;

  out << YAML::BeginMap;
  out << YAML::Key << "capture_directory" << YAML::Value << capture_directory_;
  out << YAML::Key << "fixed_frame" << YAML::Value << ui_.fixedframe->currentText().toStdString();
  out << YAML::Key << "target_frame" << YAML::Value << ui_.targetframe->currentText().toStdString();
  out << YAML::Key << "fix_orientation" << YAML::Value << ui_.actionFix_Orientation->isChecked();
  out << YAML::Key << "show_displays" << YAML::Value << ui_.actionConfig_Dock->isChecked();
  out << YAML::Key << "show_status_bar" << YAML::Value << ui_.actionShow_Status_Bar->isChecked();
  out << YAML::Key << "show_capture_tools" << YAML::Value << ui_.actionShow_Capture_Tools->isChecked();
  out << YAML::Key << "window_width" << YAML::Value << width();
  out << YAML::Key << "window_height" << YAML::Value << height();
  out << YAML::Key << "view_scale" << YAML::Value << canvas_->ViewScale();
  out << YAML::Key << "offset_x" << YAML::Value << canvas_->OffsetX();
  out << YAML::Key << "offset_y" << YAML::Value << canvas_->OffsetY();
  out << YAML::Key << "use_latest_transforms" << YAML::Value << ui_.uselatesttransforms->isChecked();
  out << YAML::Key << "background" << YAML::Value << background_.name().toStdString();

  if (force_720p_)
  {
    out << YAML::Key << "force_720p" << YAML::Value << force_720p_;
  }

  if (force_480p_)
  {
    out << YAML::Key << "force_480p" << YAML::Value << force_480p_;
  }

  if (ui_.configs->count() > 0)
  {
    out << YAML::Key << "displays"<< YAML::Value << YAML::BeginSeq;

    for (int i = 0; i < ui_.configs->count(); i++)
    {
      out << YAML::BeginMap;
      out << YAML::Key << "type" << YAML::Value << plugins_[ui_.configs->item(i)]->Type();
      out << YAML::Key << "name" << YAML::Value << (static_cast<ConfigItem*>(ui_.configs->itemWidget(ui_.configs->item(i))))->Name().toStdString();
      out << YAML::Key << "config" << YAML::Value;
      out << YAML::BeginMap;

      out << YAML::Key << "visible" << YAML::Value << plugins_[ui_.configs->item(i)]->Visible();
      out << YAML::Key << "collapsed" << YAML::Value << (static_cast<ConfigItem*>(ui_.configs->itemWidget(ui_.configs->item(i))))->Collapsed();

      plugins_[ui_.configs->item(i)]->SaveConfig(out, config_path);

      out << YAML::EndMap;
      out << YAML::EndMap;
    }

    out << YAML::EndSeq;
  }

  out << YAML::EndMap;

  fout << out.c_str();
  fout.close();
}

void Mapviz::AutoSave()
{
  Save(QDir::homePath().toStdString() + "/.mapviz_config");
}

void Mapviz::OpenConfig()
{
  QFileDialog dialog(this, "Select Config File");
  dialog.setFileMode(QFileDialog::ExistingFile);
  dialog.setNameFilter(tr("Mapviz Config Files (*.mvc)"));

  dialog.exec();

  if (dialog.result() == QDialog::Accepted && dialog.selectedFiles().count() == 1)
  {
    std::string path = dialog.selectedFiles().first().toStdString();
    Open(path);
  }
}

void Mapviz::SaveConfig()
{
  QFileDialog dialog(this, "Save Config File");
  dialog.setFileMode(QFileDialog::AnyFile);
  dialog.setAcceptMode(QFileDialog::AcceptSave);
  dialog.setNameFilter(tr("Mapviz Config Files (*.mvc)"));

  dialog.exec();

  if (dialog.result() == QDialog::Accepted && dialog.selectedFiles().count() == 1)
  {
    std::string path = dialog.selectedFiles().first().toStdString();

    Save(path);
  }
}

void Mapviz::SelectNewDisplay()
{
  ROS_INFO("Select new display ...");
  QDialog dialog;
  Ui::pluginselect ui;
  ui.setupUi(&dialog);

  std::vector<std::string> plugins = loader_->getDeclaredClasses();
  for (unsigned int i = 0; i < plugins.size(); i++)
  {
    QString type(plugins[i].c_str());
    type = type.split('/').last();
    ui.displaylist->addItem(type);
  }
  ui.displaylist->setCurrentRow(0);

  dialog.exec();

  if (dialog.result() == QDialog::Accepted)
  {
    int row =ui.displaylist->currentRow();
    std::string type = plugins[row].c_str();
    std::string name = "new display";
    CreateNewDisplay(name, type, true, false);
  }
}

void Mapviz::Hover(double x, double y, double scale)
{
  if (ui_.statusbar->isVisible())
  {
    if (scale == 0)
    {
      xy_pos_label_->setVisible(false);
      lat_lon_pos_label_->setVisible(false);
      return;
    }
  
    int32_t precision = static_cast<int32_t>(std::ceil(std::max(0.0, std::log10(1.0 / scale))));

    QString text = ui_.fixedframe->currentText();
    if (text.isEmpty() || text == "/")
    {
      text = "fixed";
    }
    text += ": ";

    std::ostringstream x_ss;
    x_ss << std::fixed << std::setprecision(precision);
    x_ss << x;
    text += x_ss.str().c_str();
    
    text += ", ";
    
    std::ostringstream y_ss;
    y_ss << std::fixed << std::setprecision(precision);
    y_ss << y;
    text += y_ss.str().c_str();
    
    xy_pos_label_->setText(text);
    xy_pos_label_->setVisible(true);
    xy_pos_label_->update();
    
    transform_util::Transform transform;
    if (tf_manager_.SupportsTransform(
           transform_util::_wgs84_frame, 
           ui_.fixedframe->currentText().toStdString()) &&
        tf_manager_.GetTransform(
           transform_util::_wgs84_frame, 
           ui_.fixedframe->currentText().toStdString(),
           transform))
    {
      tf::Vector3 point(x, y, 0);
      point = transform * point;
      
      QString lat_lon_text = "lat/lon: ";
      
      double lat_scale = (1.0 / 111111.0) * scale;
      int32_t lat_precision = static_cast<int32_t>(std::ceil(std::max(0.0, std::log10(1.0 / lat_scale))));
      
      std::ostringstream lat_ss;
      lat_ss << std::fixed << std::setprecision(lat_precision);
      lat_ss << point.y();
      lat_lon_text += lat_ss.str().c_str();
      
      lat_lon_text += ", ";
    
      double lon_scale = (1.0 / (111111.0 * std::cos(point.y() * math_util::_deg_2_rad))) * scale;
      int32_t lon_precision = static_cast<int32_t>(std::ceil(std::max(0.0, std::log10(1.0 / lon_scale))));
      
      std::ostringstream lon_ss;
      lon_ss << std::fixed << std::setprecision(lon_precision);
      lon_ss << point.x();
      lat_lon_text += lon_ss.str().c_str();
      
      lat_lon_pos_label_->setText(lat_lon_text);
      lat_lon_pos_label_->setVisible(true);
      lat_lon_pos_label_->update();
    }
    else if (lat_lon_pos_label_->isVisible())
    {
      lat_lon_pos_label_->setVisible(false);
    }
  }
}

MapvizPluginPtr Mapviz::CreateNewDisplay(
    const std::string& name,
    const std::string& type,
    bool visible,
    bool collapsed)
{
  ConfigItem* config_item = new ConfigItem();

  config_item->SetName(name.c_str());

  ROS_INFO("creating: %s", type.c_str());
  MapvizPluginPtr plugin = loader_->createInstance(type.c_str());
  plugin->Initialize(tf_, canvas_);
  plugin->SetType(type.c_str());
  plugin->SetName(name);
  plugin->SetNode(*node_);
  plugin->SetVisible(visible);
  plugin->SetDrawOrder(ui_.configs->count());

  // Setup configure widget
  config_item->SetWidget(plugin->GetConfigWidget(this));

  QString pretty_type(type.c_str());
  pretty_type = pretty_type.split('/').last();
  config_item->SetType(pretty_type);
  QListWidgetItem* item = new QListWidgetItem();
  config_item->SetListItem(item);
  item->setSizeHint(config_item->sizeHint());
  connect(config_item, SIGNAL(UpdateSizeHint()), this, SLOT(UpdateSizeHints()));
  connect(config_item, SIGNAL(ToggledDraw(QListWidgetItem*, bool)), this, SLOT(ToggleShowPlugin(QListWidgetItem*, bool)));

  ui_.configs->addItem(item);
  ui_.configs->setItemWidget(item, config_item);

  // Add plugin to canvas
  plugin->SetTargetFrame(ui_.fixedframe->currentText().toStdString());
  plugin->SetUseLatestTransforms(ui_.uselatesttransforms->isChecked());
  plugins_[item] = plugin;
  canvas_->AddPlugin(plugin, -1);

  config_item->ToggleDraw(visible);

  if (collapsed)
    config_item->Hide();

  return plugin;
}

void Mapviz::ToggleShowPlugin(QListWidgetItem* item, bool visible)
{
  ROS_INFO("Toggle show plugin");

  if (plugins_.count(item) == 1)
  {
    plugins_[item]->SetVisible(visible);
  }
  canvas_->UpdateView();
}

void Mapviz::FixedFrameSelected(const QString& text)
{
  if (!updating_frames_)
  {
    ROS_INFO("Fixed frame selected: %s", text.toStdString().c_str());
    if (canvas_ != NULL)
    {
      canvas_->SetFixedFrame(text.toStdString().c_str());
    }
  }
}

void Mapviz::TargetFrameSelected(const QString& text)
{
  if (!updating_frames_)
  {
    ROS_INFO("Target frame selected: %s", text.toStdString().c_str());

    if (canvas_ != NULL)
    {
      canvas_->SetTargetFrame(text.toStdString().c_str());
    }
  }
}

void Mapviz::ToggleUseLatestTransforms(bool on)
{
  canvas_->ToggleUseLatestTransforms(on);
}

void Mapviz::ToggleFixOrientation(bool on)
{
  canvas_->ToggleFixOrientation(on);
}

void Mapviz::ToggleConfigPanel(bool on)
{
  if (on)
  {
    ui_.configdock->show();
  }
  else
  {
    ui_.configdock->hide();
  }

  AdjustWindowSize();
}

void Mapviz::ToggleStatusBar(bool on)
{
  ui_.statusbar->setVisible(on);

  AdjustWindowSize();
}

void Mapviz::ToggleCaptureTools(bool on)
{
  if (on)
  {
    ui_.actionShow_Status_Bar->setChecked(true);
  }
  
  screenshot_button_->setVisible(on);
  rec_button_->setVisible(on);
  stop_button_->setVisible(on);
  spacer1_->setVisible(on);
  spacer2_->setVisible(on);
  spacer3_->setVisible(on);
}

void Mapviz::ToggleRecord(bool on)
{
  stop_button_->setEnabled(true);

  if (on)
  {
    rec_button_->setIcon(QIcon(":/images/media-playback-pause.png"));
    
    if (!video_writer_)
    {
      // Lock the window size.
      AdjustWindowSize();
      
      canvas_->CaptureFrames(true);
    
      std::string posix_time = boost::posix_time::to_iso_string(ros::WallTime::now().toBoost());
      boost::replace_all(posix_time, ".", "_");    
      std::string filename = capture_directory_ + "/mapviz_" + posix_time + ".avi";
      boost::replace_all(filename, "~", getenv("HOME"));
      ROS_INFO("Writing video to: %s", filename.c_str());
    
      video_writer_ = boost::make_shared<cv::VideoWriter>(
        filename, 
        CV_FOURCC('M', 'J', 'P', 'G'), 
        30, 
        cv::Size(canvas_->width(), canvas_->height()));
        
      if (!video_writer_->isOpened())
      {
        ROS_ERROR("Failed to open video file for writing.");
        StopRecord();
        return;
      }
      
      std::string status = std::string("Recording video mapviz_") + posix_time + ".avi";
      ui_.statusbar->showMessage(QString::fromStdString(status));
      
      canvas_->updateGL();
    }
    
    record_timer_.start(1000.0 / 30.0);
  }
  else
  {
    rec_button_->setIcon(QIcon(":/images/media-record.png"));
    
    record_timer_.stop();
  }
}

void Mapviz::CaptureVideoFrame()
{
  std::vector<uint8_t> frame;
  if (canvas_->CopyCaptureBuffer(frame))
  {
    cv::Mat image(canvas_->height(), canvas_->width(), CV_8UC4, &frame[0]);
    
    cv::Mat video_frame;
    cvtColor(image, video_frame, CV_BGRA2BGR);
    
    cv::flip(video_frame, video_frame, 0);
    
    video_writer_->write(video_frame);
  }
  else
  {
    ROS_ERROR("Failed to get capture buffer");
  }
}

void Mapviz::StopRecord()
{
  rec_button_->setChecked(false);
  stop_button_->setEnabled(false);
  
  record_timer_.stop();
  video_writer_.reset();
  canvas_->CaptureFrames(false);
  
   ui_.statusbar->showMessage(QString(""));
  
  AdjustWindowSize();
}

void Mapviz::Screenshot()
{
  canvas_->CaptureFrame(true);
  
  std::vector<uint8_t> frame;
  if (canvas_->CopyCaptureBuffer(frame))
  {
    cv::Mat image(canvas_->height(), canvas_->width(), CV_8UC4, &frame[0]);
    cv::Mat screenshot;
    cvtColor(image, screenshot, CV_BGRA2BGR);
    
    cv::flip(screenshot, screenshot, 0);
    
    std::string posix_time = boost::posix_time::to_iso_string(ros::WallTime::now().toBoost());
    boost::replace_all(posix_time, ".", "_");    
    std::string filename = capture_directory_ + "/mapviz_" + posix_time + ".png";
    boost::replace_all(filename, "~", getenv("HOME"));
    ROS_INFO("Writing screenshot to: %s", filename.c_str());
    
    std::string status = std::string("Saved image mapviz_") + posix_time + ".png";
    ui_.statusbar->showMessage(QString::fromStdString(status));
    
    cv::imwrite(filename, screenshot);
  }
  else
  {
    ROS_ERROR("Failed to take screenshot.");
  }
}

void Mapviz::UpdateSizeHints()
{
  ROS_INFO("Updating size hints");
  for (int i = 0; i < ui_.configs->count(); i++)
  {
    ui_.configs->item(i)->setSizeHint(ui_.configs->itemWidget(ui_.configs->item(i))->sizeHint());
  }
}

void Mapviz::RemoveDisplay()
{
  ROS_INFO("Remove display ...");

  QListWidgetItem* item = ui_.configs->takeItem(ui_.configs->currentRow());

  if (item)
  {
    canvas_->RemovePlugin(plugins_[item]);
    plugins_[item] = MapvizPluginPtr();

    delete item;
  }
}

void Mapviz::ClearDisplays()
{
  while (ui_.configs->count() > 0)
  {
    ROS_INFO("Remove display ...");

    QListWidgetItem* item = ui_.configs->takeItem(0);

    canvas_->RemovePlugin(plugins_[item]);
    plugins_[item] = MapvizPluginPtr();

    delete item;
  }
}

void Mapviz::ReorderDisplays()
{
  ROS_INFO("Reorder displays");
  for (int i = 0; i < ui_.configs->count(); i++)
  {
    plugins_[ui_.configs->item(i)]->SetDrawOrder(i);
  }
  canvas_->ReorderDisplays();
}

void Mapviz::SelectBackgroundColor()
{
  QColorDialog dialog(background_, this);
  dialog.exec();

  if (dialog.result() == QDialog::Accepted)
  {
    background_ = dialog.selectedColor();
    ui_.bg_color->setStyleSheet("background: " + background_.name() + ";");
    canvas_->SetBackground(background_);
  }
}

void Mapviz::BlackBoxTimer(const ros::TimerEvent &event)
{
  if (blackbox_trigger_srv_.exists())
    ui_.blackbox_trigger_btn->setEnabled(true);
  else
    ui_.blackbox_trigger_btn->setEnabled(false);
}

void Mapviz::BlackBoxTrigger()
{
  std_srvs::Empty srv;
  if (blackbox_trigger_srv_.exists())
    blackbox_trigger_srv_.call(srv);
}

void Mapviz::SetCaptureDirectory()
{
  QFileDialog dialog(this, "Select Capture Directory");
  dialog.setFileMode(QFileDialog::DirectoryOnly);

  dialog.exec();

  if (dialog.result() == QDialog::Accepted && dialog.selectedFiles().count() == 1)
  {
    capture_directory_ = dialog.selectedFiles().first().toStdString();
  }
}
}

int main(int argc, char **argv)
{
  // Initialize QT
  QApplication app(argc, argv);

  // Initialize glut (for displaying text)
  glutInit(&argc, argv);

  mapviz::Mapviz mapviz(argc, argv);
  mapviz.show();

  return app.exec();
}
