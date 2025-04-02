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

#include <mapviz/mapviz.hpp>

// C++ standard libraries
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#if CV_MAJOR_VERSION > 2
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#endif

// QT libraries
#if QT_VERSION >= 0x050000
#include <QtWidgets/QApplication>
#else
#include <QtGui/QApplication>
#endif
#include <QFileDialog>
#include <QActionGroup>
#include <QColorDialog>
#include <QLabel>
#include <QMessageBox>
#include <QProcessEnvironment>
#include <QFileInfo>
#include <QListWidgetItem>
#include <QMutexLocker>

// Other Project libraries
#include <swri_math_util/constants.h>
#include <swri_transform_util/frames.h>

#include <mapviz/config_item.h>
#include <QtGui/QtGui>

#include <image_transport/image_transport.hpp>

// YAML libraries
#include <yaml-cpp/yaml.h>

// Boost libraries
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace mapviz
{
const QString Mapviz::ROS_WORKSPACE_VAR = "ROS_WORKSPACE";
const QString Mapviz::MAPVIZ_CONFIG_FILE = "/.mapviz_config";
const char Mapviz::IMAGE_TRANSPORT_PARAM[] = "image_transport";

Mapviz::Mapviz(bool is_standalone, int argc, char** argv, QWidget *parent, Qt::WindowFlags flags) :
    QMainWindow(parent, flags),
    xy_pos_label_(new QLabel("fixed: 0.0,0.0")),
    lat_lon_pos_label_(new QLabel("lat/lon: 0.0,0.0")),
    argc_(argc),
    argv_(argv),
    is_standalone_(is_standalone),
    initialized_(false),
    force_720p_(false),
    force_480p_(false),
    resizable_(true),
    background_(Qt::gray),
    capture_directory_("~"),
    vid_writer_(nullptr),
    updating_frames_(false),
    node_(nullptr),
    canvas_(nullptr)
{
  // Multiple users could be using mapviz, so its name needs to be anonymous,
  // but ROS 2 Dashing doesn't have a way to set that through node options;
  // we manually do it the same way that ros::init did in ROS 1
  std::stringstream name;
  name << "mapviz";
  char buf[200];
  std::snprintf(buf, sizeof(buf), "_%llu", (unsigned long long)rclcpp::Clock().now().nanoseconds());
  name << buf;
  node_ = std::make_shared<rclcpp::Node>(name.str());

  QString default_path = GetDefaultConfigPath();
  node_->declare_parameter("config", default_path.toStdString());
  node_->declare_parameter("auto_save_backup", true);
  node_->declare_parameter("print_profile_data", false);
  node_->declare_parameter(IMAGE_TRANSPORT_PARAM, "raw");

  ui_.setupUi(this);

  xy_pos_label_->setVisible(false);
  lat_lon_pos_label_->setVisible(false);

  ui_.statusbar->addPermanentWidget(xy_pos_label_);
  ui_.statusbar->addPermanentWidget(lat_lon_pos_label_);

  spacer1_ = new QWidget(ui_.statusbar);
  spacer1_-> setMaximumSize(22, 22);
  spacer1_-> setMinimumSize(22, 22);
  ui_.statusbar->addPermanentWidget(spacer1_);

  screenshot_button_ = new QPushButton();
  screenshot_button_->setMinimumSize(22, 22);
  screenshot_button_->setMaximumSize(22, 22);
  screenshot_button_->setIcon(QIcon(":/images/image-x-generic.png"));
  screenshot_button_->setFlat(true);
  screenshot_button_->setToolTip("Capture screenshot of display canvas");
  ui_.statusbar->addPermanentWidget(screenshot_button_);

  spacer2_ = new QWidget(ui_.statusbar);
  spacer2_->setMaximumSize(22, 22);
  spacer2_->setMinimumSize(22, 22);
  ui_.statusbar->addPermanentWidget(spacer2_);

  rec_button_ = new QPushButton();
  rec_button_->setMinimumSize(22, 22);
  rec_button_->setMaximumSize(22, 22);
  rec_button_->setIcon(QIcon(":/images/media-record.png"));
  rec_button_->setCheckable(true);
  rec_button_->setFlat(true);
  rec_button_->setToolTip("Start recording video of display canvas");
  ui_.statusbar->addPermanentWidget(rec_button_);

  stop_button_ = new QPushButton();
  stop_button_->setMinimumSize(22, 22);
  stop_button_->setMaximumSize(22, 22);
  stop_button_->setIcon(QIcon(":/images/media-playback-stop.png"));
  stop_button_->setToolTip("Stop recording video of display canvas");
  stop_button_->setEnabled(false);
  stop_button_->setFlat(true);
  ui_.statusbar->addPermanentWidget(stop_button_);

  spacer3_ = new QWidget(ui_.statusbar);
  spacer3_->setMaximumSize(22, 22);
  spacer3_->setMinimumSize(22, 22);
  ui_.statusbar->addPermanentWidget(spacer3_);

  recenter_button_ = new QPushButton();
  recenter_button_->setMinimumSize(22, 22);
  recenter_button_->setMaximumSize(22, 22);
  recenter_button_->setIcon(QIcon(":/images/arrow_in.png"));
  recenter_button_->setToolTip("Reset the viewport to the default location and zoom level");
  recenter_button_->setFlat(true);
  ui_.statusbar->addPermanentWidget(recenter_button_);

  ui_.statusbar->setVisible(true);

  QActionGroup* group = new QActionGroup(this);

  ui_.actionForce_720p->setActionGroup(group);
  ui_.actionForce_480p->setActionGroup(group);
  ui_.actionResizable->setActionGroup(group);

  ui_.targetframe->addItem("<none>");

  canvas_ = new MapCanvas(this);
  setCentralWidget(canvas_);

  connect(
    canvas_,
    SIGNAL(Hover(double, double, double)),
    this,
    SLOT(Hover(double, double, double)));
  connect(ui_.configs, SIGNAL(ItemsMoved()), this, SLOT(ReorderDisplays()));
  connect(ui_.actionExit, SIGNAL(triggered()), this, SLOT(close()));
  connect(ui_.actionClear, SIGNAL(triggered()), this, SLOT(ClearConfig()));
  connect(
    ui_.bg_color,
    SIGNAL(colorEdited(const QColor &)),
    this,
    SLOT(SelectBackgroundColor(const QColor &)));

  connect(recenter_button_, SIGNAL(clicked()), this, SLOT(Recenter()));
  connect(rec_button_, SIGNAL(toggled(bool)), this, SLOT(ToggleRecord(bool)));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(StopRecord()));
  connect(screenshot_button_, SIGNAL(clicked()), this, SLOT(Screenshot()));
  connect(ui_.actionClear_History, SIGNAL(triggered()), this, SLOT(ClearHistory()));

  // Use a separate thread for writing video files so that it won't cause
  // lag on the main thread.
  // It's ok for the video writer to be a pointer that we instantiate here and
  // then forget about; the worker thread will delete it when the thread exits.
  vid_writer_ = new VideoWriter();
  vid_writer_->moveToThread(&video_thread_);
  connect(&video_thread_, SIGNAL(finished()), vid_writer_, SLOT(deleteLater()));
  connect(this, SIGNAL(FrameGrabbed(QImage)), vid_writer_, SLOT(processFrame(QImage)));
  video_thread_.start();

  image_transport_menu_ = new QMenu("Default Image Transport", ui_.menu_View);
  ui_.menu_View->addMenu(image_transport_menu_);

  connect(image_transport_menu_, SIGNAL(aboutToShow()), this, SLOT(UpdateImageTransportMenu()));

  ui_.bg_color->setColor(background_);
  canvas_->SetBackground(background_);

  // Keyboard shortcuts for the main window
  QShortcut * remove_display_shortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_X), this);
  connect(remove_display_shortcut, SIGNAL(activated()), this, SLOT(RemoveDisplay()));
  QShortcut * rename_display_shortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_R), this);
  connect(rename_display_shortcut, SIGNAL(activated()), this, SLOT(RenameDisplay()));
  QShortcut * add_display_shortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_N), this);
  connect(add_display_shortcut, SIGNAL(activated()), this, SLOT(SelectNewDisplay()));
}

Mapviz::~Mapviz()
{
  video_thread_.quit();
  video_thread_.wait();
}

rclcpp::Node::SharedPtr Mapviz::GetNode()
{
  return node_;
}

void Mapviz::showEvent(QShowEvent* event)
{
  Initialize();
}

void Mapviz::closeEvent(QCloseEvent* event)
{
  AutoSave();

  for (auto& display : plugins_) {
    MapvizPluginPtr plugin = display.second;
    canvas_->RemovePlugin(plugin);
  }

  plugins_.clear();
}

void Mapviz::Initialize()
{
  if (!initialized_) {
    if (is_standalone_) {
      spin_timer_.start(30);
      connect(&spin_timer_, SIGNAL(timeout()), this, SLOT(SpinOnce()));
    }

    // Create a sub-menu that lists all available Image Transports
    image_transport::ImageTransport it(node_);
    std::vector<std::string> transports = it.getLoadableTransports();
    QActionGroup* group = new QActionGroup(image_transport_menu_);
    for (const auto& iter : transports)
    {
      QString transport = QString::fromStdString(iter).replace(
          QString::fromStdString(IMAGE_TRANSPORT_PARAM) + "/", "");
      QAction* action = image_transport_menu_->addAction(transport);
      action->setCheckable(true);
      group->addAction(action);
    }

    connect(group, SIGNAL(triggered(QAction*)), this, SLOT(SetImageTransport(QAction*)));

    rclcpp::QoS dynamic_tf_qos(100);
    dynamic_tf_qos.best_effort();
    dynamic_tf_qos.durability_volatile();
    tf_buf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_buf_->setUsingDedicatedThread(true);
    tf_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buf_,
      node_,
      false,
      dynamic_tf_qos);
    
    tf_manager_ = std::make_shared<swri_transform_util::TransformManager>(node_);
    try
    {
      tf_manager_->Initialize(tf_buf_);
    }
    catch (...)
    {
      RCLCPP_ERROR(node_->get_logger(), "Error initializing tf_manager");
    }

    loader_ = new pluginlib::ClassLoader<MapvizPlugin>(
        "mapviz", "mapviz::MapvizPlugin");

    std::vector<std::string> plugins = loader_->getDeclaredClasses();
    for (const auto& plugin : plugins) {
      RCLCPP_INFO(node_->get_logger(), "Found mapviz plugin: %s", plugin.c_str());
    }

    canvas_->InitializeTf(tf_buf_);
    canvas_->SetFixedFrame(ui_.fixedframe->currentText().toStdString());
    canvas_->SetTargetFrame(ui_.targetframe->currentText().toStdString());

    add_display_srv_ = node_->create_service<mapviz_interfaces::srv::AddMapvizDisplay>(
                                              "add_mapviz_display",
                                              std::bind(&Mapviz::AddDisplay,
                                                  this,
                                                  std::placeholders::_1,
                                                  std::placeholders::_2));

    QString default_path = GetDefaultConfigPath();

    std::string config;
    node_->get_parameter_or("config", config, default_path.toStdString());

    bool auto_save;
    node_->get_parameter_or("auto_save_backup", auto_save, true);

    Open(config);

    UpdateFrames();
    frame_timer_.start(1000);
    connect(&frame_timer_, SIGNAL(timeout()), this, SLOT(UpdateFrames()));

    if (auto_save) {
      save_timer_.start(10000);
      connect(&save_timer_, SIGNAL(timeout()), this, SLOT(AutoSave()));
    }

    connect(&record_timer_, SIGNAL(timeout()), this, SLOT(CaptureVideoFrame()));

    bool print_profile_data;
    node_->get_parameter_or("print_profile_data", print_profile_data, false);
    if (print_profile_data) {
      profile_timer_.start(2000);
      connect(&profile_timer_, SIGNAL(timeout()), this, SLOT(HandleProfileTimer()));
    }

    setFocus();   // Set the main window as focused object,
                  // prevent other fields from obtaining focus at startup

    initialized_ = true;
  }
}

QString Mapviz::GetDefaultConfigPath()
{
  QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  QString default_path = QDir::homePath();
  if (env.contains(ROS_WORKSPACE_VAR)) {
    // If the ROS_WORKSPACE environment variable is defined, try to read our
    // config file out of that.  If we can't read it, fall back to trying to
    // read one from the user's home directory.
    QString ws_path = env.value(ROS_WORKSPACE_VAR, default_path);
    if (QFileInfo(ws_path + MAPVIZ_CONFIG_FILE).isReadable()) {
      default_path = ws_path;
    } else {
      RCLCPP_WARN(
          node_->get_logger(),
          "Could not load config file from ROS_WORKSPACE at %s; trying home directory...",
          ws_path.toStdString().c_str());
    }
  }
  default_path += MAPVIZ_CONFIG_FILE;

  return default_path;
}

void Mapviz::SpinOnce()
{
  if (rclcpp::ok()) {
    meas_spin_.start();
    rclcpp::spin_some(node_);
    meas_spin_.stop();
  } else {
    QApplication::exit();
  }
}

void Mapviz::UpdateFrames()
{
  std::vector<std::string> frames;
  tf_buf_->_getFrameStrings(frames);
  std::sort(frames.begin(), frames.end());

  if (
    ui_.fixedframe->count() >= 0 &&
    static_cast<size_t>(ui_.fixedframe->count()) == frames.size())
  {
    bool changed = false;
    for (size_t i = 0; i < frames.size(); i++) {
      if (frames[i] != ui_.fixedframe->itemText(i).toStdString()) {
        changed = true;
      }
    }

    if (!changed) {
      return;
    }
  }

  updating_frames_ = true;

  std::string current_fixed = ui_.fixedframe->currentText().toStdString();

  ui_.fixedframe->clear();
  for (const auto& frame : frames) {
    ui_.fixedframe->addItem(frame.c_str());
  }

  if (!current_fixed.empty()) {
    int index = ui_.fixedframe->findText(current_fixed.c_str());
    if (index < 0) {
      ui_.fixedframe->addItem(current_fixed.c_str());
    }

    index = ui_.fixedframe->findText(current_fixed.c_str());
    ui_.fixedframe->setCurrentIndex(index);
  }

  std::string current_target = ui_.targetframe->currentText().toStdString();

  ui_.targetframe->clear();
  ui_.targetframe->addItem("<none>");
  for (const auto& frame : frames) {
    ui_.targetframe->addItem(frame.c_str());
  }

  if (!current_target.empty()) {
    int index = ui_.targetframe->findText(current_target.c_str());
    if (index < 0) {
      ui_.targetframe->addItem(current_target.c_str());
    }

    index = ui_.targetframe->findText(current_target.c_str());
    ui_.targetframe->setCurrentIndex(index);
  }

  updating_frames_ = false;

  if (current_target != ui_.targetframe->currentText().toStdString()) {
    TargetFrameSelected(ui_.targetframe->currentText());
  }

  if (current_fixed != ui_.fixedframe->currentText().toStdString()) {
    FixedFrameSelected(ui_.fixedframe->currentText());
  }
}

void Mapviz::Force720p(bool on)
{
  if (force_720p_ != on) {
    force_720p_ = on;

    if (force_720p_) {
      force_480p_ = false;
      resizable_ = false;
    }

    AdjustWindowSize();
  }
}

void Mapviz::Force480p(bool on)
{
  if (force_480p_ != on) {
    force_480p_ = on;

    if (force_480p_) {
      force_720p_ = false;
      resizable_ = false;
    }

    AdjustWindowSize();
  }
}

void Mapviz::SetResizable(bool on)
{
  if (resizable_ != on) {
    resizable_ = on;

    if (resizable_) {
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

  if (force_720p_) {
    canvas_->setMinimumSize(1280, 720);
    canvas_->setMaximumSize(1280, 720);
    canvas_->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
    adjustSize();
    this->setMaximumSize(this->sizeHint());
    this->setMinimumSize(this->sizeHint());
    setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  } else if (force_480p_) {
    canvas_->setMinimumSize(640, 480);
    canvas_->setMaximumSize(640, 480);
    canvas_->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
    adjustSize();
    this->setMaximumSize(this->sizeHint());
    this->setMinimumSize(this->sizeHint());
    setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  } else if (stop_button_->isEnabled()) {
    canvas_->setMinimumSize(canvas_->width(), canvas_->height());
    canvas_->setMaximumSize(canvas_->width(), canvas_->height());
    canvas_->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
    adjustSize();
    this->setMaximumSize(this->sizeHint());
    this->setMinimumSize(this->sizeHint());
    setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  } else {
    canvas_->setMinimumSize(100, 100);
    canvas_->setMaximumSize(10000, 10000);
  }
}

void Mapviz::Open(const std::string& filename)
{
  RCLCPP_INFO(node_->get_logger(), "Loading configuration from %s", filename.c_str());

  std::string title;
  size_t last_slash = filename.find_last_of('/');
  if (last_slash != std::string::npos && last_slash != filename.size() - 1) {
    title = filename.substr(last_slash + 1) + " (" +
            filename.substr(0, last_slash + 1) + ")";
  } else {
    title = filename;
  }

  title += " - mapviz";
  setWindowTitle(QString::fromStdString(title));

  YAML::Node doc = YAML::LoadFile(filename);
  if (!doc) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load file: %s", filename.c_str());
    return;
  }

  std::vector<std::string> failed_plugins;

  try
  {
    boost::filesystem::path filepath(filename);
    std::string config_path = filepath.parent_path().string();

    ClearDisplays();

    if (doc["capture_directory"]) {
      capture_directory_ = doc["capture_directory"].as<std::string>();
    }

    if (doc["fixed_frame"]) {
      std::string fixed_frame = doc["fixed_frame"].as<std::string>();
      ui_.fixedframe->setEditText(fixed_frame.c_str());
    }

    if (doc["target_frame"]) {
      std::string target_frame = doc["target_frame"].as<std::string>();
      ui_.targetframe->setEditText(target_frame.c_str());
    }

    if (doc["fix_orientation"]) {
      bool fix_orientation = doc["fix_orientation"].as<bool>();
      ui_.actionFix_Orientation->setChecked(fix_orientation);
    }

    if (doc["rotate_90"]) {
      bool rotate_90 = doc["rotate_90"].as<bool>();
      ui_.actionRotate_90->setChecked(rotate_90);
    }

    if (doc["enable_antialiasing"]) {
      bool enable_antialiasing = doc["enable_antialiasing"].as<bool>();
      ui_.actionEnable_Antialiasing->setChecked(enable_antialiasing);
    }

    if (doc["show_displays"]) {
      bool show_displays = doc["show_displays"].as<bool>();
      ui_.actionConfig_Dock->setChecked(show_displays);
    }

    if (doc["show_capture_tools"]) {
      bool show_capture_tools = doc["show_capture_tools"].as<bool>();
      ui_.actionShow_Capture_Tools->setChecked(show_capture_tools);
    }

    if (doc["show_status_bar"]) {
      bool show_status_bar = doc["show_status_bar"].as<bool>();
      ui_.actionShow_Status_Bar->setChecked(show_status_bar);
    }

    if (doc["show_capture_tools"]) {
      bool show_capture_tools = doc["show_capture_tools"].as<bool>();
      ui_.actionShow_Capture_Tools->setChecked(show_capture_tools);
    }

    if (doc["window_width"]) {
      int window_width = doc["window_width"].as<int>();
      resize(window_width, height());
    }

    if (doc["window_height"]) {
      int window_height = doc["window_height"].as<int>();
      resize(width(), window_height);
    }

    if (doc["view_scale"]) {
      float scale = doc["view_scale"].as<float>();
      canvas_->SetViewScale(scale);
    }

    if (doc["offset_x"]) {
      float x = doc["offset_x"].as<float>();
      canvas_->SetOffsetX(x);
    }

    if (doc["offset_y"]) {
      float y = doc["offset_y"].as<float>();
      canvas_->SetOffsetY(y);
    }

    if (doc["force_720p"]) {
      bool force_720p = doc["force_720p"].as<bool>();

      if (force_720p) {
        ui_.actionForce_720p->setChecked(true);
      }
    }

    if (doc["force_480p"]) {
      bool force_480p = doc["force_480p"].as<bool>();

      if (force_480p) {
        ui_.actionForce_480p->setChecked(true);
      }
    }

    if (doc[IMAGE_TRANSPORT_PARAM]) {
      std::string image_transport = doc[IMAGE_TRANSPORT_PARAM].as<std::string>();

      node_->set_parameter({IMAGE_TRANSPORT_PARAM, image_transport});
    }

    bool use_latest_transforms = true;
    if (doc["use_latest_transforms"]) {
      use_latest_transforms = doc["use_latest_transforms"].as<bool>();
    }
    ui_.uselatesttransforms->setChecked(use_latest_transforms);
    canvas_->ToggleUseLatestTransforms(use_latest_transforms);

    if (doc["background"]) {
      std::string color = doc["background"].as<std::string>();
      background_ = QColor(color.c_str());
      ui_.bg_color->setColor(background_);
      canvas_->SetBackground(background_);
    }

    if (doc["displays"]) {
      const YAML::Node& displays = doc["displays"];
      for (const auto& display : displays) {
        std::string type = display["type"].as<std::string>();
        std::string name = display["name"].as<std::string>();

        const YAML::Node& config = display["config"];

        bool visible = config["visible"].as<bool>();

        bool collapsed = config["collapsed"].as<bool>();

        try
        {
          MapvizPluginPtr plugin =
              CreateNewDisplay(name, type, visible, collapsed);
          plugin->LoadConfig(config, config_path);
          plugin->DrawIcon();
        }
        catch (const pluginlib::LibraryLoadException& e)
        {
          failed_plugins.push_back(type);
          RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
        }
        catch (const YAML::ParserException& e)
        {
          failed_plugins.push_back(type);
          RCLCPP_ERROR(node_->get_logger(), "%s (%s) plugin failed with YAML parser error: %s",
              type.c_str(), name.c_str(), e.what());
          // Try to continue parsing through plugins
        }
        catch (const YAML::Exception& e)
        {
          failed_plugins.push_back(type);
          RCLCPP_ERROR(node_->get_logger(), "%s (%s) plugin failed with YAML error: %s",
              type.c_str(), name.c_str(), e.what());
          // Try to continue parsing through plugins
        }
        catch (const rclcpp::exceptions::RCLError& e)
        {
          RCLCPP_ERROR(node_->get_logger(), "%s (%s) plugin failed with RCLCPP error: %s",
              type.c_str(), name.c_str(), e.what());
        }
      }
    }
  }
  catch (const YAML::ParserException& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
    return;
  }
  catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
    return;
  }

  if (!failed_plugins.empty()) {
    std::stringstream message;
    message << "The following plugin(s) failed to load:" << std::endl;
    std::string failures = boost::algorithm::join(failed_plugins, "\n");
    message << failures << std::endl << std::endl << "Check the ROS log for more details.";

    QMessageBox::warning(this, "Failed to load plugins", QString::fromStdString(message.str()));
  }
}

void Mapviz::Save(const std::string& filename)
{
  std::ofstream fout(filename.c_str());
  if (fout.fail()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open file: %s", filename.c_str());
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
  out << YAML::Key << "rotate_90" << YAML::Value << ui_.actionRotate_90->isChecked();
  out << YAML::Key
      << "enable_antialiasing"
      << YAML::Value
      << ui_.actionEnable_Antialiasing->isChecked();
  out << YAML::Key
      << "show_displays"
      << YAML::Value
      << ui_.actionConfig_Dock->isChecked();
  out << YAML::Key << "show_status_bar" << YAML::Value << ui_.actionShow_Status_Bar->isChecked();
  out << YAML::Key
      << "show_capture_tools"
      << YAML::Value
      << ui_.actionShow_Capture_Tools->isChecked();
  out << YAML::Key << "window_width" << YAML::Value << width();
  out << YAML::Key << "window_height" << YAML::Value << height();
  out << YAML::Key << "view_scale" << YAML::Value << canvas_->ViewScale();
  out << YAML::Key << "offset_x" << YAML::Value << canvas_->OffsetX();
  out << YAML::Key << "offset_y" << YAML::Value << canvas_->OffsetY();
  out << YAML::Key
      << "use_latest_transforms"
      << YAML::Value
      << ui_.uselatesttransforms->isChecked();
  out << YAML::Key << "background" << YAML::Value << background_.name().toStdString();
  std::string image_transport;
  if (node_->get_parameter(IMAGE_TRANSPORT_PARAM, image_transport)) {
    out << YAML::Key << IMAGE_TRANSPORT_PARAM << YAML::Value << image_transport;
  }

  if (force_720p_) {
    out << YAML::Key << "force_720p" << YAML::Value << force_720p_;
  }

  if (force_480p_) {
    out << YAML::Key << "force_480p" << YAML::Value << force_480p_;
  }

  if (ui_.configs->count() > 0) {
    out << YAML::Key << "displays"<< YAML::Value << YAML::BeginSeq;

    for (int i = 0; i < ui_.configs->count(); i++) {
      out << YAML::BeginMap;
      out << YAML::Key << "type" << YAML::Value << plugins_[ui_.configs->item(i)]->Type();
      out << YAML::Key
          << "name"
          << YAML::Value
          << (dynamic_cast<ConfigItem*>(ui_.configs->itemWidget(ui_.configs->item(i))))
                ->Name().toStdString();
      out << YAML::Key << "config" << YAML::Value;
      out << YAML::BeginMap;

      out << YAML::Key << "visible" << YAML::Value << plugins_[ui_.configs->item(i)]->Visible();
      out << YAML::Key
          << "collapsed"
          << YAML::Value
          << (dynamic_cast<ConfigItem*>(ui_.configs->itemWidget(ui_.configs->item(i))))->Collapsed();

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
  QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  QString default_path = QDir::homePath();

  if (env.contains(ROS_WORKSPACE_VAR)) {
    // Try to save our config in the ROS_WORKSPACE directory, but if we can't write
    // to that -- probably because it is read-only -- try to use the home directory
    // instead.
    QString ws_path = env.value(ROS_WORKSPACE_VAR, default_path);
    QString ws_file = ws_path + MAPVIZ_CONFIG_FILE;
    QFileInfo file_info(ws_file);
    QFileInfo dir_info(ws_path);
    if (
      (!file_info.exists() && dir_info.isWritable()) ||
      file_info.isWritable())
    {
      // Note that FileInfo::isWritable will return false if a file does not exist, so
      // we need to check both if the target file is writable and if the target dir is
      // writable if the file doesn't exist.
      default_path = ws_path;
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Could not write config file to %s. Trying home directory.",
                  (ws_path + MAPVIZ_CONFIG_FILE).toStdString().c_str());
    }
  }
  default_path += MAPVIZ_CONFIG_FILE;


  Save(default_path.toStdString());
}

void Mapviz::OpenConfig()
{
  QFileDialog dialog(this, "Select Config File");
  dialog.setFileMode(QFileDialog::ExistingFile);
  dialog.setNameFilter(tr("Mapviz Config Files (*.mvc)"));

  dialog.exec();

  if (dialog.result() == QDialog::Accepted && dialog.selectedFiles().count() == 1) {
    std::string path = dialog.selectedFiles().first().toStdString();
    Open(path);
  }
}

void Mapviz::ClearConfig()
{
  ClearDisplays();
}

void Mapviz::SaveConfig()
{
  QFileDialog dialog(this, "Save Config File");
  dialog.setFileMode(QFileDialog::AnyFile);
  dialog.setAcceptMode(QFileDialog::AcceptSave);
  dialog.setNameFilter(tr("Mapviz Config Files (*.mvc)"));
  dialog.setDefaultSuffix("mvc");

  dialog.exec();

  if (dialog.result() == QDialog::Accepted && dialog.selectedFiles().count() == 1) {
    std::string path = dialog.selectedFiles().first().toStdString();

    std::string title;
    size_t last_slash = path.find_last_of('/');
    if (last_slash != std::string::npos && last_slash != path.size() - 1) {
      title = path.substr(last_slash + 1) + " (" +
              path.substr(0, last_slash + 1) + ")";
    } else {
      title = path;
    }
    title += " - mapviz";
    setWindowTitle(QString::fromStdString(title));
    Save(path);
  }
}

void Mapviz::ClearHistory()
{
  RCLCPP_DEBUG(node_->get_logger(), "Mapviz::ClearHistory()");
  for (auto& plugin : plugins_) {
    plugin.second->ClearHistory();
  }
}

void Mapviz::SelectNewDisplay()
{
  RCLCPP_INFO(rclcpp::get_logger("mapviz"), "Select new display ...");
  QDialog dialog;
  Ui::pluginselect ui;
  ui.setupUi(&dialog);

  std::vector<std::string> plugins = loader_->getDeclaredClasses();
  std::map<std::string, std::string> plugin_types;
  for (const auto& plugin : plugins) {
    QString type(plugin.c_str());
    type = type.split('/').last();
    ui.displaylist->addItem(type);
    plugin_types[type.toStdString()] = plugin;
  }
  ui.displaylist->setCurrentRow(0);

  dialog.exec();

  if (dialog.result() == QDialog::Accepted) {
    std::string type_name = ui.displaylist->selectedItems().first()->text().toStdString();
    std::string type = plugin_types[type_name];
    std::string name = "new display";
    try
    {
      CreateNewDisplay(name, type, true, false);
    }
    catch (const pluginlib::LibraryLoadException& e)
    {
      std::stringstream message;
      message << "Unable to load " << type << "." << std::endl
              << "Check the ROS log for more details.";
      QMessageBox::warning(this, "Plugin failed to load", QString::fromStdString(message.str()));
      RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
    }
  }
}

void Mapviz::AddDisplay(
      const mapviz_interfaces::srv::AddMapvizDisplay::Request::SharedPtr req,
      mapviz_interfaces::srv::AddMapvizDisplay::Response::SharedPtr resp)
{
  std::map<std::string, std::string> properties;
  for (auto& property : req->properties) {
    properties[property.key] = property.value;
  }

  YAML::Node config;
  for (auto& property_pair : properties) {
    config[property_pair.first] = property_pair.second;
  }

  if (!config) {
    // ROS_ERROR("Failed to parse properties into YAML.");
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse properties into YAML.");
    resp->success = false;
    throw std::runtime_error("Failed to parse properties into YAML.");
  }

  for (auto& display : plugins_) {
    MapvizPluginPtr plugin = display.second;
    if (!plugin) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid plugin ptr.");
      continue;
    }

    if (plugin->Name() == req->name && plugin->Type() == req->type) {
      plugin->LoadConfig(config, "");
      plugin->SetVisible(req->visible);

      if (req->draw_order > 0) {
        display.first->setData(Qt::UserRole, QVariant(req->draw_order - 1.1));
        ui_.configs->sortItems();

        ReorderDisplays();
      } else if (req->draw_order < 0) {
        display.first->setData(
          Qt::UserRole, QVariant(ui_.configs->count() + req->draw_order + 0.1));
        ui_.configs->sortItems();

        ReorderDisplays();
      }

      resp->success = true;

      return;
    }
  }

  try
  {
    MapvizPluginPtr plugin =
      CreateNewDisplay(req->name, req->type, req->visible, false, req->draw_order);
    plugin->LoadConfig(config, "");
    plugin->DrawIcon();
    resp->success = true;
  }
  catch (const pluginlib::LibraryLoadException& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
    resp->success = false;
    resp->message = "Failed to load display plug-in.";
  }
}

void Mapviz::Hover(double x, double y, double scale)
{
  if (ui_.statusbar->isVisible()) {
    if (scale == 0) {
      xy_pos_label_->setVisible(false);
      lat_lon_pos_label_->setVisible(false);
      return;
    }

    int32_t precision = static_cast<int32_t>(std::ceil(std::max(0.0, std::log10(1.0 / scale))));

    QString text = ui_.fixedframe->currentText();
    if (text.isEmpty() || text == "/") {
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

    swri_transform_util::Transform transform;
    std::string fixed_frame = ui_.fixedframe->currentText().toStdString();
    if
    (
      fixed_frame.length() > 0 &&
      tf_manager_->SupportsTransform(
        swri_transform_util::_wgs84_frame,
        fixed_frame) &&
      tf_manager_->GetTransform(
        swri_transform_util::_wgs84_frame,
        fixed_frame,
        transform))
    {
      tf2::Vector3 point(x, y, 0);
      point = transform * point;

      QString lat_lon_text = "lat/lon: ";

      double lat_scale = (1.0 / 111111.0) * scale;
      int32_t lat_precision = static_cast<int32_t>(
        std::ceil(std::max(0.0, std::log10(1.0 / lat_scale))));

      std::ostringstream lat_ss;
      lat_ss << std::fixed << std::setprecision(lat_precision);
      lat_ss << point.y();
      lat_lon_text += lat_ss.str().c_str();

      lat_lon_text += ", ";

      double lon_scale = (1.0
        / (111111.0 * std::cos(point.y() * swri_math_util::_deg_2_rad))) * scale;
      int32_t lon_precision = static_cast<int32_t>(
        std::ceil(std::max(0.0, std::log10(1.0 / lon_scale))));

      std::ostringstream lon_ss;
      lon_ss << std::fixed << std::setprecision(lon_precision);
      lon_ss << point.x();
      lat_lon_text += lon_ss.str().c_str();

      lat_lon_pos_label_->setText(lat_lon_text);
      lat_lon_pos_label_->setVisible(true);
      lat_lon_pos_label_->update();
    } else if (lat_lon_pos_label_->isVisible()) {
      lat_lon_pos_label_->setVisible(false);
    }
  }
}

MapvizPluginPtr Mapviz::CreateNewDisplay(
    const std::string& name,
    const std::string& type,
    bool visible,
    bool collapsed,
    int draw_order)
{
  auto* config_item = new ConfigItem();

  config_item->SetName(name.c_str());

  std::string real_type = type;
  if (real_type == "mapviz_plugins/mutlires_image") {
    // The "multires_image" plugin was originally accidentally named "mutlires_image".
    // Loading a mapviz config file that still has the old name would normally cause it
    // to crash, so this will check for and correct it.
    real_type = "mapviz_plugins/multires_image";
  }

  RCLCPP_INFO(node_->get_logger(), "creating: %s", real_type.c_str());
  MapvizPluginPtr plugin = loader_->createSharedInstance(real_type);

  // Setup configure widget
  config_item->SetWidget(plugin->GetConfigWidget(this));
  plugin->SetIcon(config_item->ui_.icon);
  plugin->SetType(real_type);
  plugin->SetName(name);
  plugin->SetNode(*node_);
  plugin->Initialize(tf_buf_, tf_, tf_manager_, canvas_);
  plugin->SetVisible(visible);

  if (draw_order == 0) {
    plugin->SetDrawOrder(ui_.configs->count());
  } else if (draw_order > 0) {
    plugin->SetDrawOrder(std::min(ui_.configs->count(), draw_order - 1));
  } else if (draw_order < 0) {
    plugin->SetDrawOrder(std::max(0, ui_.configs->count() + draw_order + 1));
  }

  QString pretty_type(real_type.c_str());
  pretty_type = pretty_type.split('/').last();
  config_item->SetType(pretty_type);
  QListWidgetItem* item = new PluginConfigListItem();
  config_item->SetListItem(item);
  item->setSizeHint(config_item->sizeHint());
  connect(config_item, SIGNAL(UpdateSizeHint()), this, SLOT(UpdateSizeHints()));
  connect(
    config_item,
    SIGNAL(ToggledDraw(QListWidgetItem*, bool)),
    this,
    SLOT(ToggleShowPlugin(QListWidgetItem*, bool)));
  connect(
    config_item,
    SIGNAL(RemoveRequest(QListWidgetItem*)),
    this,
    SLOT(RemoveDisplay(QListWidgetItem*)));
  connect(plugin.get(), SIGNAL(VisibleChanged(bool)), config_item, SLOT(ToggleDraw(bool)));
  connect(plugin.get(), SIGNAL(SizeChanged()), this, SLOT(UpdateSizeHints()));

  if (real_type == "mapviz_plugins/image") {
    // This is a little kludgey because we're relying on hard-coding a
    // plugin type here... feel free to suggest a better way.
    // If the default image transport has changed, we want to notify all of our
    // image plugins of it so that they will resubscribe appropriately.
    connect(this, SIGNAL(ImageTransportChanged()),
            plugin.get(), SLOT(Resubscribe()));
  }

  if (draw_order == 0) {
    ui_.configs->addItem(item);
  } else {
    ui_.configs->insertItem(plugin->DrawOrder(), item);
  }

  ui_.configs->setItemWidget(item, config_item);
  ui_.configs->UpdateIndices();

  // Add plugin to canvas
  plugin->SetTargetFrame(ui_.fixedframe->currentText().toStdString());
  plugin->SetUseLatestTransforms(ui_.uselatesttransforms->isChecked());
  plugins_[item] = plugin;
  canvas_->AddPlugin(plugin, -1);

  config_item->ToggleDraw(visible);

  if (collapsed) {
    config_item->Hide();
  }

  ReorderDisplays();

  return plugin;
}

void Mapviz::ToggleShowPlugin(QListWidgetItem* item, bool visible)
{
  RCLCPP_INFO(node_->get_logger(), "Toggle show plugin");

  if (plugins_.count(item) == 1) {
    plugins_[item]->SetVisible(visible);
  }
  canvas_->UpdateView();
}

void Mapviz::FixedFrameSelected(const QString& text)
{
  if (!updating_frames_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "fixed frame selected: %s",
      text.toStdString().c_str());
    if (canvas_ != nullptr) {
      canvas_->SetFixedFrame(text.toStdString());
    }
  }
}

void Mapviz::TargetFrameSelected(const QString& text)
{
  if (!updating_frames_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Target frame selected: %s",
      text.toStdString().c_str());

    if (canvas_ != nullptr) {
      canvas_->SetTargetFrame(text.toStdString());
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

void Mapviz::ToggleRotate90(bool on)
{
  canvas_->ToggleRotate90(on);
}

void Mapviz::ToggleEnableAntialiasing(bool on)
{
  canvas_->ToggleEnableAntialiasing(on);
}

void Mapviz::ToggleConfigPanel(bool on)
{
  if (on) {
    ui_.configdock->show();
  } else {
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
  if (on) {
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

  if (on) {
    rec_button_->setIcon(QIcon(":/images/media-playback-pause.png"));
    rec_button_->setToolTip("Pause recording video of display canvas");
    if (!vid_writer_->isRecording()) {
      // Lock the window size.
      AdjustWindowSize();

      canvas_->CaptureFrames(true);

      std::string posix_time = boost::posix_time::to_iso_string(
                                boost::posix_time::second_clock::local_time());
      boost::replace_all(posix_time, ".", "_");
      std::string filename = capture_directory_ + "/mapviz_" + posix_time + ".avi";
      boost::replace_all(filename, "~", getenv("HOME"));


      if (!vid_writer_->initializeWriter(filename, canvas_->width(), canvas_->height())) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open video file for writing");
        StopRecord();
        return;
      }

      RCLCPP_INFO(node_->get_logger(), "Writing video to: %s", filename.c_str());
      ui_.statusbar->showMessage("Recording video to " + QString::fromStdString(filename));

      canvas_->updateGL();
    }

    record_timer_.start(1000.0 / 30.0);
  } else {
    rec_button_->setIcon(QIcon(":/images/media-record.png"));
    rec_button_->setToolTip("Continue recording video of display canvas");
    record_timer_.stop();
  }
}

void Mapviz::SetImageTransport(QAction* transport_action)
{
  std::string transport = transport_action->text().toStdString();
  RCLCPP_INFO(
    node_->get_logger(),
    "Setting %s to %s",
    IMAGE_TRANSPORT_PARAM,
    transport.c_str());
  node_->set_parameter({IMAGE_TRANSPORT_PARAM, transport});

  Q_EMIT(ImageTransportChanged());
}

void Mapviz::UpdateImageTransportMenu()
{
  QList<QAction*> actions = image_transport_menu_->actions();

  std::string current_transport;
  node_->get_parameter_or(IMAGE_TRANSPORT_PARAM, current_transport, std::string("raw"));
  for(const auto action : actions)
  {
    if (action->text() == QString::fromStdString(current_transport)) {
      action->setChecked(true);
      return;
    }
  }

  RCLCPP_WARN(
    node_->get_logger(),
    "%s param was set to an unrecognized value: %s",
    IMAGE_TRANSPORT_PARAM,
    current_transport.c_str());
}

void Mapviz::CaptureVideoFrame()
{
  // We need to store the data inside a QImage in order to emit it as a
  // signal.
  // Note that the QImage here is set to "ARGB32", but it is actually BGRA.
  // Qt doesn't have a comparable BGR format, and the cv::VideoWriter this
  // is going to expects BGR format, but it'd be a waste for us to convert
  // to RGB and then back to BGR.
  QImage frame(canvas_->width(), canvas_->height(), QImage::Format_ARGB32);
  if (canvas_->CopyCaptureBuffer(frame.bits())) {
    Q_EMIT(FrameGrabbed(frame));
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("mapviz"), "Failed to get capture buffer");
  }
}

void Mapviz::Recenter()
{
  canvas_->ResetLocation();
}

void Mapviz::StopRecord()
{
  rec_button_->setChecked(false);
  stop_button_->setEnabled(false);

  record_timer_.stop();
  if (vid_writer_) {
    vid_writer_->stop();
  }
  canvas_->CaptureFrames(false);

  ui_.statusbar->showMessage(QString(""));
  rec_button_->setToolTip("Start recording video of display canvas");

  AdjustWindowSize();
}

void Mapviz::Screenshot()
{
  canvas_->CaptureFrame(true);

  std::vector<uint8_t> frame;
  if (canvas_->CopyCaptureBuffer(frame)) {
    cv::Mat image(canvas_->height(), canvas_->width(), CV_8UC4, &frame[0]);
    cv::Mat screenshot;
    cvtColor(image, screenshot, cv::COLOR_BGRA2BGR);

    cv::flip(screenshot, screenshot, 0);

    std::string posix_time = boost::posix_time::to_iso_string(
                              boost::posix_time::second_clock::local_time());
    boost::replace_all(posix_time, ".", "_");
    std::string filename = capture_directory_ + "/mapviz_" + posix_time + ".png";
    boost::replace_all(filename, "~", getenv("HOME"));

    RCLCPP_INFO(rclcpp::get_logger("mapviz"), "Writing screenshot to: %s", filename.c_str());
    ui_.statusbar->showMessage("Saved image to " + QString::fromStdString(filename));

    cv::imwrite(filename, screenshot);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("mapviz"), "Failed to take screenshot.");
  }
}

void Mapviz::UpdateSizeHints()
{
  for (int i = 0; i < ui_.configs->count(); i++) {
    QListWidgetItem* item = ui_.configs->item(i);
    auto* widget = dynamic_cast<ConfigItem*>(ui_.configs->itemWidget(item));
    if (widget) {
      // Make sure the ConfigItem in the QListWidgetItem we're getting really
      // exists; if this method is called before it's been initialized, it would
      // cause a crash.
      item->setSizeHint(widget->sizeHint());
    }
  }
}

void Mapviz::RemoveDisplay()
{
  QListWidgetItem* item = ui_.configs->takeItem(ui_.configs->currentRow());
  RemoveDisplay(item);
}

void Mapviz::RemoveDisplay(QListWidgetItem* item)
{
  RCLCPP_INFO(rclcpp::get_logger("mapviz"), "Remove display ...");

  if (item) {
    canvas_->RemovePlugin(plugins_[item]);
    plugins_.erase(item);

    delete item;
  }
}

void Mapviz::RenameDisplay()
{
  QListWidgetItem* item = ui_.configs->currentItem();
  RenameDisplay(item);
}

void Mapviz::RenameDisplay(QListWidgetItem* item)
{
  if (item) {
    ConfigItem* config_item = static_cast<ConfigItem*>(ui_.configs->itemWidget(item));
    config_item->EditName();
  }
}

void Mapviz::ClearDisplays()
{
  while (ui_.configs->count() > 0) {
    RCLCPP_INFO(node_->get_logger(), "Remove display ...");

    QListWidgetItem* item = ui_.configs->takeItem(0);

    canvas_->RemovePlugin(plugins_[item]);
    plugins_.erase(item);

    delete item;
  }
}

void Mapviz::ReorderDisplays()
{
  RCLCPP_INFO(rclcpp::get_logger("mapviz"), "Reorder displays");
  for (int i = 0; i < ui_.configs->count(); i++) {
    plugins_[ui_.configs->item(i)]->SetDrawOrder(i);
  }
  canvas_->ReorderDisplays();
}

void Mapviz::SelectBackgroundColor(const QColor &color)
{
  background_ = color;
  canvas_->SetBackground(background_);
}

void Mapviz::SetCaptureDirectory()
{
  QFileDialog dialog(this, "Select Capture Directory");
  dialog.setOption(QFileDialog::ShowDirsOnly, true);

  dialog.exec();

  if (dialog.result() == QDialog::Accepted && dialog.selectedFiles().count() == 1) {
    capture_directory_ = dialog.selectedFiles().first().toStdString();
  }
}

void Mapviz::HandleProfileTimer()
{
  RCLCPP_INFO(node_->get_logger(), "Mapviz Profiling Data");
  meas_spin_.printInfo(node_->get_logger(), "ROS SpinOnce()");
  for (auto& display : plugins_) {
    MapvizPluginPtr plugin = display.second;
    if (plugin) {
      plugin->PrintMeasurements();
    }
  }
}
}   // namespace mapviz
