// C++ standard libraries
#include <fstream>

// Boost libraries
#include <boost/filesystem.hpp>
#define BOOST_FILESYSTEM_VERSION 2

// QT libraries
#include <QtGui/QApplication>
#include <QFileDialog>
#include <QActionGroup>
#include <QColorDialog>

#include "mapviz.h"
#include "config_item.h"

Mapviz::Mapviz(int argc, char **argv, QWidget *parent, Qt::WFlags flags) :
    QMainWindow(parent, flags),
    argc_(argc),
    argv_(argv),
    initialized_(false),
    force_720p_(false),
    force_480p_(false),
    resizable_(true),
    background_(Qt::gray),
    node_(NULL),
    canvas_(NULL)
{
  ui_.setupUi(this);

  ui_.statusbar->setVisible(false);

  QActionGroup* group = new QActionGroup(this);

  ui_.actionForce_720p->setActionGroup(group);
  ui_.actionForce_480p->setActionGroup(group);
  ui_.actionResizable->setActionGroup(group);

  ui_.targetframecombo->addItem("<none>");

  canvas_ = new MapCanvas(this);
  setCentralWidget(canvas_);
  QObject::connect(ui_.configlist, SIGNAL(ItemsMoved()), this, SLOT(ReorderDisplays()));
  QObject::connect(ui_.actionExit, SIGNAL(triggered()), this, SLOT(close()));


  ui_.backgroundcolor->setStyleSheet("background: " + background_.name() + ";");
  canvas_->SetBackground(background_);
}

Mapviz::~Mapviz()
{
  delete tf_;
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
    QObject::connect(&spin_timer_, SIGNAL(timeout()), this, SLOT(SpinOnce()));

    node_ = new ros::NodeHandle();
    tf_ = new tf::TransformListener();

    loader_ = new pluginlib::ClassLoader<mapviz::MapvizPlugin>("mapviz", "mapviz::MapvizPlugin");

    std::vector<std::string> plugins = loader_->getDeclaredClasses();
    for (unsigned int i = 0; i < plugins.size(); i++)
    {
      ROS_INFO("Found mapviz plugin: %s", plugins[i].c_str());
    }

    canvas_->InitializeTf();
    canvas_->SetFixedFrame(ui_.fixedframecombo->currentText().toStdString());
    canvas_->SetTargetFrame(ui_.targetframecombo->currentText().toStdString());

    std::string config;
    node_->param(ros::this_node::getName() + "/config", config, std::string(QDir::homePath().toStdString() + "/.mapviz_config"));

    Open(config);

    UpdateFrames();
    frame_timer_.start(1000);
    QObject::connect(&frame_timer_, SIGNAL(timeout()), this, SLOT(UpdateFrames()));

    save_timer_.start(10000);
    QObject::connect(&save_timer_, SIGNAL(timeout()), this, SLOT(AutoSave()));

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

  if ((int)frames.size() == ui_.fixedframecombo->count())
  {
    bool changed = false;
    for (unsigned int i = 0; i < frames.size(); i++)
    {
      if (frames[i] != ui_.fixedframecombo->itemText(i).toStdString())
      {
        changed = true;
      }
    }

    if (!changed)
      return;
  }

  ROS_INFO("Updating frames...");

  std::string current = ui_.fixedframecombo->currentText().toStdString();

  ui_.fixedframecombo->clear();
  for (unsigned int i = 0; i < frames.size(); i++)
  {
    ui_.fixedframecombo->addItem(frames[i].c_str());
  }

  if (current != "")
  {
    int index = ui_.fixedframecombo->findText(current.c_str());
    if (index < 0)
    {
      ui_.fixedframecombo->addItem(current.c_str());
    }

    index = ui_.fixedframecombo->findText(current.c_str());
    ui_.fixedframecombo->setCurrentIndex(index);
  }

  current = ui_.targetframecombo->currentText().toStdString();

  ui_.targetframecombo->clear();
  ui_.targetframecombo->addItem("<none>");
  for (unsigned int i = 0; i < frames.size(); i++)
  {
    ui_.targetframecombo->addItem(frames[i].c_str());
  }

  if (current != "")
  {
    int index = ui_.targetframecombo->findText(current.c_str());
    if (index < 0)
    {
      ui_.targetframecombo->addItem(current.c_str());
    }

    index = ui_.targetframecombo->findText(current.c_str());
    ui_.targetframecombo->setCurrentIndex(index);
  }
}

void Mapviz::Force720p(bool on)
{
  force_720p_ = on;
  AdjustWindowSize();
}

void Mapviz::Force480p(bool on)
{
  force_480p_ = on;
  AdjustWindowSize();
}

void Mapviz::SetResizable(bool on)
{
  resizable_ = on;
  AdjustWindowSize();
}

void Mapviz::AdjustWindowSize()
{
  canvas_->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
  this->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));

  if (force_720p_)
  {
    canvas_->setMinimumSize(1280, 720);
    canvas_->setMaximumSize(1280, 720);
    canvas_->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
    adjustSize();
    this->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  }
  else if (force_480p_)
  {
    canvas_->setMinimumSize(640, 480);
    canvas_->setMaximumSize(640, 480);
    canvas_->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
    adjustSize();
    this->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
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

  std::ifstream fin(filename.c_str());
  if (fin.fail())
  {
    ROS_ERROR("Failed to load file: %s", filename.c_str());
    return;
  }

  try
  {
    boost::filesystem::path filepath(filename);
    std::string config_path = filepath.parent_path().string();

    ClearDisplays();

    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);

    if (doc.FindValue("fixed_frame"))
    {
      std::string fixed_frame;
      doc["fixed_frame"] >> fixed_frame;
      ui_.fixedframecombo->setEditText(fixed_frame.c_str());
    }

    if (doc.FindValue("target_frame"))
    {
      std::string target_frame;
      doc["target_frame"] >> target_frame;
      ui_.targetframecombo->setEditText(target_frame.c_str());
    }

    if (doc.FindValue("fix_orientation"))
    {
      bool fix_orientation = false;
      doc["fix_orientation"] >> fix_orientation;
      ui_.actionFix_Orientation->setChecked(fix_orientation);
    }

    if (doc.FindValue("show_displays"))
    {
      bool show_displays = false;
      doc["show_displays"] >> show_displays;
      ui_.actionConfig_Dock->setChecked(show_displays);
    }

    if (doc.FindValue("window_width"))
    {
      int window_width = 0;
      doc["window_width"] >> window_width;
      resize(window_width, height());
    }

    if (doc.FindValue("window_height"))
    {
      int window_height = 0;
      doc["window_height"] >> window_height;
      resize(width(), window_height);
    }

    if (doc.FindValue("view_scale"))
    {
      float scale = 0;
      doc["view_scale"] >> scale;
      canvas_->SetViewScale(scale);
    }

    if (doc.FindValue("offset_x"))
    {
      float x = 0;
      doc["offset_x"] >> x;
      canvas_->SetOffsetX(x);
    }

    if (doc.FindValue("offset_x"))
    {
      float y = 0;
      doc["offset_y"] >> y;
      canvas_->SetOffsetY(y);
    }

    if (doc.FindValue("background"))
    {
      std::string color;
      doc["background"] >> color;
      background_ = QColor(color.c_str());
      ui_.backgroundcolor->setStyleSheet("background: " + background_.name() + ";");
      canvas_->SetBackground(background_);
    }

    if (const YAML::Node *displays = doc.FindValue("displays"))
    {
      for (unsigned int i = 0; i< displays->size();i++) 
      {
        std::string type, name;
        (*displays)[i]["type"] >> type;
        (*displays)[i]["name"] >> name;

        const YAML::Node& config = (*displays)[i]["config"];

        bool visible = false;
        config["visible"] >> visible;

        bool collapsed = false;
        config["collapsed"] >> collapsed;

        mapviz::MapvizPlugin* plugin = CreateNewDisplay(name, type, visible, collapsed);
        plugin->LoadConfiguration(config, config_path);
      }
    }
  }
  catch (YAML::ParserException& e)
  {
    ROS_ERROR("%s", e.what());
    return;
  }
  catch (YAML::Exception& e)
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
  out << YAML::Key << "fixed_frame" << YAML::Value << ui_.fixedframecombo->currentText().toStdString();
  out << YAML::Key << "target_frame" << YAML::Value << ui_.targetframecombo->currentText().toStdString();
  out << YAML::Key << "fix_orientation" << YAML::Value << ui_.actionFix_Orientation->isChecked();
  out << YAML::Key << "show_displays" << YAML::Value << ui_.actionConfig_Dock->isChecked();
  out << YAML::Key << "window_width" << YAML::Value << width();
  out << YAML::Key << "window_height" << YAML::Value << height();
  out << YAML::Key << "view_scale" << YAML::Value << canvas_->ViewScale();
  out << YAML::Key << "offset_x" << YAML::Value << canvas_->OffsetX();
  out << YAML::Key << "offset_y" << YAML::Value << canvas_->OffsetY();
  out << YAML::Key << "background" << YAML::Value << background_.name().toStdString();


  if (ui_.configlist->count() > 0)
  {
    out << YAML::Key << "displays"<< YAML::Value << YAML::BeginSeq;

    for (int i = 0; i < ui_.configlist->count(); i++)
    {
      out << YAML::BeginMap;
      out << YAML::Key << "type" << YAML::Value << plugins_[ui_.configlist->item(i)]->Type();
      out << YAML::Key << "name" << YAML::Value << plugins_[ui_.configlist->item(i)]->Name();
      out << YAML::Key << "config" << YAML::Value;
      out << YAML::BeginMap;

      out << YAML::Key << "visible" << YAML::Value << plugins_[ui_.configlist->item(i)]->Visible();
      out << YAML::Key << "collapsed" << YAML::Value << (static_cast<ConfigItem*>(ui_.configlist->itemWidget(ui_.configlist->item(i))))->Collapsed();
      
      plugins_[ui_.configlist->item(i)]->SaveConfiguration(out, config_path);

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

mapviz::MapvizPlugin* Mapviz::CreateNewDisplay(const std::string& name, const std::string& type, bool visible, bool collapsed)
{
  ConfigItem* config_item = new ConfigItem();

  config_item->SetName(name.c_str());

  ROS_INFO("creating: %s", type.c_str());
  mapviz::MapvizPlugin* plugin = loader_->createClassInstance(type.c_str());
  plugin->Initialize(canvas_);
  plugin->SetType(type.c_str());
  plugin->SetName(name);
  plugin->SetNode(*node_);
  plugin->SetVisible(visible);
  plugin->SetDrawOrder(ui_.configlist->count());

  // Setup configure widget
  config_item->SetWidget(plugin->GetConfigWidget(this));

  QString pretty_type(type.c_str());
  pretty_type = pretty_type.split('/').last();
  config_item->SetType(pretty_type);
  QListWidgetItem* item = new QListWidgetItem();
  config_item->SetListItem(item);
  item->setSizeHint(config_item->sizeHint());
  QObject::connect(config_item, SIGNAL(UpdateSizeHint()), this, SLOT(UpdateSizeHints()));
  QObject::connect(config_item, SIGNAL(ToggledDraw(QListWidgetItem*,bool)), this, SLOT(ToggleShowPlugin(QListWidgetItem*,bool)));

  ui_.configlist->addItem(item);
  ui_.configlist->setItemWidget(item, config_item);

  // Add plugin to canvas
  plugin->SetTargetFrame(ui_.fixedframecombo->currentText().toStdString());
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
  ROS_INFO("Fixed frame selected: %s", text.toStdString().c_str());
  if (canvas_ != NULL)
  {
    canvas_->SetFixedFrame(text.toStdString().c_str());
  }
}

void Mapviz::TargetFrameSelected(const QString& text)
{
  ROS_INFO("Target frame selected: %s", text.toStdString().c_str());

  if (canvas_ != NULL)
  {
    canvas_->SetTargetFrame(text.toStdString().c_str());
  }
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

void Mapviz::UpdateSizeHints()
{
  ROS_INFO("Updating size hints");
  for (int i = 0; i < ui_.configlist->count(); i++)
  {
    ui_.configlist->item(i)->setSizeHint(ui_.configlist->itemWidget(ui_.configlist->item(i))->sizeHint());
  }
}

void Mapviz::RemoveDisplay()
{
  ROS_INFO("Remove display ...");

  QListWidgetItem* item = ui_.configlist->takeItem(ui_.configlist->currentRow());

  canvas_->RemovePlugin(plugins_[item]);
  plugins_[item] = 0;

  delete item;
}

void Mapviz::ClearDisplays()
{
  while (ui_.configlist->count() > 0)
  {
    ROS_INFO("Remove display ...");

    QListWidgetItem* item = ui_.configlist->takeItem(0);

    canvas_->RemovePlugin(plugins_[item]);
    plugins_[item] = 0;

    delete item;
  }
}

void Mapviz::ReorderDisplays()
{
  ROS_INFO("Reorder displays");
  for (int i = 0; i < ui_.configlist->count(); i++)
  {
    plugins_[ui_.configlist->item(i)]->SetDrawOrder(i);
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
    ui_.backgroundcolor->setStyleSheet("background: " + background_.name() + ";");
    canvas_->SetBackground(background_);
  }
}

int main(int argc, char **argv)
{
  // Initialize QT
  QApplication app(argc, argv);

  Mapviz mapviz(argc, argv);
  mapviz.show();

  return app.exec();
}
