// C++ standard libraries
#include <cstdio>

// QT libraries
#include <QFileDialog>
#include <QGLWidget>
#include <QPalette>

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <mapviz_plugins/multires_image_plugin.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(mapviz_plugins, mutlires_image, mapviz_plugins::MultiresImagePlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{

  MultiresImagePlugin::MultiresImagePlugin() :
    loaded_(false),
    tile_set_(NULL),
    tile_view_(NULL),
    config_widget_(new QWidget()),
    transformed_(false)
  {
    ui_.setupUi(config_widget_);

    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    QPalette p2(ui_.status->palette());
    p2.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p2);

    QObject::connect(ui_.browse, SIGNAL(clicked()), this, SLOT(SelectFile()));
    QObject::connect(ui_.path, SIGNAL(editingFinished()), this, SLOT(AcceptConfiguration()));

    source_frame_ = "/utm";
  }

  MultiresImagePlugin::~MultiresImagePlugin()
  {
    delete tile_view_;
    delete tile_set_;
  }

  void MultiresImagePlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_ERROR("Error: %s", message.c_str()); 
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void MultiresImagePlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_INFO("%s", message.c_str()); 
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void MultiresImagePlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void MultiresImagePlugin::AcceptConfiguration()
  {
    ROS_INFO("Accept multires image configuration.");
    if (tile_set_ != NULL && tile_set_->GeoReference().GeoPath() == ui_.path->text().toStdString())
    {
      // Nothing to do.
    }
    else
    {
      loaded_ = false;
      delete tile_set_;
      delete tile_view_;
      tile_set_ = new multires_image::TileSet(ui_.path->text().toStdString());

      if (tile_set_->Load())
      {
        loaded_ = true;

        QPalette p(ui_.status->palette());
        p.setColor(QPalette::Text, Qt::green);
        ui_.status->setPalette(p);
        ui_.status->setText("OK");

        InitializeTiles();

        initialized_ = true;

        MultiresView* view = new MultiresView(tile_set_, canvas_);
        tile_view_ = view;

        canvas_->update();
      }
      else
      {
        PrintError("Failed to load image.");
        delete tile_set_;
        tile_set_ = 0;
        tile_view_ = 0;
      }
    }
  }

  void MultiresImagePlugin::InitializeTiles()
  {
    // Initialize all of the tiles.
    for (int i = 0; i < tile_set_->LayerCount(); i++)
    {
      multires_image::TileSetLayer* layer = tile_set_->GetLayer(i);
      for (int r = 0; r < layer->RowCount(); r++)
      {
        for (int c = 0; c < layer->ColumnCount(); c++)
        {
          layer->GetTile(c, r)->Initialize();
        }
      }
    }
  }

  void MultiresImagePlugin::SelectFile()
  {
    QFileDialog dialog(config_widget_, "Select Multires Image");
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setNameFilter(tr("Geo Files (*.geo)"));

    dialog.exec();

    if (dialog.result() == QDialog::Accepted && dialog.selectedFiles().count() == 1)
    {
      ui_.path->setText(dialog.selectedFiles().first());
      AcceptConfiguration();
    }
  }

  QWidget* MultiresImagePlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool MultiresImagePlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;
  
    return true;
  }

  void MultiresImagePlugin::GetCenterPoint(double x, double y)
  {
      tf::Point point(x, y, 0);
      tf::Point center = transform_.inverse() * point;
      center_x_ = center.getX();
      center_y_ = center.getY();
  }
    
  void MultiresImagePlugin::Draw(double x, double y, double scale)
  {
    if (transformed_ && tile_set_ != NULL && tile_view_ != NULL)
    {
      GetCenterPoint(x, y);
      tile_view_->SetView(center_x_, center_y_, 1, scale);
      
      tile_view_->Draw();

      PrintInfo("OK");
    }
  }

  void MultiresImagePlugin::Transform()
  {
    transformed_ = false;

    if (!loaded_)
      return;

    if (target_frame_ == "" || target_frame_ == "utm" || target_frame_ == "/utm")
    {
      // Set relative positions of tile points to be in straight UTM
      for (int i = 0; i < tile_set_->LayerCount(); i++)
      {
        multires_image::TileSetLayer* layer = tile_set_->GetLayer(i);
        for (int r = 0; r < layer->RowCount(); r++)
        {
          for (int c = 0; c < layer->ColumnCount(); c++)
          {
            multires_image::Tile* tile = layer->GetTile(c, r);

            if (!tile->HasUtm())
            {
              PrintError("Can't resolve image to UTM space.");
            }

            double top_left_x, top_left_y; 
            double top_right_x, top_right_y;
            double bottom_right_x, bottom_right_y;
            double bottom_left_x, bottom_left_y;
            tile->GetUtmPosition(
              top_left_x, top_left_y, 
              top_right_x, top_right_y,
              bottom_right_x, bottom_right_y,
              bottom_left_x, bottom_left_y);

            tile->SetRelativePosition(
              top_left_x, top_left_y, 
              top_right_x, top_right_y,
              bottom_right_x, bottom_right_y,
              bottom_left_x, bottom_left_y);
          }
        }

        transformed_ = true;
      }
    }
    else if (GetTransform(ros::Time(), transform_))
    {
      // Set relative positions of tile points based on tf transform
      for (int i = 0; i < tile_set_->LayerCount(); i++)
      {
        multires_image::TileSetLayer* layer = tile_set_->GetLayer(i);
        for (int r = 0; r < layer->RowCount(); r++)
        {
          for (int c = 0; c < layer->ColumnCount(); c++)
          {
            multires_image::Tile* tile = layer->GetTile(c, r);

            if (!tile->HasUtm())
            {
              PrintError("Can't resolve image to UTM space.");
              return;
            }

            double top_left_x, top_left_y; 
            double top_right_x, top_right_y;
            double bottom_right_x, bottom_right_y;
            double bottom_left_x, bottom_left_y;
            tile->GetUtmPosition(
              top_left_x, top_left_y, 
              top_right_x, top_right_y,
              bottom_right_x, bottom_right_y,
              bottom_left_x, bottom_left_y);

            tf::Point topLeftUtm(top_left_x, top_left_y, 0);
            tf::Point topRightUtm(top_right_x, top_right_y, 0);
            tf::Point bottomRightUtm(bottom_right_x, bottom_right_y, 0);
            tf::Point bottomLeftUtm(bottom_left_x, bottom_left_y, 0);

            tf::Point topLeftRelative = transform_ * topLeftUtm;
            tf::Point topRightRelative = transform_ * topRightUtm;
            tf::Point bottomRightRelative = transform_ * bottomRightUtm;
            tf::Point bottomLeftRelative = transform_ * bottomLeftUtm;

            tile->SetRelativePosition(
              topLeftRelative.getX(),
              topLeftRelative.getY(),
              topRightRelative.getX(),
              topRightRelative.getY(),
              bottomRightRelative.getX(),
              bottomRightRelative.getY(),
              bottomLeftRelative.getX(),
              bottomLeftRelative.getY());
          }
        }
      }

      transformed_ = true;
    }
    else
    {
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }
  }
    
  boost::filesystem::path MultiresImagePlugin::MakePathRelative(boost::filesystem::path path, boost::filesystem::path base)
  {
    // Borrowed from: https://svn.boost.org/trac/boost/ticket/1976#comment:2
    if (path.has_root_path())
    {
      if (path.root_path() != base.root_path())
      {
        return path;
      }
      else
      {
        return MakePathRelative(path.relative_path(), base.relative_path());
      }
    }
    else
    {
      if (base.has_root_path())
      {
        ROS_ERROR("Cannot uncomplete a path relative path from a rooted base.");
        return path;
      }
      else
      {
        typedef boost::filesystem::path::const_iterator path_iterator;
        path_iterator path_it = path.begin();
        path_iterator base_it = base.begin();
        while (path_it != path.end() && base_it != base.end())
        {
          if (*path_it != *base_it)
            break;
          ++path_it;
          ++base_it;
        }
        boost::filesystem::path result;
        for (; base_it != base.end(); ++base_it)
        {
          result /= "..";
        }
        for (; path_it != path.end(); ++path_it)
        {
          result /= *path_it;
        }
        return result;
      }
    }
  }

  void MultiresImagePlugin::LoadConfiguration(const YAML::Node& node, const std::string& config_path)
  {
    std::string path;
    node["path"] >> path;

    boost::filesystem::path image_path(path);
    if(image_path.is_complete() == false)
    {
      boost::filesystem::path base_path(config_path);
      path = (config_path / image_path.relative_path()).normalize().string();
    }

    ui_.path->setText(path.c_str());

    AcceptConfiguration();
  }
  
  void MultiresImagePlugin::SaveConfiguration(YAML::Emitter& emitter, const std::string& config_path)
  {
    boost::filesystem::path absolute_path(ui_.path->text().toStdString());
    boost::filesystem::path base_path(config_path);
    boost::filesystem::path relative_path = MakePathRelative(absolute_path, base_path);

    emitter << YAML::Key << "path" << YAML::Value << relative_path.string();
  }
  
}

