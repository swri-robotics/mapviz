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
