#ifndef MULTIRES_IMAGE_QGLMAP_H_
#define MULTIRES_IMAGE_QGLMAP_H_

// QT libraries
#include <QGLWidget>
#include <QMouseEvent>
#include <QWheelEvent>

// QT auto-generated headers
#include "ui_QGLMap.h"

#include <multires_image/tile.h>
#include <multires_image/tile_view.h>
#include <multires_image/bounding_box.h>
#include <multires_image/point.h>

namespace multires_image
{
  class QGLMap : public QGLWidget
  {
    Q_OBJECT

  public:
    QGLMap(QWidget *parent = 0);
    ~QGLMap();

    void Exit();
    void UpdateView();
    void SetTiles(TileSet* tiles);

    PointT<double> SceneCenter() { return m_sceneBox.Center; }
    PointT<double> ViewCenter() { return m_viewBox.Center; }

  signals:
    void SignalZoomChange(double z);
    void SignalViewChange(double x1, double y1, double x2, double y2);
    void SignalMemorySize(long);

  public slots:
    void LoadTexture(Tile* tile);
    void DeleteTexture(Tile* tile);
    void ChangeCenter(double x, double y);
    void SetTextureMemory(long);

  protected:
    void initializeGL();
    void resizeGL( int w, int h );
    void paintGL();
    void mousePressEvent(QMouseEvent* e);
    void mouseDoubleClickEvent(QMouseEvent* e);
    void mouseReleaseEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent* e);
    void wheelEvent(QWheelEvent* e);

  private:
    Ui::QGLMapClass ui;
    PointT<double>  m_viewCenter;
    bool            m_initialized;

    double          m_scale;

    bool            m_mouseDown;
    int             m_mouseDownX;
    int             m_mouseDownY;

    TileView*       m_tileView;

    BoundingBox<double> m_viewBox;
    BoundingBox<double> m_sceneBox;

    void Recenter();
    void MousePan(int x, int y);
  };
}

#endif  // MULTIRES_IMAGE_QGLMAP_H_
