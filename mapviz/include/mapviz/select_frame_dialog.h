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
#ifndef MAPVIZ__SELECT_FRAME_DIALOG_H_
#define MAPVIZ__SELECT_FRAME_DIALOG_H_

#include <tf2_ros/buffer.h>

#include <QDialog>

#include <memory>
#include <string>
#include <vector>


QT_BEGIN_NAMESPACE
class QLineEdit;
class QListWidget;
class QPushButton;
QT_END_NAMESPACE

namespace mapviz
{
/**
 * Provides a dialog for the user to select one or more TF frames.
 * Several static functions are provided that can be used instead of
 * instantiating the class directly.
 */
class SelectFrameDialog : public QDialog
{
  Q_OBJECT

 public:
  /**
   * Present the user with a dialog to select a single frame.
   *
   * If the user cancels the selection or doesn't make a valid
   * selection, the returned string be empty.
   */
  static std::string selectFrame(
      std::shared_ptr<tf2_ros::Buffer> tf_buffer,
      QWidget *parent = nullptr);

  /**
   * Present the user with a dialog to select a multiple TF frames.
   *
   * If the user cancels the selection or doesn't make a valid
   * selection, the returned vector will be empty.
   */
  static std::vector<std::string> selectFrames(
      std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    QWidget *parent = nullptr);

  /**
   * Constructor for the SelectFrameDialog.
   */
  explicit SelectFrameDialog(std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                             QWidget *parent = nullptr);

  /**
   * Choose whether the user can select one (allow=false) or multiple
   * (allow=true) frames.  The default is false.
   */
  void allowMultipleFrames(bool allow);

  /**
   * Returns the currently selected frame.  If multiple frames are
   * allowed, this will only return the first selected element.  If
   * there is no selection, the returned info will have an empty frame
   * name and datatype.
   */
  std::string selectedFrame() const;
  /**
   * Returns the currently selected frames.  If there is no selection,
   * the returned vector will be empty.
   */
  std::vector<std::string> selectedFrames() const;

 private:
  void timerEvent(QTimerEvent *) override;
  void closeEvent(QCloseEvent *) override;

  std::vector<std::string> filterFrames(
    const std::vector<std::string> &) const;

 private Q_SLOTS:
  void fetchFrames();
  void updateDisplayedFrames();

 private:
  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::vector<std::string> known_frames_;
  std::vector<std::string> displayed_frames_;
  int fetch_frames_timer_id_;

  QPushButton *ok_button_;
  QPushButton *cancel_button_;
  QListWidget *list_widget_;
  QLineEdit *name_filter_;
};  // class SelectFrameDialog
}  // namespace mapviz
#endif  // MAPVIZ__SELECT_FRAME_DIALOG_H_
