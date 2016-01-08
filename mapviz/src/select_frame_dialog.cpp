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
#include <mapviz/select_frame_dialog.h>

#include <algorithm>
#include <set>

#include <QListWidget>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTimerEvent>

#include <tf/transform_listener.h>

namespace mapviz
{
std::string SelectFrameDialog::selectFrame(
  boost::shared_ptr<tf::TransformListener> tf_listener,
  QWidget *parent)
{
  SelectFrameDialog dialog(tf_listener, parent);
  dialog.allowMultipleFrames(false);
  if (dialog.exec() == QDialog::Accepted) {
    return dialog.selectedFrame();
  } else {
    return "";
  }
}

std::vector<std::string> SelectFrameDialog::selectFrames(
  boost::shared_ptr<tf::TransformListener> tf_listener,
  QWidget *parent)
{
  SelectFrameDialog dialog(tf_listener, parent);
  dialog.allowMultipleFrames(true);
  if (dialog.exec() == QDialog::Accepted) {
    return dialog.selectedFrames();
  } else {
    return std::vector<std::string>();
  }
}

SelectFrameDialog::SelectFrameDialog(
  boost::shared_ptr<tf::TransformListener> tf_listener,
  QWidget *parent)
  :
  tf_(tf_listener),
  ok_button_(new QPushButton("&Ok")),
  cancel_button_(new QPushButton("&Cancel")),
  list_widget_(new QListWidget()),
  name_filter_(new QLineEdit())
{
  QHBoxLayout *filter_box = new QHBoxLayout();
  filter_box->addWidget(new QLabel("Filter:"));
  filter_box->addWidget(name_filter_);

  QHBoxLayout *button_box = new QHBoxLayout();
  button_box->addStretch(1);
  button_box->addWidget(cancel_button_);
  button_box->addWidget(ok_button_);

  QVBoxLayout *vbox = new QVBoxLayout();
  vbox->addWidget(list_widget_);
  vbox->addLayout(filter_box);
  vbox->addLayout(button_box);
  setLayout(vbox);

  connect(ok_button_, SIGNAL(clicked(bool)),
          this, SLOT(accept()));
  connect(cancel_button_, SIGNAL(clicked(bool)),
          this, SLOT(reject()));
  connect(name_filter_, SIGNAL(textChanged(const QString &)),
          this, SLOT(updateDisplayedFrames()));

  ok_button_->setDefault(true);
  
  allowMultipleFrames(false);
  setWindowTitle("Select frames...");

  resize(600, 600);
  
  fetch_frames_timer_id_ = startTimer(1000);
  fetchFrames();
}

void SelectFrameDialog::timerEvent(QTimerEvent *event)
{  
  if (event->timerId() == fetch_frames_timer_id_) {
    fetchFrames();
  }
}

void SelectFrameDialog::closeEvent(QCloseEvent *event)
{
  // We don't need to keep making requests from the ROS master.
  killTimer(fetch_frames_timer_id_);
  QDialog::closeEvent(event);
}

void SelectFrameDialog::allowMultipleFrames(
  bool allow)
{
  if (allow) {
    list_widget_->setSelectionMode(QAbstractItemView::MultiSelection);
  } else {
    list_widget_->setSelectionMode(QAbstractItemView::SingleSelection);
  }
}

std::string SelectFrameDialog::selectedFrame() const
{
  std::vector<std::string> selection = selectedFrames();
  if (selection.empty()) {
    return "";
  } else {
    return selection.front();
  }
}

std::vector<std::string> SelectFrameDialog::selectedFrames() const
{
  QModelIndexList qt_selection = list_widget_->selectionModel()->selectedIndexes();

  std::vector<std::string> selection;
  selection.resize(qt_selection.size());
  for (int i = 0; i < qt_selection.size(); i++) {
    if (!qt_selection[i].isValid()) {
      continue;
    }
    
    int row = qt_selection[i].row();
    if (row < 0 || static_cast<size_t>(row) >= displayed_frames_.size()) {
      continue;
    }
    
    selection[i] = displayed_frames_[row];
  }

  return selection;
}

void SelectFrameDialog::fetchFrames()
{
  if (!tf_) { return; }
  
  known_frames_.clear();
  tf_->getFrameStrings(known_frames_);
  std::sort(known_frames_.begin(), known_frames_.end());
  updateDisplayedFrames();
}

std::vector<std::string> SelectFrameDialog::filterFrames(
  const std::vector<std::string> &frames) const
{
  QString frame_filter = name_filter_->text();
  std::vector<std::string> filtered;

  for (size_t i = 0; i < frames.size(); i++) {
    QString frame_name = QString::fromStdString(frames[i]);
    if (!frame_filter.isEmpty() &&
        !frame_name.contains(frame_filter, Qt::CaseInsensitive)) {
      continue;
    }

    filtered.push_back(frames[i]);
  }
  
  return filtered;  
}

void SelectFrameDialog::updateDisplayedFrames()
{
  std::vector<std::string> next_displayed_frames = filterFrames(known_frames_);
  
  // It's a lot more work to keep track of the additions/removals like
  // this compared to resetting the QListWidget's items each time, but
  // it allows Qt to properly track the selection and current items
  // across updates, which results in much less frustration for the user.
  
  std::set<std::string> prev_names;
  prev_names.insert(displayed_frames_.begin(), displayed_frames_.end());
  
  std::set<std::string> next_names;
  next_names.insert(next_displayed_frames.begin(), next_displayed_frames.end());

  std::set<std::string> added_names;
  std::set_difference(next_names.begin(), next_names.end(),
                      prev_names.begin(), prev_names.end(),
                      std::inserter(added_names, added_names.end()));

  std::set<std::string> removed_names;
  std::set_difference(prev_names.begin(), prev_names.end(),
                      next_names.begin(), next_names.end(),
                      std::inserter(removed_names, removed_names.end()));

  // Remove all the removed names
  size_t removed = 0;
  for (size_t i = 0; i < displayed_frames_.size(); i++) {
    if (removed_names.count(displayed_frames_[i]) == 0) {
      continue;
    }

    QListWidgetItem *item = list_widget_->takeItem(i - removed);
    delete item;
    removed++;
  }

  // Now we can add the new items.
  for (size_t i = 0; i < next_displayed_frames.size(); i++) {
    if (added_names.count(next_displayed_frames[i]) == 0) {
      continue;
    }

    list_widget_->insertItem(i, QString::fromStdString(next_displayed_frames[i]));
    if (list_widget_->count() == 1) {
      list_widget_->setCurrentRow(0);
    }
  }

  displayed_frames_.swap(next_displayed_frames);  
}
}  // namespace mapviz
