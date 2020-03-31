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
#include <mapviz/select_topic_dialog.h>

#include <QListWidget>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTimerEvent>

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <vector>


namespace mapviz
{
std::string SelectTopicDialog::selectTopic(
  const rclcpp::Node::SharedPtr& node,
  const std::string &datatype,
  QWidget *parent)
{
  std::vector<std::string> datatypes;
  datatypes.push_back(datatype);
  return selectTopic(node, datatypes, parent);
}

std::string SelectTopicDialog::selectTopic(
  const rclcpp::Node::SharedPtr& node,
  const std::string &datatype1,
  const std::string &datatype2,
  QWidget *parent)
{
  std::vector<std::string> datatypes;
  datatypes.push_back(datatype1);
  datatypes.push_back(datatype2);
  return selectTopic(node, datatypes, parent);
}

std::string SelectTopicDialog::selectTopic(
  const rclcpp::Node::SharedPtr& node,
  const std::vector<std::string> &datatypes,
  QWidget *parent)
{
  SelectTopicDialog dialog(node, parent);
  dialog.allowMultipleTopics(false);
  dialog.setDatatypeFilter(datatypes);
  if (dialog.exec() == QDialog::Accepted) {
    return dialog.selectedTopic();
  } else {
    return std::string();
  }
}

std::vector<std::string> SelectTopicDialog::selectTopics(
  const rclcpp::Node::SharedPtr& node,
  const std::string &datatype,
  QWidget *parent)
{
  std::vector<std::string> datatypes;
  datatypes.push_back(datatype);
  return selectTopics(node, datatypes, parent);
}

std::vector<std::string> SelectTopicDialog::selectTopics(
  const rclcpp::Node::SharedPtr& node,
  const std::string &datatype1,
  const std::string &datatype2,
  QWidget *parent)
{
  std::vector<std::string> datatypes;
  datatypes.push_back(datatype1);
  datatypes.push_back(datatype2);
  return selectTopics(node, datatypes, parent);
}

std::vector<std::string> SelectTopicDialog::selectTopics(
  const rclcpp::Node::SharedPtr& node,
  const std::vector<std::string> &datatypes,
  QWidget *parent)
{
  SelectTopicDialog dialog(node, parent);
  dialog.allowMultipleTopics(true);
  dialog.setDatatypeFilter(datatypes);
  if (dialog.exec() == QDialog::Accepted) {
    return dialog.selectedTopics();
  } else {
    return std::vector<std::string>();
  }
}

SelectTopicDialog::SelectTopicDialog(const rclcpp::Node::SharedPtr& node, QWidget *parent)
  :
  nh_(node),
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
          this, SLOT(updateDisplayedTopics()));

  ok_button_->setDefault(true);

  allowMultipleTopics(false);
  setWindowTitle("Select topics...");

  fetch_topics_timer_id_ = startTimer(1000);
  fetchTopics();
}

void SelectTopicDialog::timerEvent(QTimerEvent *event)
{
  if (event->timerId() == fetch_topics_timer_id_) {
    fetchTopics();
  }
}

void SelectTopicDialog::closeEvent(QCloseEvent *event)
{
  // We don't need to keep making requests from the ROS master.
  killTimer(fetch_topics_timer_id_);
  QDialog::closeEvent(event);
}

void SelectTopicDialog::allowMultipleTopics(
  bool allow)
{
  if (allow) {
    list_widget_->setSelectionMode(QAbstractItemView::MultiSelection);
  } else {
    list_widget_->setSelectionMode(QAbstractItemView::SingleSelection);
  }
}

void SelectTopicDialog::setDatatypeFilter(
  const std::vector<std::string> &datatypes)
{
  allowed_datatypes_.clear();
  for (const auto & datatype : datatypes) {
    allowed_datatypes_.insert(datatype);
  }
  updateDisplayedTopics();
}

std::string SelectTopicDialog::selectedTopic() const
{
  std::vector<std::string> selection = selectedTopics();
  if (selection.empty()) {
    return std::string();
  } else {
    return selection.front();
  }
}

std::vector<std::string> SelectTopicDialog::selectedTopics() const
{
  QModelIndexList qt_selection = list_widget_->selectionModel()->selectedIndexes();

  std::vector<std::string> selection;
  selection.resize(qt_selection.size());
  for (int i = 0; i < qt_selection.size(); i++) {
    if (!qt_selection[i].isValid()) {
      continue;
    }

    int row = qt_selection[i].row();
    if (row < 0 || static_cast<size_t>(row) >= displayed_topics_.size()) {
      continue;
    }

    selection[i] = displayed_topics_[row];
  }

  return selection;
}

static bool topicSort(const std::string &info1,
                      const std::string &info2)
{
  return info1 < info2;
}

void SelectTopicDialog::fetchTopics()
{
  known_topics_ = nh_->get_topic_names_and_types();
  std::vector<std::string> map_keys;
  for (auto const& element : known_topics_)
  {
    map_keys.push_back(element.first);
  }
  std::sort(map_keys.begin(), map_keys.end(), topicSort);
  updateDisplayedTopics();
}

std::vector<std::string> SelectTopicDialog::filterTopics(
  const std::map<std::string, std::vector<std::string>> &topics) const
{
  QString topic_filter = name_filter_->text();
  std::vector<std::string> filtered;

  for (auto const& topic : topics) {
    if (!allowed_datatypes_.empty()) {
      // Skip any topic names that don't contain allowed types
      bool missing_allowed_type = true;   // Assume the worst
      for (auto const& datatype : topic.second) {
        if (allowed_datatypes_.count(datatype) == 1) {
          missing_allowed_type = false;
          break;
        }
      }
      if (missing_allowed_type) {
        continue;
      }
    }

    QString topic_name = QString::fromStdString(topic.first);
    if (!topic_filter.isEmpty() &&
        !topic_name.contains(topic_filter, Qt::CaseInsensitive)) {
          continue;
    }

    filtered.push_back(topic.first);
  }

  return filtered;
}

void SelectTopicDialog::updateDisplayedTopics()
{
  std::vector<std::string> next_displayed_topics = filterTopics(known_topics_);

  // It's a lot more work to keep track of the additions/removals like
  // this compared to resetting the QListWidget's items each time, but
  // it allows Qt to properly track the selection and current items
  // across updates, which results in much less frustration for the user.

  std::set<std::string> prev_names;
  for (const auto & displayed_topic : displayed_topics_) {
    prev_names.insert(displayed_topic);
  }

  std::set<std::string> next_names;
  for (const auto & next_displayed_topic : next_displayed_topics) {
    next_names.insert(next_displayed_topic);
  }

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
  for (size_t i = 0; i < displayed_topics_.size(); i++) {
    if (removed_names.count(displayed_topics_[i]) == 0) {
      continue;
    }
    RCLCPP_DEBUG(nh_->get_logger(), "Removing %s", displayed_topics_[i].c_str());

    QListWidgetItem *item = list_widget_->takeItem(i - removed);
    delete item;
    removed++;
  }

  // Now we can add the new items.
  for (size_t i = 0; i < next_displayed_topics.size(); i++) {
    if (added_names.count(next_displayed_topics[i]) == 0) {
      continue;
    }

    list_widget_->insertItem(i, QString::fromStdString(next_displayed_topics[i]));
    RCLCPP_DEBUG(nh_->get_logger(), "Inserting %s", next_displayed_topics[i].c_str());
    if (list_widget_->count() == 1) {
      list_widget_->setCurrentRow(0);
    }
  }

  displayed_topics_.swap(next_displayed_topics);
}
}  // namespace mapviz
