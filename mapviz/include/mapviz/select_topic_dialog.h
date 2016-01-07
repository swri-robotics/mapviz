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
#ifndef MAPVIZ_SELECT_TOPIC_DIALOG_H_
#define MAPVIZ_SELECT_TOPIC_DIALOG_H_

#include <set>
#include <string>
#include <vector>

#include <QDialog>

#include <ros/master.h>

QT_BEGIN_NAMESPACE
class QLineEdit;
class QListWidget;
class QPushButton;
QT_END_NAMESPACE

namespace mapviz
{
/**
 * Provides a dialog for the user to select one or more topics.
 * Several static functions are provided that can be used instead of
 * instantiating the class directly.
 */
class SelectTopicDialog : public QDialog
{
  Q_OBJECT;

 public:
  /**
   * Present the user with a dialog to select a single topic.  This is
   * convenience wrapper for the common case where only one datatype
   * is allowed.
   *
   * If the user cancels the selection or doesn't make a valid
   * selection, the topic and datatype fields of the returned topic
   * info will be empty.
   */
  static ros::master::TopicInfo selectTopic(
    const std::string &datatype,
    QWidget *parent=0);

  /**
   * Present the user with a dialog to select a single topic This is a
   * convenience wrapper for the common case where two datatypes are allowed.
   *
   * If the user cancels the selection or doesn't make a valid
   * selection, the topic and datatype fields of the returned topic
   * info will be empty.
   */
  static ros::master::TopicInfo selectTopic(
    const std::string &datatype1,
    const std::string &datatype2,
    QWidget *parent=0);

  /**
   * Present the user with a dialog to select a single topic.
   *
   * If the user cancels the selection or doesn't make a valid
   * selection, the topic and datatype fields of the returned topic
   * info will be empty.
   */
  static ros::master::TopicInfo selectTopic(
    const std::vector<std::string> &datatypes,
    QWidget *parent=0);
  
  /**
   * Present the user with a dialog to select a multiple topics.  This
   * is a convenience wrapper for the common case where only one
   * datatype is allowed.
   *
   * If the user cancels the selection or doesn't make a valid
   * selection, the returned vector will be empty.
   */
  static std::vector<ros::master::TopicInfo> selectTopics(
    const std::string &datatype,
    QWidget *parent=0);

  /**
   * Present the user with a dialog to select a multiple topics.  This
   * is a convenience wrapper for the common case where two datatypes
   * are allowed.
   *
   * If the user cancels the selection or doesn't make a valid
   * selection, the returned vector will be empty.
   */
  static std::vector<ros::master::TopicInfo> selectTopics(
    const std::string &datatype1,
    const std::string &datatype2,
    QWidget *parent=0);

  /**
   * Present the user with a dialog to select a multiple topics.
   *
   * If the user cancels the selection or doesn't make a valid
   * selection, the returned vector will be empty.
   */
  static std::vector<ros::master::TopicInfo> selectTopics(
    const std::vector<std::string> &datatypes,
    QWidget *parent=0);

  /**
   * Constructor for the SelectTopicDialog.
   */
  SelectTopicDialog(QWidget *parent=0);
  
  /**
   * Choose whether the user can select one (allow=false) or multiple
   * (allow=true) topics.  The default is false.
   */
  void allowMultipleTopics(bool allow);

  /**
   * Set a datatype filter to limit displayed topics based on their
   * types.  If the vector is empty (default), the dialog will display
   * all available topics.
   */
  void setDatatypeFilter(const std::vector<std::string> &datatypes);

  /**
   * Returns the currently selected topic.  If multiple topics are
   * allowed, this will only return the first selected element.  If
   * there is no selection, the returned info will have an empty topic
   * name and datatype.
   */
  ros::master::TopicInfo selectedTopic() const;
  /**
   * Returns the currently selected topics.  If there is no selection,
   * the returned vector will be empty.
   */
  std::vector<ros::master::TopicInfo> selectedTopics() const;

 private:
  void timerEvent(QTimerEvent *);
  void closeEvent(QCloseEvent *);

  std::vector<ros::master::TopicInfo> filterTopics(
    const std::vector<ros::master::TopicInfo> &) const;

 private Q_SLOTS:
  void fetchTopics();
  void updateDisplayedTopics();

 private:
  std::set<std::string> allowed_datatypes_;
  std::vector<ros::master::TopicInfo> known_topics_;
  std::vector<ros::master::TopicInfo> displayed_topics_;
  int fetch_topics_timer_id_;

  QPushButton *ok_button_;
  QPushButton *cancel_button_;
  QListWidget *list_widget_;
  QLineEdit *name_filter_;
};  // class SelectTopicDialog
}  // namespace mapviz
#endif  // MAPVIZ_SELECT_TOPIC_DIALOG_H_
