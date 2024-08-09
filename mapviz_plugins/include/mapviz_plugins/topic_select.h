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
#ifndef MAPVIZ_PLUGINS__TOPIC_SELECT_H_
#define MAPVIZ_PLUGINS__TOPIC_SELECT_H_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <QDialog>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include "ui_topicselect.h"

QT_BEGIN_NAMESPACE
class QLineEdit;
class QListWidget;
class QPushButton;
class QSpinBox;
QT_END_NAMESPACE

namespace mapviz_plugins
{
inline bool qosEqual(const rmw_qos_profile_t& lhs, const rmw_qos_profile_t& rhs)
{
  if (lhs.depth != rhs.depth) { return false; }
  if (lhs.history != rhs.history) { return false; }
  if (lhs.durability != rhs.durability) { return false; }
  if (lhs.reliability != rhs.reliability) { return false; }
  return true;
}

/**
 * Provides a dialog for the user to select one or more topics.
 * Several static functions are provided that can be used instead of
 * instantiating the class directly.
 */
class SelectTopicDialog : public QDialog
{
  Q_OBJECT

 public:
  /**
   * Present the user with a dialog to select a single topic and configure
   * QoS settings.  This is convenience wrapper for the common case where
   * only one datatype is allowed.
   *
   * If the user cancels the selection or doesn't make a valid
   * selection, the topic will be empty and the QoS will be the RMW default.
   */
  static std::pair<std::string, rmw_qos_profile_t> selectTopic(
    const rclcpp::Node::SharedPtr& node,
    const std::string& datatype,
    const rmw_qos_profile_t& qos,
    QWidget* parent = nullptr);

  /**
   * Present the user with a dialog to select a single topic and configure QoS
   * settings. This is a convenience wrapper for the common case where two
   * datatypes are allowed.
   *
   * If the user cancels the selection or doesn't make a valid
   * selection, the topic and datatype fields of the returned topic
   * info will be empty, and the QoS will be the RMW defaults.
   */
  static std::pair<std::string, rmw_qos_profile_t> selectTopic(
    const rclcpp::Node::SharedPtr& node,
    const std::string& datatype1,
    const std::string& datatype2,
    const rmw_qos_profile_t& qos,
    QWidget* parent = nullptr);

  /**
   * Present the user with a dialog to select a single topic and configure QoS
   * settings.
   *
   * If the user cancels the selection or doesn't make a valid
   * selection, the topic will be an empty string the QoS will be RMW defaults.
   */
  static std::pair<std::string, rmw_qos_profile_t> selectTopic(
    const rclcpp::Node::SharedPtr& node,
    const std::vector<std::string>& datatypes,
    const rmw_qos_profile_t& qos,
    QWidget* parent = nullptr);

  /**
   * Present the user with a dialog to select a multiple topics and configure QoS
   * settings.  This is a convenience wrapper for the common case where only one
   * datatype is allowed.
   *
   * If the user cancels the selection or doesn't make a valid selection, the 
   * returned vector will be empty, and the QoS will be the RMW default.
   */
  static std::pair<std::vector<std::string>, rmw_qos_profile_t> selectTopics(
    const rclcpp::Node::SharedPtr& node,
    const std::string& datatype,
    const rmw_qos_profile_t& qos,
    QWidget* parent = nullptr);

  /**
   * Present the user with a dialog to select a multiple topics and configure QoS.
   * This is a convenience wrapper for the common case where two datatypes are 
   * allowed.
   *
   * If the user cancels the selection or doesn't make a valid
   * selection, the returned vector will be empty and the QoS will be
   * the RMW default.
   */
  static std::pair<std::vector<std::string>, rmw_qos_profile_t> selectTopics(
    const rclcpp::Node::SharedPtr& node,
    const std::string& datatype1,
    const std::string& datatype2,
    const rmw_qos_profile_t& qos,
    QWidget* parent = nullptr);

  /**
   * Present the user with a dialog to select a multiple topics and configure
   * QoS settings..
   *
   * If the user cancels the selection or doesn't make a valid
   * selection, the returned vector will be empty and the QoS will be the
   * RMW defaults.
   */
  static std::pair<std::vector<std::string>, rmw_qos_profile_t> selectTopics(
    const rclcpp::Node::SharedPtr& node,
    const std::vector<std::string>& datatypes,
    const rmw_qos_profile_t& qos,
    QWidget* parent = nullptr);

  /**
   * Constructor for the SelectTopicDialog.
   */
  explicit SelectTopicDialog(
    const rclcpp::Node::SharedPtr& node,
    const rmw_qos_profile_t& qos,
    QWidget* parent = nullptr);

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
   * Returns the currently selected topic and QoS profile. If multiple
   * topics are allowed, this will only return the first selected element.
   * If there is no selection, the returned info will have an empty topic
   * name and default QoS profile.
   */
  std::pair<std::string, rmw_qos_profile_t> selectedTopic() const;
  /**
   * Returns the currently selected topics and QoS setting.  If there is no
   * selection, the returned vector will be empty and the QoS will be the RMW
   * default.
   */
  std::pair<std::vector<std::string>, rmw_qos_profile_t> selectedTopics() const;

 private:
  void timerEvent(QTimerEvent *) override;
  void closeEvent(QCloseEvent *) override;

  std::vector<std::string> filterTopics(
    const std::map<std::string, std::vector<std::string>> &) const;

 private Q_SLOTS:
  void fetchTopics();
  void updateDisplayedTopics();

 private:
  std::set<std::string> allowed_datatypes_;
  std::map<std::string, std::vector<std::string>> known_topics_;

  std::vector<std::string> displayed_topics_;
  int fetch_topics_timer_id_;
  Ui::TopicSelect *ui_;

  std::shared_ptr<rclcpp::Node> nh_;   // This may need to be a shared instance of Mapviz's node
};  // class SelectTopicDialog
}  // namespace mapviz

#endif  // MAPVIZ_PLUGINS__TOPIC_SELECT_H_
