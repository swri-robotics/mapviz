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

#ifndef MAPVIZ__MAPVIZ_PLUGIN_HPP_
#define MAPVIZ__MAPVIZ_PLUGIN_HPP_

// ROS libraries
#include <swri_transform_util/transform.h>
#include <swri_transform_util/transform_manager.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <mapviz/topic_source.hpp>
#include <mapviz/widgets.hpp>
#include <yaml-cpp/yaml.h>

// QT libraries
#include <QWidget>
#include <QObject>
#include <QOpenGLWidget>
#include <QThread>

// C++ standard libraries
#include <functional>
#include <memory>
#include <mutex>
#include <string>


#include "mapviz/stopwatch.hpp"

namespace mapviz
{

/**
 * Asserts that the calling code is running on the GUI thread (the thread that
 * owns this plugin object).  Plugin state, widgets, the GL context, and
 * tf_manager_ may only be touched from that thread.
 *
 * The framework entry points that reach Draw(), Paint(), Transform(),
 * LoadConfig(), and SaveConfig() (DrawPlugin(), PaintPlugin(),
 * SetTargetFrame(), LoadConfigPlugin(), and SaveConfigPlugin()) already
 * assert this, so plugin overrides of those methods do not need it.  Use it
 * at the top of any *other* entry point that assumes the GUI thread -- e.g. a
 * QTimer callback or an eventFilter() -- where the caller isn't the mapviz
 * framework.
 *
 * Unlike a bare Q_ASSERT (which is compiled out when QT_NO_DEBUG is defined,
 * i.e. in the Release builds that ROS packages ship), this always logs an
 * error when the check fails, and additionally aborts via Q_ASSERT in debug
 * builds.  It must be used from a non-static member function of a MapvizPlugin
 * (it relies on thread() and Logger()).
 */
#define MAPVIZ_ASSERT_GUI_THREAD()                                             \
  do {                                                                         \
    if (QThread::currentThread() != this->thread()) {                          \
      RCLCPP_ERROR(this->Logger(),                                             \
        "%s: called off the GUI thread; mapviz plugin state is not "           \
        "thread-safe and must only be accessed on the GUI thread", __func__);  \
      Q_ASSERT(!"mapviz: this function must run on the GUI thread");           \
    }                                                                          \
  } while (0)
class MapvizPlugin : public QObject
{
  Q_OBJECT
public:
  ~MapvizPlugin() override = default;

  /**
   * Threading model
   *
   * ROS message callbacks run on a background spin thread, while rendering,
   * widgets, and the (not thread safe) TransformManager belong to the GUI
   * thread.  Plugins bridge the two with queued signals: the subscription
   * callback does any expensive, configuration-independent decoding on
   * local data, then emits the result through a signal connected to a slot
   * on this object.  Because the emitting thread differs from the receiving
   * object's thread, Qt delivers it as a queued event on the GUI thread,
   * where all plugin state may be used without locking.  Carry results in
   * shared_ptrs (declared with Q_DECLARE_METATYPE and registered with
   * qRegisterMetaType) so queued copies stay cheap.  See OdometryPlugin
   * (simple handoff) and PointCloud2Plugin (heavy decode in the callback)
   * for the pattern.
   *
   * The rules this imposes on plugin code:
   *  - Subscription callbacks must not touch widgets, the GL context,
   *    tf_manager_, or any state shared with the GUI thread; they decode
   *    and emit.
   *  - Everything else (Draw/Paint/Transform, config slots, GetTransform)
   *    runs on the GUI thread and needs no synchronization.
   */

  virtual bool Initialize(
      std::shared_ptr<tf2_ros::Buffer> tf_buffer,
      std::shared_ptr<tf2_ros::TransformListener> tf_listener,
      swri_transform_util::TransformManagerPtr tf_manager,
      QOpenGLWidget* canvas)
  {
    tf_buf_ = tf_buffer;
    tf_ = tf_listener;
    tf_manager_ = tf_manager;
    return Initialize(canvas);
  }

  virtual void Shutdown() = 0;

  virtual void ClearHistory() {}

  void SetUseLatestTransforms(bool value)
  {
    if (value != use_latest_transforms_) {
      use_latest_transforms_ = value;
      Q_EMIT UseLatestTransformsChanged(use_latest_transforms_);
    }
  }

  void SetName(const std::string& name) { name_ = name; }

  std::string Name() const { return name_; }

  void SetType(const std::string& type) { type_ = type; }

  std::string Type() const { return type_; }

  int DrawOrder() const { return draw_order_; }

  void SetDrawOrder(int order)
  {
    if (draw_order_ != order) {
      draw_order_ = order;
      Q_EMIT DrawOrderChanged(draw_order_);
    }
  }

  virtual void SetNode(rclcpp::Node& node)
  {
    // node_ = node;
    node_ = node.shared_from_this();
  }

  void DrawPlugin(double x, double y, double scale)
  {
    MAPVIZ_ASSERT_GUI_THREAD();
    if (visible_ && initialized_) {
      meas_transform_.start();
      Transform();
      meas_transform_.stop();

      meas_draw_.start();
      Draw(x, y, scale);
      meas_draw_.stop();
    }
  }

  void PaintPlugin(QPainter* painter, double x, double y, double scale)
  {
    MAPVIZ_ASSERT_GUI_THREAD();
    if (visible_ && initialized_) {
      meas_transform_.start();
      Transform();
      meas_transform_.stop();

      meas_paint_.start();
      Paint(painter, x, y, scale);
      meas_paint_.stop();
    }
  }

  void SetTargetFrame(const std::string& frame_id)
  {
    MAPVIZ_ASSERT_GUI_THREAD();
    if (frame_id != target_frame_) {
      target_frame_ = frame_id;

      meas_transform_.start();
      Transform();
      meas_transform_.stop();

      Q_EMIT TargetFrameChanged(target_frame_);
    }
  }

  bool Visible() const { return visible_; }

  void SetVisible(bool visible)
  {
    if (visible_ != visible) {
      visible_ = visible;
      Q_EMIT VisibleChanged(visible_);
    }
  }

  bool GetTransform(
    const rclcpp::Time& stamp,
    swri_transform_util::Transform& transform,
    bool use_latest_transforms = true)
  {
    return GetTransform(source_frame_, stamp, transform, use_latest_transforms);
  }

  bool GetTransform(const std::string& source,
    const rclcpp::Time& stamp,
    swri_transform_util::Transform& transform,
    bool use_latest_transforms = true)
  {
    if (!initialized_) {
      return false;
    }

    tf2::TimePoint time;
    rclcpp::Time now = node_->now();

    if (use_latest_transforms_ && use_latest_transforms) {
      time = tf2::TimePointZero;
    }
    else
    {
      time = tf2::timeFromSec(stamp.seconds());
    }

    if (tf_manager_->GetTransform(
        target_frame_,
        source,
        time,
        transform))
    {
      return true;
    } else if (time != tf2::TimePointZero) {
      rclcpp::Duration elapsed = now - stamp;

      if (elapsed.seconds() < 0.1)
      {
        // If the stamped transform failed because it is too recent, find the
        // most recent transform in the cache instead.
        if (tf_manager_->GetTransform(
            target_frame_,
            source,
            tf2::TimePointZero,
            transform))
        {
          return true;
        }
      }
    }

    return false;
  }

  /**
   * The framework's entry points to LoadConfig()/SaveConfig().  Like
   * DrawPlugin()/PaintPlugin(), these assert the GUI thread on behalf of the
   * plugin's overrides, which read and write widgets.
   */
  void LoadConfigPlugin(const YAML::Node& load, const std::string& path)
  {
    MAPVIZ_ASSERT_GUI_THREAD();
    LoadConfig(load, path);
  }

  void SaveConfigPlugin(YAML::Emitter& emitter, const std::string& path)
  {
    MAPVIZ_ASSERT_GUI_THREAD();
    SaveConfig(emitter, path);
  }

  virtual QWidget* GetConfigWidget(QWidget* /* parent */) { return nullptr; }

  virtual void PrintError(const std::string& message) = 0;
  virtual void PrintInfo(const std::string& message) = 0;
  virtual void PrintWarning(const std::string& message) = 0;

  void SetIcon(IconWidget* icon) { icon_ = icon; }

  void PrintMeasurements()
  {
    std::string header = type_ + " (" + name_ + ")";
    meas_transform_.printInfo(node_->get_logger(), header + " Transform()");
    meas_paint_.printInfo(node_->get_logger(), header + " Paint()");
    meas_draw_.printInfo(node_->get_logger(), header + " Draw()");
  }

  void PrintErrorHelper(
    QLabel *status_label,
    const std::string& message,
    double throttle = 0.0);
  void PrintInfoHelper(
    QLabel *status_label,
    const std::string& message,
    double throttle = 0.0);
  void PrintWarningHelper(
    QLabel *status_label,
    const std::string& message,
    double throttle = 0.0);

public Q_SLOTS:
  virtual void DrawIcon() {}

  /**
   * Override this to return "true" if you want QPainter support for your
   * plugin.
   */
  virtual bool SupportsPainting()
  {
    return false;
  }

Q_SIGNALS:
  void DrawOrderChanged(int draw_order);
  void SizeChanged();
  void TargetFrameChanged(const std::string& target_frame);
  void UseLatestTransformsChanged(bool use_latest_transforms);
  void VisibleChanged(bool visible);


protected:
  /**
   * Customization hooks, deliberately protected: the framework only invokes
   * them through the public wrappers (DrawPlugin(), PaintPlugin(),
   * SetTargetFrame(), LoadConfigPlugin(), SaveConfigPlugin()), which assert
   * the GUI thread before dispatching, so nothing can call these off-thread
   * through the base class.
   */

  /**
   * Draws on the Mapviz canvas using OpenGL commands; this will be called
   * before Paint();
   */
  virtual void Draw(double x, double y, double scale) = 0;

  /**
   * Draws on the Mapviz canvas using a QPainter; this is called after Draw().
   * You only need to implement this if you're actually using a QPainter.
   */
  virtual void Paint(QPainter* /* painter */, double /* x */,
                     double /* y */, double /* scale */) {}

  virtual void Transform() = 0;

  virtual void LoadConfig(const YAML::Node& load, const std::string& path) = 0;
  virtual void SaveConfig(YAML::Emitter& emitter, const std::string& path) = 0;

  /**
   * Subscribe to @p topic, delivering every message to @p on_gui_thread on the
   * GUI thread.  The subscription is created on the node's default callback
   * group, so it is serviced by the background ROS spin thread; this helper
   * marshals each message onto the GUI thread through a queued invocation, so
   * @p on_gui_thread may freely touch plugin state, widgets, and tf_manager_
   * without any locking.  Use this overload when the per-message work is cheap
   * (the common "store it and repaint" case).
   *
   * The subscription is written into @p out_sub; reset it (or overwrite it via
   * another Subscribe call) to unsubscribe.
   */
  template <typename MsgT>
  void Subscribe(
      const std::string& topic,
      const rmw_qos_profile_t& qos,
      typename rclcpp::Subscription<MsgT>::SharedPtr& out_sub,
      std::function<void(typename MsgT::ConstSharedPtr)> on_gui_thread)
  {
    rclcpp::QoS ros_qos(rclcpp::QoSInitialization::from_rmw(qos), qos);
    out_sub = node_->create_subscription<MsgT>(
        topic, ros_qos,
        [this, cb = std::move(on_gui_thread)](typename MsgT::ConstSharedPtr msg)
        {
          // Runs on the ROS spin thread.  Hand the message to the GUI thread
          // and return immediately; using 'this' as the invocation context
          // means Qt discards the event if the plugin is destroyed, and
          // because teardown runs on the GUI thread there is no race.
          QMetaObject::invokeMethod(
              this, [cb, msg]() { cb(msg); }, Qt::QueuedConnection);
        });
  }

  /**
   * Subscribe to @p topic, running @p decode on the background ROS spin thread
   * and delivering its result to @p on_gui_thread on the GUI thread.  Use this
   * overload when decoding a message is expensive (e.g. unpacking a point
   * cloud) and you want that work off the GUI thread.
   *
   * @p decode is a plain function pointer, not a std::function: it therefore
   * cannot capture, which structurally prevents it from touching plugin state
   * from the spin thread.  It must depend only on the message (any
   * configuration-dependent work belongs in @p on_gui_thread).  Its result is
   * moved into a shared_ptr and marshaled to the GUI thread.
   */
  template <typename MsgT, typename DecodedT>
  void Subscribe(
      const std::string& topic,
      const rmw_qos_profile_t& qos,
      typename rclcpp::Subscription<MsgT>::SharedPtr& out_sub,
      DecodedT (*decode)(const typename MsgT::ConstSharedPtr&),
      std::function<void(std::shared_ptr<DecodedT>)> on_gui_thread)
  {
    rclcpp::QoS ros_qos(rclcpp::QoSInitialization::from_rmw(qos), qos);
    out_sub = node_->create_subscription<MsgT>(
        topic, ros_qos,
        [this, decode, cb = std::move(on_gui_thread)]
        (typename MsgT::ConstSharedPtr msg)
        {
          // Runs on the ROS spin thread.  'decode' may only look at the
          // message; the decoded result is handed to the GUI thread.
          auto decoded = std::make_shared<DecodedT>(decode(msg));
          QMetaObject::invokeMethod(
              this, [cb, decoded]() { cb(decoded); }, Qt::QueuedConnection);
        });
  }

  /**
   * Create a publisher on the mapviz node.  Publishing is thread-safe, so this
   * may be called from the GUI thread.  Arguments are forwarded to
   * rclcpp::Node::create_publisher().
   */
  template <typename MsgT, typename... Args>
  typename rclcpp::Publisher<MsgT>::SharedPtr Publisher(Args&&... args)
  {
    return node_->create_publisher<MsgT>(std::forward<Args>(args)...);
  }

  /// The mapviz node's logger.  Safe to call from any thread.
  rclcpp::Logger Logger() const
  {
    return node_ ? node_->get_logger() : rclcpp::get_logger("mapviz");
  }

  /// The current time from the mapviz node's clock.
  rclcpp::Time Now() const { return node_->now(); }

  /// The mapviz node's clock.
  rclcpp::Clock::SharedPtr Clock() const { return node_->get_clock(); }

  /**
   * Direct access to the underlying node, for APIs the safe helpers above do
   * not wrap (image_transport, service clients, node introspection).  You are
   * responsible for the thread-safety of anything you do with it: in
   * particular, never register a subscription/timer callback here that
   * touches plugin state, since those run on the background spin thread --
   * use Subscribe() instead.  For the topic/service selection dialogs, use
   * TopicSource() instead of handing over the node.
   */
  rclcpp::Node::SharedPtr NodeUnsafe() { return node_; }

  /**
   * A restricted view of the node for topic/service discovery (e.g. the
   * SelectTopicDialog): thread-safe, read-only graph queries and a logger,
   * with none of NodeUnsafe()'s caveats.
   */
  mapviz::TopicSource TopicSource() const
  {
    return {
      [node = node_] { return node->get_topic_names_and_types(); },
      [node = node_] { return node->get_service_names_and_types(); },
      Logger()
    };
  }

  bool initialized_;
  bool visible_;

  QOpenGLWidget* canvas_;
  IconWidget* icon_;

  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_;
  swri_transform_util::TransformManagerPtr tf_manager_;

  std::string target_frame_;
  std::string source_frame_;
  std::string type_;
  std::string name_;

  bool use_latest_transforms_;

  int draw_order_;

  virtual bool Initialize(QOpenGLWidget* canvas) = 0;

  MapvizPlugin() :
    initialized_(false),
    visible_(true),
    canvas_(nullptr),
    icon_(nullptr),
    node_(nullptr),
    tf_(),
    target_frame_(""),
    source_frame_(""),
    use_latest_transforms_(false),
    draw_order_(0)
  {}

  void LoadQosConfig(const YAML::Node& node, rmw_qos_profile_t& qos, const std::string prefix = "") const
  {
    if (node[prefix + "qos_depth"])
    {
      qos.depth = node[prefix + "qos_depth"].as<int>();
    }

    if (node[prefix + "qos_history"])
    {
      qos.history = static_cast<rmw_qos_history_policy_e>(node[prefix + "qos_history"].as<int>());
    }

    if (node[prefix + "qos_reliability"])
    {
      qos.reliability = static_cast<rmw_qos_reliability_policy_e>(node[prefix + "qos_reliability"].as<int>());
    }

    if (node[prefix + "qos_durability"])
    {
      qos.durability = static_cast<rmw_qos_durability_policy_e>(node[prefix + "qos_durability"].as<int>());
    }
  }

  void SaveQosConfig(YAML::Emitter& emitter, const rmw_qos_profile_t& qos, const std::string prefix = "") const
  {
    emitter << YAML::Key << prefix + "qos_depth" << YAML::Value << qos.depth;
    emitter << YAML::Key << prefix + "qos_history" << YAML::Value << qos.history;
    emitter << YAML::Key << prefix + "qos_reliability" << YAML::Value << qos.reliability;
    emitter << YAML::Key << prefix + "qos_durability" << YAML::Value << qos.durability;
  }

  // Dealing with YAML frequently requires trimming whitespace from strings
  inline std::string TrimString(const std::string& str)
  {
    auto begin = str.begin();
    auto end = str.end();

    // Trim leading whitespace
    while (begin != end && std::isspace(*begin))
    {
      ++begin;
    }

    // Trim trailing whitespace
    if (begin != end)
    {
      do
      {
        --end;
      } while (std::isspace(*end));
      ++end;
    }

    return std::string(begin, end);
  }

private:
  // The mapviz node.  Private so plugins reach it only through the accessors
  // above (Subscribe(), Publisher(), Logger(), Now(), Clock(), NodeUnsafe()),
  // which steer subscription callbacks onto the GUI thread and make raw,
  // thread-unsafe access explicit at the call site.
  std::shared_ptr<rclcpp::Node> node_;

  // Collect basic profiling info to know how much time each plugin
  // spends in Transform(), Paint(), and Draw().
  Stopwatch meas_transform_;
  Stopwatch meas_paint_;
  Stopwatch meas_draw_;

  // Deduplicates status messages; the print helpers can be called from the
  // ROS spin thread, so the label text can't be read there for comparison.
  std::mutex status_mutex_;
  std::string last_status_msg_;

  // Returns true the first time each unique message is seen; used so the
  // status label and log are only updated when the message changes.
  bool StatusMessageChanged(const std::string& message)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (message == last_status_msg_) {
      return false;
    }
    last_status_msg_ = message;
    return true;
  }
};
typedef std::shared_ptr<MapvizPlugin> MapvizPluginPtr;

// Implementation
//
// The print helpers may be called from the ROS spin thread, but QLabel can
// only be touched from the GUI thread, so the label update is posted to the
// label's thread with a queued invocation when necessary.  The label is used
// as the invocation context so pending updates are dropped if it is deleted.
inline void MapvizPlugin::PrintErrorHelper(QLabel *status_label, const std::string &message,
                                            double throttle)
{
    if (!StatusMessageChanged(message)) {
      return;
    }

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("mapviz");
    if (throttle > 0.0) {
        RCLCPP_ERROR(logger, "Error: %s", message.c_str());
    } else {
        RCLCPP_ERROR(logger, "%s", message.c_str());
    }
    auto update_label = [status_label, message]() {
      QPalette p(status_label->palette());
      p.setColor(QPalette::Text, Qt::red);
      status_label->setPalette(p);
      status_label->setText(message.c_str());
    };
    if (QThread::currentThread() == status_label->thread()) {
      update_label();
    } else {
      QMetaObject::invokeMethod(status_label, update_label, Qt::QueuedConnection);
    }
}

inline void MapvizPlugin::PrintInfoHelper(QLabel *status_label, const std::string &message,
                                          double throttle)
{
    if (!StatusMessageChanged(message)) {
      return;
    }

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("mapviz");
    if (throttle > 0.0) {
        RCLCPP_INFO(logger, "%s", message.c_str());
    } else {
        RCLCPP_INFO(logger, "%s", message.c_str());
    }
    auto update_label = [status_label, message]() {
      QPalette p(status_label->palette());
      p.setColor(QPalette::Text, Qt::darkGreen);
      status_label->setPalette(p);
      status_label->setText(message.c_str());
    };
    if (QThread::currentThread() == status_label->thread()) {
      update_label();
    } else {
      QMetaObject::invokeMethod(status_label, update_label, Qt::QueuedConnection);
    }
}

inline void MapvizPlugin::PrintWarningHelper(QLabel *status_label, const std::string &message,
                                              double throttle)
{
    if (!StatusMessageChanged(message)) {
      return;
    }

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("mapviz");
    if (throttle > 0.0) {
        RCLCPP_WARN(logger, "%s", message.c_str());
    } else {
        RCLCPP_WARN(logger, "%s", message.c_str());
    }
    auto update_label = [status_label, message]() {
      QPalette p(status_label->palette());
      p.setColor(QPalette::Text, Qt::darkYellow);
      status_label->setPalette(p);
      status_label->setText(message.c_str());
    };
    if (QThread::currentThread() == status_label->thread()) {
      update_label();
    } else {
      QMetaObject::invokeMethod(status_label, update_label, Qt::QueuedConnection);
    }
}

}   // namespace mapviz

#endif  // MAPVIZ__MAPVIZ_PLUGIN_HPP_

