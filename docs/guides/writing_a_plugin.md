---
title: Writing a plugin
parent: Guides
has_children: false
---

# Writing a plugin
{: .no_toc }

Mapviz displays are plugins loaded at runtime through
[pluginlib](https://github.com/ros/pluginlib). The core application knows
nothing about the data types your plugin displays — it only speaks the
abstract `mapviz::MapvizPlugin` interface — so you can add support for new
message types in your own package without modifying mapviz itself. The
`tile_map` and `multires_image` packages in this repository are examples of
plugins that live outside the core.

1. TOC
{:toc}

## Package setup

Your plugin package depends on `mapviz` (the core, **not** `mapviz_plugins`),
`pluginlib`, `rclcpp`, and whatever message packages it displays.

Declare the plugin manifest in `package.xml` so mapviz can discover it:

```xml
<export>
  <mapviz plugin="${prefix}/mapviz_plugins.xml" />
</export>
```

The manifest (`mapviz_plugins.xml`) names each display class you export:

```xml
<library path="my_plugin_library">
  <class name="my_package/my_display"
         type="my_package::MyDisplayPlugin"
         base_class_type="mapviz::MapvizPlugin">
    <description>Displays MyMessage data.</description>
  </class>
</library>
```

In `CMakeLists.txt`, build your plugin as a shared library with `AUTOMOC`
enabled (the plugin is a `QObject`), link it against mapviz and Qt, and
register the manifest:

```cmake
find_package(mapviz REQUIRED)
find_package(pluginlib REQUIRED)

set(CMAKE_AUTOMOC ON)

add_library(my_plugin_library SHARED src/my_display_plugin.cpp)
target_link_libraries(my_plugin_library
  ${mapviz_TARGETS}
  pluginlib::pluginlib
  # ... Qt5::Widgets, message packages, etc.
)

pluginlib_export_plugin_description_file(mapviz mapviz_plugins.xml)
```

Finally, export the class from your source file:

```cpp
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(my_package::MyDisplayPlugin, mapviz::MapvizPlugin)
```

When mapviz starts it logs every discovered class
(`Found mapviz plugin: my_package/my_display`), and the display appears in
the *Add display* dialog.

## The MapvizPlugin interface

Subclass `mapviz::MapvizPlugin` (see `mapviz/mapviz_plugin.hpp`) and
implement:

| Method | Purpose |
|:-|:-|
| `Initialize(QOpenGLWidget* canvas)` | One-time setup; the canvas is provided for GL work. |
| `Shutdown()` | Tear down anything not handled by the destructor. |
| `Draw(x, y, scale)` | Render with OpenGL. Called every frame while visible; the GL context is already current. |
| `Transform()` | Re-project stored data into the current target frame. Called before every draw and when the target frame changes. |
| `LoadConfig(node, path)` / `SaveConfig(emitter, path)` | Persist settings to the mapviz config file (yaml-cpp). |
| `GetConfigWidget(parent)` | Return the Qt widget shown in the config panel. |
| `PrintError/PrintInfo/PrintWarning` | Status reporting; delegate to the `Print*Helper` base-class methods with your status `QLabel`. |

Optionally override `Paint()` (and return `true` from `SupportsPainting()`)
to draw with a `QPainter` on top of the GL scene, `DrawIcon()` to render the
display-list icon, and `ClearHistory()` to drop buffered data.

Use the base class `GetTransform(...)` to look up transforms, and the
`LoadQosConfig`/`SaveQosConfig` helpers to persist subscription QoS.

The rendering and config hooks (`Draw`, `Paint`, `Transform`, `LoadConfig`,
`SaveConfig`) are **protected** virtuals. You override them, but you never call
them yourself — the framework invokes them through public wrappers
(`DrawPlugin()`, `SetTargetFrame()`, `LoadConfigPlugin()`, …) that assert
GUI-thread affinity first. See [Threading model](#threading-model).

## A skeleton plugin

The shape below is the common case: subscribe to a topic, stash each message on
the GUI thread, re-project it in `Transform()`, and render it in `Draw()`. The
comments call out which thread each method runs on — the whole contract in one
picture.

```cpp
// my_display_plugin.hpp
#include <mapviz/mapviz_plugin.hpp>

#include <QOpenGLFunctions_1_1>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>

#include <my_msgs/msg/my_message.hpp>
#include "ui_my_display_config.h"   // Qt Designer-generated config widget

namespace my_package
{
// Inherit QOpenGLFunctions_* if Draw() issues GL calls.
class MyDisplayPlugin : public mapviz::MapvizPlugin,
                        protected QOpenGLFunctions_1_1
{
  Q_OBJECT

 public:
  MyDisplayPlugin();

  // Framework lifecycle — all called on the GUI thread.
  bool Initialize(QOpenGLWidget* canvas) override;
  void Shutdown() override {}
  QWidget* GetConfigWidget(QWidget* parent) override;
  bool SupportsPainting() override { return true; }   // only if you override Paint()

 protected:
  // Customization hooks: protected virtuals the framework reaches through its
  // GUI-thread-asserting wrappers. Override them; never call them directly.
  void Draw(double x, double y, double scale) override;
  void Paint(QPainter* painter, double x, double y, double scale) override;  // optional
  void Transform() override;
  void LoadConfig(const YAML::Node& node, const std::string& path) override;
  void SaveConfig(YAML::Emitter& emitter, const std::string& path) override;

  void PrintError(const std::string& message) override;
  void PrintInfo(const std::string& message) override;
  void PrintWarning(const std::string& message) override;

 protected Q_SLOTS:
  void TopicEdited();   // wired to the config widget's topic field; GUI thread

 private:
  // Subscribe() marshals each message here, on the GUI thread, so this may
  // freely touch data_, widgets, and GetTransform() without any locking.
  void handleMessage(my_msgs::msg::MyMessage::ConstSharedPtr msg);

  Ui::my_display_config ui_;
  QWidget* config_widget_;

  rclcpp::Subscription<my_msgs::msg::MyMessage>::SharedPtr sub_;
  rmw_qos_profile_t qos_ = rmw_qos_profile_default;

  MyDecodedData data_;   // written by handleMessage(), projected by Transform(),
                         // read by Draw() — all on the GUI thread
};
}  // namespace my_package
```

```cpp
// my_display_plugin.cpp
#include <my_package/my_display_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(my_package::MyDisplayPlugin, mapviz::MapvizPlugin)

namespace my_package
{
MyDisplayPlugin::MyDisplayPlugin()
: config_widget_(new QWidget())
{
  ui_.setupUi(config_widget_);
  QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
}

bool MyDisplayPlugin::Initialize(QOpenGLWidget* canvas)
{
  canvas->makeCurrent();
  initializeOpenGLFunctions();   // required before any GL call in Draw()
  canvas->doneCurrent();
  initialized_ = true;
  return true;
}

QWidget* MyDisplayPlugin::GetConfigWidget(QWidget* parent)
{
  config_widget_->setParent(parent);
  return config_widget_;
}

void MyDisplayPlugin::TopicEdited()
{
  // Subscribe() creates the subscription (serviced by the spin thread) and
  // delivers every message to handleMessage() on the GUI thread. Overwriting
  // sub_ drops the previous subscription.
  Subscribe<my_msgs::msg::MyMessage>(
      ui_.topic->text().toStdString(), qos_, sub_,
      [this](my_msgs::msg::MyMessage::ConstSharedPtr msg) { handleMessage(msg); });
}

void MyDisplayPlugin::handleMessage(my_msgs::msg::MyMessage::ConstSharedPtr msg)
{
  // GUI thread. Copy what you need out of msg into data_; no locking.
  data_ = Decode(*msg);
}

void MyDisplayPlugin::Transform()
{
  // GUI thread. Re-project data_ into target_frame_ with GetTransform(...).
}

void MyDisplayPlugin::Draw(double x, double y, double scale)
{
  // GUI thread, GL context current. Render data_.
}

// PrintError/PrintInfo/PrintWarning delegate to the base Print*Helper()s with
// the config widget's status QLabel; LoadConfig/SaveConfig read and write the
// ui_ fields. Omitted here for brevity.
}  // namespace my_package
```

`OdometryPlugin` is the closest in-tree match to this shape.

## Accessing the ROS node

The mapviz node is **private** to the base class — there is no `node_` member
for plugins to reach into. Instead the base class exposes a small set of
accessors, so that the thread-unsafe part of ROS access (registering a
callback that then runs on the background thread) can only be done through
`Subscribe()`, which routes the result back to the GUI thread for you:

| Accessor | Use |
|:-|:-|
| `Subscribe<MsgT>(...)` | Subscribe to a topic and receive messages on the GUI thread. See [Threading model](#threading-model). |
| `Publisher<MsgT>(topic, qos)` | Create a publisher (thin wrapper over `create_publisher`). Publishing is thread-safe. |
| `Logger()` | The node's `rclcpp::Logger`. Safe to call from any thread. |
| `Now()` / `Clock()` | The node's current time / clock. |
| `TopicSource()` | A restricted, thread-safe view of the ROS graph (topic/service enumeration plus a logger) for the select-topic/service dialogs. Hand this to the dialog instead of a raw node — it grants nothing that can register a callback. |
| `NodeUnsafe()` | Escape hatch returning the raw `rclcpp::Node::SharedPtr`, for APIs the helpers don't wrap: `image_transport`, service clients, and node introspection. **You** are responsible for the thread-safety of whatever you do with it — in particular, never register a subscription or timer callback here that touches plugin state, since those run on the spin thread. Use `Subscribe()` instead. |

## Threading model

This is the one contract you must follow. Mapviz services ROS on a
background spin thread so that message traffic never stalls rendering, while
rendering, widgets, the GL context, and the (not thread safe)
`TransformManager` all belong to the GUI thread.

`Draw()`, `Paint()`, `Transform()`, and your config-widget slots all run on
the GUI thread and need no locking. The only work that happens off it is
message reception — and `Subscribe()` exists so you never have to hand-write
the thread hop.

### Subscribing

`Subscribe<MsgT>()` creates the subscription (serviced by the spin thread) and
delivers each message to a handler that runs on the **GUI thread**, so the
handler may freely touch buffers, widgets, `GetTransform()`, and config:

```cpp
// my_display_plugin.hpp
private:
  rclcpp::Subscription<my_msgs::msg::MyMessage>::SharedPtr sub_;
  // Runs on the GUI thread; owns all plugin state.
  void handleMessage(my_msgs::msg::MyMessage::ConstSharedPtr msg);
```

```cpp
// my_display_plugin.cpp — from a topic-edited slot, SelectTopic(), or LoadConfig():
Subscribe<my_msgs::msg::MyMessage>(
    topic, qos, sub_,
    [this](my_msgs::msg::MyMessage::ConstSharedPtr msg) { handleMessage(msg); });
```

The subscription is written into the handle you pass (`sub_`); reset it, or
call `Subscribe()` again, to unsubscribe. No `Q_DECLARE_METATYPE`,
`qRegisterMetaType`, signal, or `connect` is required — the marshaling is
handled internally, and messages are carried as `ConstSharedPtr` so the hop is
just a pointer copy. `OdometryPlugin` is a minimal example.

### Decoding off the GUI thread

If decoding a message is expensive (e.g. unpacking a point cloud), use the
two-type overload to do that work on the spin thread and hand only the decoded
result to the GUI thread:

```cpp
Subscribe<sensor_msgs::msg::PointCloud2, Scan>(
    topic, qos, sub_,
    &MyDisplayPlugin::DecodeScan,                       // Scan (*)(const PointCloud2::ConstSharedPtr&)
    [this](std::shared_ptr<Scan> scan) { handleScan(scan); });
```

`DecodeScan` is a plain **function pointer**, not a lambda or `std::function` —
it therefore cannot capture, which is what guarantees it can't touch plugin
state from the spin thread. It must depend only on the message; anything
configuration-dependent (coloring, tf, widgets) belongs in the GUI-thread
handler. Because it can't be a normal member function, make it `static`. Its
returned value is moved into a `shared_ptr` and delivered to your handler.
`PointCloud2Plugin` is a full example.

### Asserting the thread

`MAPVIZ_ASSERT_GUI_THREAD()` logs an error whenever it is reached off the GUI
thread (in every build, including Release, unlike a bare `Q_ASSERT`) and
additionally aborts in debug builds.

You do **not** need it in `Draw()`, `Paint()`, `Transform()`, `LoadConfig()`,
or `SaveConfig()`: the framework wrappers that reach those overrides
(`DrawPlugin()`, `PaintPlugin()`, `SetTargetFrame()`, `LoadConfigPlugin()`,
`SaveConfigPlugin()`) already assert on your behalf, and `Subscribe()` handlers
are marshaled to the GUI thread for you.

Use it at the top of any *other* entry point that assumes the GUI thread but
whose caller isn't the mapviz framework — a `QTimer` callback, an
`eventFilter()`, or a service-client response you hop back to the GUI thread
yourself:

```cpp
void MyDisplayPlugin::onRetryTimer()   // connected to a QTimer::timeout
{
  MAPVIZ_ASSERT_GUI_THREAD();
  // ... safe to touch widgets and plugin state
}
```

Notes:

- The `Print*Helper` methods are safe to call from either thread.
- `Subscribe()` does not apply backpressure: if your topic can outrun the GUI,
  coalesce (e.g. keep only the latest message) in your handler.
- Timers: use a `QTimer` (fires on the GUI thread), not
  `NodeUnsafe()->create_wall_timer()` (fires on the spin thread) — see
  `TfFramePlugin`.
- Create and reset subscriptions from GUI code (topic-edited slots,
  `LoadConfig`), as the built-in plugins do.
- If you must go around `Subscribe()` — for example `image_transport`, whose
  subscription factory `Subscribe()` can't wrap — you are back to the manual
  contract: the callback runs on the spin thread and must only decode and hand
  off to the GUI thread with a queued Qt signal. Declare the metatype for the
  carried type with `Q_DECLARE_METATYPE(...)` in your plugin header and
  `qRegisterMetaType<...>()` in the constructor. `ImagePlugin` is the one
  in-tree example.
