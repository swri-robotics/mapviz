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

## Threading model

This is the one contract you must follow. Mapviz services ROS on a
background spin thread so that message traffic never stalls rendering, while
rendering, widgets, the GL context, and the (not thread safe)
`TransformManager` all belong to the GUI thread.

**Subscription callbacks run on the spin thread and must not touch widgets,
the GL context, `tf_manager_`/`GetTransform()`, or any state shared with the
GUI thread.** A callback should do at most expensive, configuration-independent
decoding on local data, then hand the result to the GUI thread with a queued
signal. Everything else — `Draw()`, `Paint()`, `Transform()`, config-widget
slots — runs on the GUI thread and needs no locking.

The handoff uses an ordinary Qt signal/slot pair. Because the signal is
emitted from a different thread than the one your plugin object lives on, Qt
automatically delivers it as a queued event on the GUI thread:

```cpp
// my_display_plugin.hpp
class MyDisplayPlugin : public mapviz::MapvizPlugin
{
  Q_OBJECT
  // ...

Q_SIGNALS:
  // Emitted from the ROS spin thread; delivered as a queued connection to
  // handleMessage() on the GUI thread, which owns all plugin state.
  void MessageReceived(const my_msgs::msg::MyMessage::ConstSharedPtr msg);

private Q_SLOTS:
  void handleMessage(const my_msgs::msg::MyMessage::ConstSharedPtr msg);

private:
  void messageCallback(const my_msgs::msg::MyMessage::ConstSharedPtr msg);
};

// Required so the shared_ptr can be carried by a queued emission.
Q_DECLARE_METATYPE(my_msgs::msg::MyMessage::ConstSharedPtr)
```

```cpp
// my_display_plugin.cpp — in the constructor:
qRegisterMetaType<my_msgs::msg::MyMessage::ConstSharedPtr>(
    "my_msgs::msg::MyMessage::ConstSharedPtr");
QObject::connect(this, &MyDisplayPlugin::MessageReceived,
                 this, &MyDisplayPlugin::handleMessage);

// The subscription callback: decode and emit, nothing else.
void MyDisplayPlugin::messageCallback(
    const my_msgs::msg::MyMessage::ConstSharedPtr msg)
{
  Q_EMIT MessageReceived(msg);
}

// The slot: runs on the GUI thread, free to use tf, widgets, and config.
void MyDisplayPlugin::handleMessage(
    const my_msgs::msg::MyMessage::ConstSharedPtr msg)
{
  // update buffers, call GetTransform(), touch ui_, etc.
}
```

Emit `ConstSharedPtr`s (or a `shared_ptr` to your own decoded struct) so the
queued copy is just a pointer. For a simple example see `OdometryPlugin`; for
a plugin that does heavy per-message decoding in the callback before emitting
see `PointCloud2Plugin`.

Notes:

- Declare metatypes for your **own** types in your own headers. If several
  headers in one library share a type, put the declaration in one common
  header (see `mapviz_plugins/ros_metatypes.hpp`) — `AUTOMOC` compiles a
  library's moc files in a single translation unit, so a type declared in
  two of its headers is a redefinition. Two *separate* plugin libraries
  registering the same type is fine.
- The `Print*Helper` methods are safe to call from either thread.
- Queued connections do not apply backpressure: if your topic can outrun the
  GUI, coalesce in the callback (e.g. store the latest message and emit a
  lightweight notification) instead of emitting every message.
- Timers: use a `QTimer` (fires on the GUI thread), not
  `node_->create_wall_timer()` (fires on the spin thread) — see
  `TfFramePlugin`.
- Create and reset subscriptions from GUI code (topic-edited slots,
  `LoadConfig`), as the built-in plugins do.
