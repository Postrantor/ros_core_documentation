---
tip: translate by baidu@2023-12-29 09:38:38
---
---
title: "`rclcpp`: CPP Client Library Overview"
---


The ROS 2 core software stack breaks down into a few discrete but related parts:

> ROS 2核心软件堆栈可分解为几个离散但相关的部分：

::: {.toctree hidden=""}
glossary.rst
:::

::: {.contents depth="2" local=""}
:::

::: danger
::: title
Danger
:::


This document is under construction and should not be used as a reference. Some things that are implemented are not documented here and other things documented here are the \"desired\" state but are not implemented in the actual code yet.

> 本文件正在编制中，不应作为参考。有些实现的东西没有在这里记录，而其他记录的东西是“期望的”状态，但还没有在实际代码中实现。
:::

# Initialization and Shutdown {#init_and_shutdown}


Before using it must be initialized exactly once per process. Initializing is done using the :cpp`rclcpp::init`{.interpreted-text role="func"} function:

> 在使用之前，每个进程必须初始化一次。初始化是使用：cpp`rclcpp:：init`｛.explored text role=“func”｝函数完成的：

```c
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
}
```


This function initializes any global resources needed by the middleware and the client library, as well as doing client library related command line argument parsing. The command line arguments can be mutated by this function, as it will remove any client library specific arguments so that later argument parsing does not have to deal with client library specific arguments. Therefore, it is generally a good idea to call this function before doing application specific command line argument parsing.

> 此函数初始化中间件和客户端库所需的任何全局资源，并执行与客户端库相关的命令行参数解析。此函数可以更改命令行参数，因为它将删除任何客户端库特定的参数，以便以后的参数解析不必处理客户端库特定参数。因此，在进行特定于应用程序的命令行参数解析之前，通常最好调用此函数。

## Shutdown and Reinitialization


Initialization can be done again, after a call to :cpp`rclcpp::shutdown`{.interpreted-text role="func"} has been completed successfully:

> 成功完成对：cpp`rclcpp:：shutdown`｛.explored text role=“func”｝的调用后，可以再次进行初始化：

```c
int main(int argc, char ** argv)
{
  while (/* condition */) {
    rclcpp::init(argc, argv, rclcpp::init::do_not_prune_arguments);
    // ...
    rclcpp::shutdown();
  }
}
```


The shutdown function causes all nodes and their constituent parts to become invalid and shutdown. It also destroys any global resources created when the initialization function was originally called.

> 关闭功能会导致所有节点及其组成部分失效并关闭。它还破坏最初调用初始化函数时创建的任何全局资源。


Note that if you intend to call :cpp`rclcpp::init`{.interpreted-text role="func"} multiple times, be sure to use the `do_not_prune_arguments` initialization option, as is done above, in order to preserve the original arguments for future invocations.

> 请注意，如果您打算多次调用：cpp`rclcpp:：init`｛.explored text role=“func”｝，请确保使用如上所述的`do_not_prune_arguments`初始化选项，以便为将来的调用保留原始参数。

## Testing for Shutdown and Reinitialization


In order to test whether or not :cpp`rclcpp::shutdown`{.interpreted-text role="func"} has been called, the :cpp`rclcpp::ok`{.interpreted-text role="func"} function can be used:

> 为了测试是否调用了：cpp`rclcpp:：shutdown`｛.explored text role=“func”｝，可以使用：cpp` rcccpp:：ok`｛.sexplored textrole=”func“｝函数：

```c
while (rclcpp::ok()) {
  // Do work...
}
```


In order to test if the system has been reinitialized, an :cpp`rclcpp::init::InitInstance`{.interpreted-text role="class"} can be retrieved using the :cpp`rclcpp::init::get_instance`{.interpreted-text role="func"} function and tested against the current instance using :cpp`rclcpp::ok`{.interpreted-text role="func"}:

> 为了测试系统是否已重新初始化，可以使用：cpp`rclcpp:：init:：InitInstance`｛.explored text role=“class”｝函数检索：cpp` rcccpp:：init：：get_instance`｛.expered text role=“func”｝，并使用：cpp `rclccpp:：ok`｛.sexplored textrole=”func“｝针对当前实例进行测试：

```c
rclcpp::init::InitInstance init_instance = rclcpp::init::get_instance();
while (rclcpp::ok(init_instance)) {
  // Do work...
}
// Either shutting down or restarting...
```


The instance can be compared to check that the reinitialization was not missed:

> 可以比较实例以检查是否未错过重新初始化：

```c
rclcpp::init::InitInstance init_instance = rclcpp::init::get_instance();
while (rclcpp::ok(init_instance)) {
  // Do work...
  if (init_instance != rclcpp::init::get_instance()) {
    // Reinitialization happened since the last rclcpp::ok check...
  }
}
```


The initialization instance is just a handle used for comparison and can be copied, moved, or destroyed without consequence.

> 初始化实例只是一个用于比较的句柄，可以被复制、移动或销毁而不会产生任何后果。

# Nodes


The main entry point to the API is the :cpp`rclcpp::Node`{.interpreted-text role="class"} class. The :cpp`rclcpp::Node`{.interpreted-text role="class"} class represents a single element in the ROS graph. Node\'s can have publishers and subscribers on topics, provide or call services, have parameters, and many other things.

> API的主要入口点是：cpp`rclcpp:：Node`｛.inprecated-text role=“class”｝类。：cpp`rclcpp:：Node`｛.explored text role=“class”｝类表示ROS图中的单个元素。节点可以有主题的发布者和订阅者，提供或调用服务，具有参数，以及许多其他内容。


Creating a node is done by calling the constructor of the :cpp`rclcpp::Node`{.interpreted-text role="class"} class and providing a name for the node (after calling :cpp`rclcpp::init`{.interpreted-text role="func"}):

> 创建节点的方法是调用：cpp`rclcpp:：node`｛.depreced text role=“class”｝类的构造函数，并提供节点的名称（在调用：cpp `rclccpp:：init`｛.epreced text-role=“func”｝之后）：

```c
#include <rclcpp/rclcpp.hpp>

// ...
{
  rclcpp::init(/* ... */);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("my_node");
}
```


It is recommended that nodes be created within a smart pointer for automatic object lifetime management, e.g. a shared pointer or a unique pointer, as demonstrated above for the former with the `make_shared` alias:: However, it can be created on the stack as well:

> 建议在智能指针内创建节点，以进行自动对象生存期管理，例如共享指针或唯一指针，如上所述，前者的别名为“make_shared”：但是，也可以在堆栈上创建：

```c
// ...
{
  rclcpp::Node node("my_node");
}
```


Since the :cpp`rclcpp::Node`{.interpreted-text role="class"} class operates on an [RAII-style pattern](http://en.cppreference.com/w/cpp/language/raii), the node is initialized and exposed to the ROS graph on construction and is shutdown and removed from the graph on destruction. Therefore nodes are scoped and must be kept around to keep the node valid. If the node object goes out of scope or is explicitly shutdown then any objects created using the node are also invalid.

> 由于：cpp`rclcpp:：Node`｛.explored text role=“class”｝类对[RAII样式模式]进行操作(http://en.cppreference.com/w/cpp/language/raii)，节点在构建时被初始化并暴露于ROS图，在销毁时被关闭并从图中移除。因此，节点是有作用域的，必须保留在周围以保持节点的有效性。如果节点对象超出范围或显式关闭，则使用该节点创建的任何对象也将无效。


The name of the node must be unique across all nodes in the ROS graph. If a node with a colliding name is created, then the conflicting node already running will be shutdown.

> 在ROS图中的所有节点中，节点的名称必须是唯一的。如果创建了具有冲突名称的节点，则已运行的冲突节点将关闭。

::: todo

Add section about namespaces within nodes, e.g. <http://wiki.ros.org/roscpp/Overview/NodeHandles#Namespaces>

> 添加关于节点内命名空间的部分，例如<http://wiki.ros.org/roscpp/Overview/NodeHandles#Namespaces>
:::

# Publish and Subscribe with Topics


One of the middleware communication primitives provided by is the publish-subscribe pattern using topics. In this pattern Messages, that are defined by the user in an interface description file, are passed between Publishers and Subscribers which are on the same Topic. A Topic is a name with an associated Message type, which determines whether or not Publishers and Subscribers should exchange messages. Publishers publish new Messages onto the Topic and any Subscribers that have subscribed to the same Topic (and with the same Message type) will receive those messages asynchronously.

> 提供的中间件通信原语之一是使用主题的发布-订阅模式。在这种模式中，由用户在接口描述文件中定义的消息在位于同一主题的发布服务器和订阅服务器之间传递。Topic是一个具有关联消息类型的名称，用于确定发布服务器和订阅服务器是否应交换消息。发布者将新消息发布到主题上，订阅了同一主题（具有相同消息类型）的任何订阅服务器都将异步接收这些消息。

## Working with Messages


Before publishing, a message must be created and filled with information. Messages are defined using the ROS IDL within `.msg` files. These files are used to generate C++ code and data structures which are used for publishing and when receiving from a subscription. Messages are namespaced by the package in which they are defined and are converted into C++ code in a conventional way. For example, a C++ header file is generated for each message:

> 在发布之前，必须创建消息并填充信息。消息是使用“.msg”文件中的ROS IDL定义的。这些文件用于生成用于发布和从订阅接收的C++代码和数据结构。消息按定义它们的包命名，并以传统方式转换为C++代码。例如，为每条消息生成一个C++头文件：

- `package_name/msg/Foo.msg` -\> `package_name/msg/foo.hpp`


And that header would contain a C++ data structure with a similar namespace:

> 该标头将包含一个具有类似名称空间的C++数据结构：

- `package_name/msg/Foo.msg` -\> `package_name::msg::Foo`


In addition to defining custom Messages, there are many predefined Messages that are defined in the common Message packages that come with ROS, for example:

> 除了定义自定义消息外，ROS附带的常见消息包中还定义了许多预定义消息，例如：

- `std_msgs/msg/String.msg`
- `geometry_msgs/msg/Point.msg`
- `builtin_msgs/msg/Time.msg`


There are many others, but throughout this document some of the standard messages will be used.

> 还有许多其他消息，但在整个文档中，将使用一些标准消息。


Generated Messages provide attribute access to the Fields so they can be accessed directly for setting and getting:

> 生成的消息提供了对字段的属性访问，因此可以直接访问字段以设置和获取：

```c
#include <geometry_msgs/msg/point.hpp>

// ...
{
  geometry_msgs::msg::Point p;
  p.x = 1;
  p.y = 2;
  p.z = 3;
  printf("Point at (%d, %d, %d)\n", p.x, p.y, p.z);
}
```


The fields can also be accessed using methods and the named parameter idiom, a.k.a. [method chaining](https://en.wikipedia.org/wiki/Method_chaining):

> 也可以使用方法和命名参数习惯用法（也称为[方法链接]）访问字段(https://en.wikipedia.org/wiki/Method_chaining):

```c
#include <geometry_msgs/msg/point.hpp>

// ...
{
  geometry_msgs::msg::Point p;
  p.set__x(1).set__y(2).set__z(3);
  printf("Point at (%d, %d, %d)\n", p.x, p.y, p.z);
}
```

::: rst-class
html-toggle
:::

### **Advanced:** Messages and Smart Pointers


Generated Messages also have some common smart pointer definitions built in, for example:

> 生成的消息还内置了一些常见的智能指针定义，例如：


- `geometry_msgs::msg::Point::SharedPtr` is equivalent to `std::shared_ptr<geometry_msgs::msg::Point>`

> -`geometry_msgs:：msg:：Point:：SharedPtr`等效于`std:：shared_ptr<geometry_msgs:：msg：：Point>`

- `geometry_msgs::msg::Point::ConstSharedPtr` is equivalent to `std::shared_ptr<const geometry_msgs::msg::Point>`

> -`geometry_msgs:：msg:：Point:：ConstSharedPtr`等效于`std:：shared_ptr<const geometry_msgs:：msg：：Point>`

- `geometry_msgs::msg::Point::UniquePtr` is equivalent to `std::unique_ptr<geometry_msgs::msg::Point>`

> -`geometry_msgs:：msg:：Point:：UniquePtr`等效于`std:：unique_ptr<geometry_msgs:：msg：：Point>`
- `geometry_msgs::msg::Point::WeakPtr` is equivalent to `std::weak_ptr<geometry_msgs::msg::Point>`

::: todo
link to exhaustive list of aliases provided.
:::

::: rst-class
html-toggle
:::

### **Advanced:** Messages and Allocators


Generated Messages are also template-able based on an Allocator. Since the fixed and dynamic sized arrays within a generated C++ Message use STL containers like `std::vector`, the generated Messages also expose an Allocator. For example, the `nav_msgs/msg/Path.msg` Message has a list of time stamped poses which are stored in a `std::vector`. You could use the Message with a custom allocator by using the template version of the Message structure that ends with a `_`:

> 生成的消息也可以基于分配器进行模板化。由于生成的C++消息中的固定大小和动态大小的数组使用STL容器，如“std:：vector”，因此生成的消息还公开了一个分配器。例如，“nav_msgs/msg/Path.msg”消息有一个带有时间戳的姿势列表，这些姿势存储在“std:：vector”中。您可以通过使用以“_”结尾的Message结构的模板版本，将Message与自定义分配器一起使用：

```c
#include <nav_msgs/msg/path.hpp>

// ...
{
  MyAllocator a;
  nav_msgs::msg::Path_<MyAllocator> path(a);
}
```

## Publishing with a `Publisher`


In publishing is achieved by creating an :cpp`rclcpp::Publisher`{.interpreted-text role="class"} object and calling :cpp`rclcpp::Publisher::publish`{.interpreted-text role="member"} with a Message as the first parameter.

> 在发布中，通过创建一个：cpp`rclcpp:：Publisher`｛.explored text role=“class”｝对象并调用：cpp` rcrccpp:：Publisher:：publish`｛.expered text rol=“member”｝并将Message作为第一个参数来实现。

::: todo
link to complete API docs for Publishers.
:::


Creating an :cpp`rclcpp::Publisher`{.interpreted-text role="class"} is done using the node and by providing a topic name, topic type, and, at a minimum, the publishing queue depth. The topic type is conveyed as a template argument to the :cpp`rclcpp::Node::advertise`{.interpreted-text role="member"} method, for example:

> 创建：cpp`rclcpp:：Publisher`｛.explored text role=“class”｝是通过使用节点并提供主题名称、主题类型以及至少发布队列深度来完成的。主题类型作为模板参数传递给：cpp`rclcpp:：Node:：advertise`｛.explored text role=“member”｝方法，例如：

```c
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>

// ...
{
  // Previously created a node of type rclcpp::Node::SharedPtr...
  rclcpp::Publisher::SharedPtr publisher = node->advertise<geometry_msgs::msg::Point>("my_topic", 10);
  geometry_msgs::msg::Point point;
  point.set__x(1).set__y(2).set__z(3);
  publisher->publish(point);
}
```

::: rst-class
html-toggle
:::

### **Advanced:** Alternative Ways to Create Publishers


An :cpp`rclcpp::Publisher`{.interpreted-text role="class"} can also be created by passing one of the built-in QoS policies:

> 也可以通过传递以下内置QoS策略之一来创建：cpp`rclcpp:：Publisher`｛.explored text role=“class”｝：

```c
// ...
{
  auto publisher = node->advertise<geometry_msgs::msg::Point>("my_topic", rclcpp::qos::profile_sensor_data);
}
```

Or with a programmatically created QoS profile based on an existing one:

```c
// ...
{
  rclcpp::qos::QoS qos = rclcpp::qos::profile_default;
  qos.depth = 10;
  auto publisher = node->advertise<geometry_msgs::msg::Point>("my_topic", qos);
}
```

Or passed directly to the method call:

```c
// ...
{
  auto publisher = node->advertise<geometry_msgs::msg::Point>(
    "my_topic",
    rclcpp::qos::profile_default | rclcpp::qos::best_effort | rclcpp::qos::depth(10));
}
```

::: todo

Consider moving alternative signatures to a separate section and link to it.

> 考虑将备选签名移动到一个单独的部分并链接到该部分。
:::


The [RAII-style pattern](http://en.cppreference.com/w/cpp/language/raii) is also used with the :cpp`rclcpp::Publisher`{.interpreted-text role="class"}, so once created it has been advertised to the ROS graph and other nodes are aware of it. Conversely, when the :cpp`rclcpp::Publisher`{.interpreted-text role="class"} is allowed to go out of scope, or is explicitly deleted, it is unadvertised from the ROS graph.

> [RAII样式](http://en.cppreference.com/w/cpp/language/raii)也与：cpp`rccpp:：Publisher`｛.explored text role=“class”｝一起使用，因此一旦创建，它就会被播发到ROS图中，其他节点也会注意到它。相反，当：cpp` rccpp::：Publisher`{.explered text rol=“class”}被允许超出范围或被显式删除时，它将不会从ROS图进行公告。
