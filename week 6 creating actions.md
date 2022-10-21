# Creating an action
> Goal: Define an action in a ROS 2 package.

do not forget  to source your ROS 2 installation first.)

<br />

crat the file action_tutorials_interfaces

<br />

~~~
mkdir -p ros2_ws/src #you can reuse existing workspace with this naming convention
cd ros2_ws/src
ros2 pkg create action_tutorials_interfaces
~~~

<br />

## task 1 Defining an action

<br />

Actions are defined in .action files of the form:
~~~
# Request
---
# Result
---
# Feedback
~~~

<br />

Create an action directory in our ROS 2 package action_tutorials_interfaces:
~~~
cd action_tutorials_interfaces
mkdir action
~~~

<br />

Within the action directory, create a file called Fibonacci.action with the following contents:
~~~
int32 order
---
int32[] sequence
---
int32[] partial_sequence
~~~
![action 1 chack](https://user-images.githubusercontent.com/91989561/197100204-076bb5a4-d047-4c1d-90f4-a07d9b6bc4ef.png)

<br />

## task 2 Building an action
This is accomplished by adding the following lines to our CMakeLists.txt before the ament_package() line, in the action_tutorials_interfaces:
~~~
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
~~~

We should also add the required dependencies to our package.xml:
~~~
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
~~~

<br />

We should now be able to build the package containing the Fibonacci action definition:
~~~
# Change to the root of the workspace
cd ~/ros2_ws
# Build
colcon build
~~~~
We can check that our action built successfully with the command line tool:
~~~
# Source our workspace
# On Windows: call install/setup.bat
. install/setup.bash
# Check that our action definition exists
ros2 interface show action_tutorials_interfaces/action/Fibonacci
~~~

![Screenshot from 2022-10-15 14-14-27](https://user-images.githubusercontent.com/91989561/197101946-3f28e0f0-2184-448e-8c9b-1d33e80b0acc.png)

<br />

# Writing an action server and client (C++)
>Goal: Implement an action server and client in C++.

## step 1 Creating the action_tutorials_cpp package
Go into the action workspace you created which we created before 
~~~
cd ~/ros2_ws/src
ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp
~~~

<br />

### 1.2 Adding in visibility control
Open up action_tutorials_cpp/include/action_tutorials_cpp/visibility_control.h, and put the following code in:
~~~
#ifndef ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
#define ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((dllexport))
    #define ACTION_TUTORIALS_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define ACTION_TUTORIALS_CPP_EXPORT __declspec(dllexport)
    #define ACTION_TUTORIALS_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACTION_TUTORIALS_CPP_BUILDING_DLL
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_EXPORT
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_IMPORT
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE ACTION_TUTORIALS_CPP_PUBLIC
  #define ACTION_TUTORIALS_CPP_LOCAL
#else
  #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((visibility("default")))
  #define ACTION_TUTORIALS_CPP_IMPORT
  #if __GNUC__ >= 4
    #define ACTION_TUTORIALS_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define ACTION_TUTORIALS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC
    #define ACTION_TUTORIALS_CPP_LOCAL
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
~~~

## 2 Writing an action server
Open up action_tutorials_cpp/src/fibonacci_action_server.cpp, and put the following code in:
~~~
#include <functional>
#include <memory>
#include <thread>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

namespace action_tutorials_cpp
{
class FibonacciActionServer : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  ACTION_TUTORIALS_CPP_PUBLIC
  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class FibonacciActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)
~~~

Next we create a class that is a derived class of rclcpp::Node:
~~~
class FibonacciActionServer : public rclcpp::Node
~~~

The constructor for the FibonacciActionServer class initializes the node name as fibonacci_action_server:
~~~
  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options)
  ~~~
  
  The constructor for the FibonacciActionServer class initializes the node name as fibonacci_action_server:
  ~~~
    explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options)
  ~~~
  
  The constructor also instantiates a new action server:
  ~~~
      this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
     
  ~~~
  
  We start with the callback for handling new goals:
  ~~~
    rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  ~~~
  Next up is the callback for dealing with cancellation:
  ~~~
    rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  ~~~
  
  All further processing and updates are done in the execute method in the new thread:
  ~~~
    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
  ~~~
  
  ### 2.2 Compiling the action server
  First we need to setup the CMakeLists.txt so that the action server is compiled. Open up action_tutorials_cpp/CMakeLists.txt, 
  and add the following right after the find_package calls:
  ~~~
  add_library(action_server SHARED
  src/fibonacci_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  ~~~
  
  And now we can compile the package. Go to the top-level of the ros2_ws, and run:
  ~~~
  colcon build
  ~~~
  
  ### 2.3 Running the action server
  Now that we have the action server built, we can run it. Source the workspace we just built (ros2_ws), and try to run the action server:
  ~~~
  ros2 run action_tutorials_cpp fibonacci_action_server
  ~~~
  
  <br />
  
  ![action server](https://user-images.githubusercontent.com/91989561/197103235-456810d2-2b7e-44c4-b9c0-a57bb4a1c15b.png)

## 3 Writing an action client
### 3.1 Writing the action client code
Open up action_tutorials_cpp/src/fibonacci_action_client.cpp, and put the following code in:
~~~
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp
{
class FibonacciActionClient : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit FibonacciActionClient(const rclcpp::NodeOptions & options)
  : Node("fibonacci_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FibonacciActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FibonacciActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class FibonacciActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)
~~~

Next we create a class that is a derived class of rclcpp::Node:
~~~
class FibonacciActionClient : public rclcpp::Node
~~~

The constructor for the FibonacciActionClient class initializes the node name as fibonacci_action_client:
~~~
  explicit FibonacciActionClient(const rclcpp::NodeOptions & options)
  : Node("fibonacci_action_client", options)
  ~~~
  
  The constructor also instantiates a new action client:
  ~~~
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci");
  ~~~
  
  An action client requires 3 things:

The templated action type name: Fibonacci.

A ROS 2 node to add the action client to: this.

The action name: 'fibonacci'.

<br />

We also instantiate a ROS timer that will kick off the one and only call to send_goal:
~~~
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FibonacciActionClient::send_goal, this));
~~~
When the timer expires, it will call send_goal:
~~~
  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FibonacciActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }
  ~~~
  This function does the following:

Cancels the timer (so it is only called once).

Waits for the action server to come up.

Instantiates a new Fibonacci::Goal.

Sets the response, feedback, and result callbacks.

Sends the goal to the server.

<br />

When the server receives and accepts the goal, it will send a response to the client. That response is handled by goal_response_callback:
~~~
  void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }
  ~~~
  
  Assuming the goal was accepted by the server, it will start processing. Any feedback to the client will be handled by the feedback_callback:
  ~~~
    void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }
  ~~~
  
  When the server is finished processing, it will return a result to the client. The result is handled by the result_callback:
  ~~~
    void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class FibonacciActionClient
~~~

### 3.2 Compiling the action client
First we need to setup the CMakeLists.txt so that the action client is compiled. 
Open up action_tutorials_cpp/CMakeLists.txt, and add the following right after the find_package calls:
~~~
add_library(action_client SHARED
  src/fibonacci_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
 ~~~
  
  And now we can compile the package. Go to the top-level of the ros2_ws, and run:
  ~~~
  colcon build
  ~~~
  
  ### 3.3 Running the action client
  Now that we have the action client built, we can run it
  ~~~
  ros2 run action_tutorials_cpp fibonacci_action_client
  ~~~
  ![action client](https://user-images.githubusercontent.com/91989561/197104415-a5f8fdc2-d9da-4add-a250-8a2fd1cb4163.png)
  
  # Composing multiple nodes in a single process
>Goal: Compose multiple nodes into a single process.
## step 1 Discover available components
To see what components are registered and available in the workspace, execute the following in a shell:
~~~
ros2 component types
~~~

The terminal will return the list of all available components:
~~~
(... components of other packages here)
composition
  composition::Talker
  composition::Listener
  composition::NodeLikeListener
  composition::Server
  composition::Client
(... components of other packages here)
~~~
## step2 Run-time composition using ROS services with a publisher and subscriber
In the first shell, start the component container:
~~~
ros2 run rclcpp_components component_container
~~~

Open the second shell and verify that the container is running via ros2 command line tools:
~~~
ros2 component list
~~~

You should see a name of the component:
~~~
/ComponentManager
~~~

In the second shell load the talker component (see talker source code):
~~~
ros2 component load /ComponentManager composition composition::Talker
ros2 component load /ComponentManager composition composition::Listener
~~~

Use the unique ID to unload the node from the component container.
~~~
ros2 component unload /ComponentManager 1 2
~~~

The terminal should return:
~~~
Unloaded component 1 from '/ComponentManager' container
Unloaded component 2 from '/ComponentManager' container
~~~

The component manager name and namespace can be remapped via standard command line arguments:
~~~
ros2 run rclcpp_components component_container --ros-args -r __node:=MyContainer -r __ns:=/ns
~~~

In a second shell, components can be loaded by using the updated container name:
~~~
ros2 component load /ns/MyContainer composition composition::Listener
~~~

## step3 Remap component names and namespaces
In the first shell, start the component container:
~~~
ros2 run rclcpp_components component_container
~~~

Some examples of how to remap names and namespaces.
~~~
ros2 component load /ComponentManager composition composition::Talker --node-name talker2
~~~

Remap namespace:
~~~
ros2 component load /ComponentManager composition composition::Talker --node-namespace /ns
~~~

Remap both:
~~~
ros2 component load /ComponentManager composition composition::Talker --node-name talker3 --node-namespace /ns2
~~~

Now use ros2 command line utility:
~~~
ros2 component list
~~~

In the console you should see corresponding entries:
~~~
/ComponentManager
   1  /talker2
   2  /ns/talker
   3  /ns2/talker3
~~~
## step 4 Passing parameter values into components
The ros2 component load command-line supports passing arbitrary parameters to the node as it is constructed. This functionality can be used as follows:
~~~
ros2 component load /ComponentManager image_tools image_tools::Cam2Image -p burger_mode:=true
~~~

The ros2 component load command-line supports passing particular options to the component manager for use when constructing the node.
As of now, the only command-line option that is supported is to instantiate a node using intra-process communication. 
This functionality can be used as follows:
~~~
ros2 component load /ComponentManager composition composition::Talker -e use_intra_process_comms:=true
~~~
![single node](https://user-images.githubusercontent.com/91989561/197106497-f59a6ba2-bbeb-4f43-aac6-fe303e34ca6a.png)
![single node image](https://user-images.githubusercontent.com/91989561/197106624-ba1e2a52-d40b-4b90-b0ed-6e6d75a9fa82.png)


