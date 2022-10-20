# Creating custom msg and srv files

>Goal: Define custom interface files (.msg and .srv) and use them with Python and C++ nodes.
## step 1 Create a new package
For this tutorial you will be creating custom .msg and .srv files in their own package, 
and then utilizing them in a separate package. Both packages should be in the same workspace.
as we created the pub/sub and service/client packages earler be sure be same workspaces
~~~
ros2 pkg create --build-type ament_cmake tutorial_interfaces
~~~
It is good practice to keep .msg and .srv files in their own directories within a package. Create the directories in ros2_ws/src/tutorial_interfaces:
~~~
mkdir msg

mkdir srv
~~~
## step 2 Create custom definitions
### 2.1 msg definition
In the tutorial_interfaces/msg directory you just created, make a new file called Num.msg with one line of code declaring its data structure:
~~~
int64 num
~~~
>This is a custom message that transfers a single 64-bit integer called num.
Also in the tutorial_interfaces/msg directory you just created, make a new file called Sphere.msg with the following content:
~~~
geometry_msgs/Point center
float64 radius
~~~
This custom message uses a message from another message package (geometry_msgs/Point in this case).
### 2.2 srv definition
>Back in the tutorial_interfaces/srv directory you just created, make a new file called AddThreeInts.srv with the following request and response structure:
~~~
int64 a
int64 b
int64 c
---
int64 sum
~~~

<br />

![confirm msg and srv creation](https://user-images.githubusercontent.com/91989561/197029374-779a7528-4543-4b7c-9bff-cd2a23f7ac7b.png)
## step 3 CMakeLists.txt
>To convert the interfaces you defined into language-specific code (like C++ and Python) so that they can be used in those languages, 
add the following lines to CMakeLists.txt:
~~~
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
~~~
## step 4 package.xml
>Because the interfaces rely on rosidl_default_generators for generating language-specific code, you need to declare a dependency on it. 
The <exec_depend> tag is used to specify runtime or 
execution-stage dependencies and the rosidl_interface_packages is the name of the dependency group to which the package belongs,
declared using the <member_of_group> tag.

<br />
by using gedit open the package.xml

<br />

~~~
<depend>geometry_msgs</depend>

<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
~~~

<br />

## step 5 Build the tutorial_interfaces package

with the going back to  (~/ros2_ws), run the following command:

<br />

~~~
colcon build --packages-select tutorial_interfaces
~~~

<br />

Now the interfaces will be discoverable by other ROS 2 packages.

## step 6 Confirm msg and srv creation

go to the new terminal and dont forget soursing following command from within your workspace (ros2_ws)

<br />

~~~
. install/setup.bash
~~~

<br />

Now you can confirm that your interface creation worked by using the ros2 interface show command:

<br />

~~~
ros2 interface show tutorial_interfaces/msg/Num
~~~

<br />

should return:

<br />

~~~
int64 num
~~~

<br />

then

<br />

~~~
ros2 interface show tutorial_interfaces/msg/Sphere
~~~

<br />

![confirm msg and srv creation](https://user-images.githubusercontent.com/91989561/197029374-779a7528-4543-4b7c-9bff-cd2a23f7ac7b.png)

<br />

after that

<br />

~~~
ros2 interface show tutorial_interfaces/srv/AddThreeInts
~~~

<br />

should return 
~~~
int64 a
int64 b
int64 c
---
int64 sum
~~~


![confirm msg and srv creation](https://user-images.githubusercontent.com/91989561/197029374-779a7528-4543-4b7c-9bff-cd2a23f7ac7b.png)

<br />

## step 7 Test the new interfaces
For this step you can use the packages you created in previous tutorials. A few simple modifications to the nodes,
CMakeLists and package files will allow you to use your new interfaces.

### 7.1 Testing Num.msg with pub/sub

With some slight modifications to the publisher/subscriber package created in a previous tutorial (C++ or Python), you can see Num.msg in action. 
Since you’ll be changing the standard string msg to a numerical one, the output will be slightly different.

<br />

Publisher:
~~~import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num                            # CHANGE


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Num, 'topic', 10)  # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Num()                                                # CHANGE
        msg.num = self.i                                           # CHANGE
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.num)       # CHANGE
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    ~~~
    
    <br />
    
    Subscriber:
    ~~~
    import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num                        # CHANGE


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Num,                                               # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info('I heard: "%d"' % msg.num)  # CHANGE


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    ~~~
    
    <br />
    
    CMakeLists.txt: use only use C ++
    ~~~
    #...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)                      # CHANGE

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp tutorial_interfaces)    # CHANGE

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp tutorial_interfaces)  # CHANGE

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
~~~

<br />

package.xml:
~~~
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)                      # CHANGE

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp tutorial_interfaces)    # CHANGE

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp tutorial_interfaces)  # CHANGE

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
~~~
and add command bellow
~~~
<exec_depend>tutorial_interfaces</exec_depend>
~~~

After making the above edits and saving all the changes, build the package:
~~~
colcon build --packages-select py_pubsub
~~~
Then open two new terminals, source ros2_ws in each, and run:
~~~
ros2 run py_pubsub talker
~~~

<br />

![talker msg srv](https://user-images.githubusercontent.com/91989561/197034495-32b3aa1b-1399-4b33-b54d-d3920087b4c2.png)

<br />

~~~
ros2 run py_pubsub listener
~~~

<br />

![liner msg and srv](https://user-images.githubusercontent.com/91989561/197034672-cd5ed9de-6acc-43b5-ae1a-003890ec46a8.png)
 
 <br />
 
 ### 7.2 Testing AddThreeInts.srv with service/client
 With some slight modifications to the service/client package created in a previous tutorial (C++ or Python),
 you can see AddThreeInts.srv in action. Since you’ll be changing the original two integer request srv to a three integer request srv,
 the output will be slightly different.
 
 Service:
 ~~~
 from tutorial_interfaces.srv import AddThreeInts                                                           # CHANGE

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)       # CHANGE

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c                                                   # CHANGE
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c))  # CHANGE

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
    ~~~
    
    <br />
    
    Client:
    ~~~
    from tutorial_interfaces.srv import AddThreeInts                            # CHANGE
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()                                   # CHANGE

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.req.c = int(sys.argv[3])                                       # CHANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_three_ints: for %d + %d + %d = %d' %                                # CHANGE
                    (minimal_client.req.a, minimal_client.req.b, minimal_client.req.c, response.sum))  # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    from tutorial_interfaces.srv import AddThreeInts                            # CHANGE
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()                                   # CHANGE

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.req.c = int(sys.argv[3])                                       # CHANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_three_ints: for %d + %d + %d = %d' %                                # CHANGE
                    (minimal_client.req.a, minimal_client.req.b, minimal_client.req.c, response.sum))  # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    ~~~
    
    CMakeLists.txt: Add the following lines (C++ only):
    ~~~
    #...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)         # CHANGE

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
  rclcpp tutorial_interfaces)                      # CHANGE

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp tutorial_interfaces)                      # CHANGE

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
~~~

package.xml:
Add the following line:
~~~
<exec_depend>tutorial_interfaces</exec_depend>
~~~

<br />

After making the above edits and saving all the changes, build the package:
~~~
colcon build --packages-select py_srvcli
~~~

<br />

we need to open two new terminals, source ros2_ws in each, and run:
~~~
ros2 run py_srvcli service
~~~

<br />

~~~
ros2 run py_srvcli client 2 3 1
~~~

<br />

![service msgand srv](https://user-images.githubusercontent.com/91989561/197036005-c30eb86d-dd06-4005-a8a1-e06939009207.png)

<br />

![client msg and srv](https://user-images.githubusercontent.com/91989561/197036148-efffcc71-3c5c-42ba-9ee0-e95d374e647a.png)
