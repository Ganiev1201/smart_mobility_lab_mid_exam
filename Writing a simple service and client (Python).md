# Create Writing a simple service and client (Python).md
>Open a new terminal and source your ROS 2 installation so that ros2 commands will work.
~~~
source /opt/ros/humble/setup.bash
~~~
## Step 1: Create a package

<br />
we need to ros2_ws directory created in a previous tutorial.

<br />

Recall that packages should be created in the src directory, not the root of the workspace. Navigate into ros2_ws/src and create a new package:
~~~
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
~~~

The --dependencies argument will automatically add the necessary dependency lines to package.xml. example_interfaces is the package that includes the 
.srv file you will need to structure your requests and responses:
~~~
int64 a
int64 b
---
int64 sum
~~~
### 1.1 Update package.xml
gedit the package.xml. make sure to add the description, maintainer email and name, and license information to package.xml.
~~~
<description>Python client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
~~~
### 1.2 Update setup.py
Add the same information to the setup.py file for the maintainer, maintainer_email, description and license fields:
~~~
maintainer='Your Name',
maintainer_email='you@email.com',
description='Python client server tutorial',
license='Apache License 2.0',
~~~
## step 2 Write the service node
Inside the ros2_ws/src/py_srvcli/py_srvcli directory, create a new file called service_member_function.py and paste the following code within:
~~~
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
~~~
### 2.1 Examine the code
The first import statement imports the AddTwoInts service type from the example_interfaces package. 
The following import statement imports the ROS 2 Python client library,and specifically the Node class
~~~
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
~~~
The MinimalService class constructor initializes the node with the name minimal_service. Then, it creates a service and defines the type, name, and callback.
~~~
def __init__(self):
    super().__init__('minimal_service')
    self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
~~~
The definition of the service callback receives the request data, sums it, and returns the sum as a response.
~~~
def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

    return response
~~~
### 2.2 Add an entry point
Add the following line between the 'console_scripts': brackets:
~~~
'service = py_srvcli.service_member_function:main',
~~~
## step 3 Write the client node
we should go to the ros2_ws/src/py_srvcli/py_srvcli directory, create a new file called client_member_function.py and paste the following code within:
~~~
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
~~~
### 3.1 Examine the code

The only different import statement for the client is import sys. The client node code uses sys.argv to get access to command line input arguments for the request.

The constructor definition creates a client with the same type and name as the service node. 
The type and name must match for the client and service to be able to communicate.

The while loop in the constructor checks if a service matching the type and name of the client is available once a second.

Below the constructor is the request definition, followed by main.

The only significant difference in the client’s main is the while loop. The loop checks the future to see if there is a response from the service,
as long as the system is running. 
If the service has sent a response, the result will be written in a log message.

### 3.2 Add an entry point
The entry_points field of your setup.py file should look like this:
~~~
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
~~~

## 4 Build and run
don't forget to return to the ros2_ws and check the missing parts 

~~~
rosdep install -i --from-path src --rosdistro humble -y
~~~
Navigate back to the root of your workspace, ros2_ws, and build your new package:
~~~
colcon build --packages-select py_srvcli
~~~
we need to oppen Open a new terminal, navigate to ros2_ws, and source the setup files:
~~~
. install/setup.bash
~~~
now we can run the code 
~~~
ros2 run py_srvcli service
~~~

<br />

![service node](https://user-images.githubusercontent.com/91989561/196821269-7f4e6139-bcbf-4334-92c0-97eacc99b012.png)

<br />


Open another terminal and source the setup files from inside ros2_ws again. Start the client node, followed by any two integers separated by a space:
~~~
ros2 run py_srvcli client 2 3
~~~
we choiced 2 and 3 so result wii become 
~~~
[INFO] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5
~~~
<br />

![service node respond](https://user-images.githubusercontent.com/91989561/196821687-efcb016c-c9fa-4ea5-b7a8-a360ebcedc86.png)

<br />
Return to the terminal where your service node is running. You will see that it published log messages when it received the request:
~~~
[INFO] [minimal_service]: Incoming request
a: 2 b: 3
~~~
<br />

![service node](https://user-images.githubusercontent.com/91989561/196821841-967dad6a-e11b-4ad6-9ac0-fe81d5babb40.png)

<br />
