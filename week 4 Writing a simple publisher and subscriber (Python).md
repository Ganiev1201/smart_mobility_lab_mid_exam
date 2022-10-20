# Writing a simple publisher and subscriber (Python).md
>Open a new terminal and source your ROS 2 installation so that ros2 commands will work.
~~~
source /opt/ros/humble/setup.bash
~~~
## Step 1: Create a package
<br />

Be carefully that Recall that packages should be created in the src directory, not the root of the workspace. 

<br />
> In the terminal will return a message verifying the creation of your package py_pubsub and all its necessary files and folders.

## step 2;Write the publisher node
> need to Download the example talker code by entering the command bellow

~~~
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
~~~
> Now there will be a new file named publisher_member_function.py adjacent to __init__.py.

<br />

by using gedit we should to open the file for the edit 
~~~
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
~~~
### 2.1  Examine the code
The first lines of code after the comments import rclpy so its Node class can be used.
~~~
import rclpy
from rclpy.node import Node
~~~
The next statement imports the built-in string message type that the node uses to structure the data that it passes on the topic.
~~~
from std_msgs.msg import String
~~~
These lines represent the node’s dependencies. Recall that dependencies have to be added to package.xml, which you’ll do in the next section.
Next, the MinimalPublisher class is created, which inherits from (or is a subclass of) Node.
~~~
class MinimalPublisher(Node):
~~~
Next, a timer is created with a callback to execute every 0.5 seconds. self.i is a counter used in the callback.
~~~
def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
~~~
timer_callback creates a message with the counter value appended, and publishes it to the console with get_logger().info.
~~~
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
~~~
and last one is First the rclpy library is initialized, then the node is created, and then it “spins” the node so its callbacks are called.
~~~
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
~~~
### 2.2 Add dependencies
we should to navigate ros2_ws/src/py_pubsub directory, where the setup.py, setup.cfg, and package.xml files have been created earlier 
gedit package.xml change the file given bellow
~~~
<description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
~~~
After the lines above, add the following dependencies corresponding to your node’s import statements:
~~~
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
~~~
### Add an entry point
Open the setup.py file.and check the maintainer, maintainer_email, description and license fields to users  package.xml:
~~~
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
~~~
Add the following line within the console_scripts brackets of the entry_points field:
~~~
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
~~~
### 2.4 Check setup.cfg
setup.cfg file should be correctly populated automatically
~~~
[develop]
script_dir=$base/lib/py_pubsub
[install]
install_scripts=$base/lib/py_pubsub
~~~

<br />

## step 3 Write the subscriber node
we should return for the os2_ws/src/py_pubsub/py_pubsub to create the next node
~~~
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
~~~
after thet we should to have __init__.py file 

~~~
__init__.py  publisher_member_function.py  subscriber_member_function.py
~~~

### 3.1 Add an entry point
eopen setup.py and add the entry point for the subscriber node below the publisher’s entry point. The entry_points field should now look like this:
~~~
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
~~~
## step 4 Build and run
 we should to jhave the files to check before run the code lready have the rclpy and std_msgs packages installed as part of your ROS 2 system.
 It’s good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:
 <br />
 from the command which given bellov lets try to run the code 
 ~~~
 rosdep install -i --from-path src --rosdistro humble -y
 ~~~
 we need to for the ros2_ws colcone build 
 ~~~
 colcon build --packages-select py_pubsub
 ~~~
 lets open the new terminal and setup it for the ros2_ws and run the command bellow
 ~~~
 . install/setup.bash
 ~~~
 Now run the talker node:
 ~~~
 ros2 run py_pubsub talker
 ~~~
 
 <br />
 
 ![talker](https://user-images.githubusercontent.com/91989561/196817972-2ca3820a-4018-422b-b3f1-347fa00428cb.png)

 Open another terminal, source the setup files from inside ros2_ws again, for the running listener
 ~~~
 ros2 run py_pubsub listener
 ~~~
 
 <br />
 ![listener](https://user-images.githubusercontent.com/91989561/196818254-2175c966-c010-400c-80fb-cfe4a4ead746.png)

 
 
