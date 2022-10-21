# Monitoring for parameter changes (C++)
> Goal: Learn to use the ParameterEventHandler class to monitor and respond to parameter changes.

## step 1 Create a package
Recall that packages should be created in the src directory, not the root of the workspace. So, navigate into ros2_ws/src and then create a new package there
~~~
ros2 pkg create --build-type ament_cmake cpp_parameter_event_handler --dependencies rclcpp
~~~

### 1.1 Update package.xml
Because you used the --dependencies option during package creation, you don’t have to manually add dependencies to package.xml 
or CMakeLists.txt. As always, though,
make sure to add the description, maintainer email and name, and license information to package.xml.
~~~
<description>C++ parameter events client tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
~~~
## step 2 Write the C++ node
~~~
#include <memory>

#include "rclcpp/rclcpp.hpp"

class SampleNodeWithParameters : public rclcpp::Node
{
public:
  SampleNodeWithParameters()
  : Node("node_with_parameters")
  {
    this->declare_parameter("an_int_param", 0);

    // Create a parameter subscriber that can be used to monitor parameter changes
    // (for this node's parameters as well as other nodes' parameters)
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Set a callback for this node's integer parameter, "an_int_param"
    auto cb = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(
          this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
          p.get_name().c_str(),
          p.get_type_name().c_str(),
          p.as_int());
      };
    cb_handle_ = param_subscriber_->add_parameter_callback("an_int_param", cb);
  }

private:
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleNodeWithParameters>());
  rclcpp::shutdown();

  return 0;
}
~~~
### 2 exam code i will just skip that part 

## 3 Build and run
It’s good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:
~~~
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
~~~

Navigate back to the root of your workspace, ros2_ws, and build your new package:
~~~
colcon build --packages-select cpp_parameter_event_handler
~~~

Open a new terminal, navigate to ros2_ws, and source the setup files:
~~~
. install/setup.bash
~~~

Now run the node:
~~~
ros2 run cpp_parameter_event_handler parameter_event_handler
~~~

![monitoring for parameters](https://user-images.githubusercontent.com/91989561/197109576-cf53d4b7-1ba5-4d19-b1e2-400c18db6e5a.png)

The node is now active and has a single parameter and will print a message whenever this parameter is updated. To test this,
open up another terminal and source the ROS setup file as before 
(. install/setup.bash) and execute the following command:
~~~
ros2 param set node_with_parameters an_int_param 43
~~~
### 3.1 Monitor changes to another node’s parameters
First update the constructor to add the following code after the existing code:
~~~
// Now, add a callback to monitor any changes to the remote node's parameter. In this
// case, we supply the remote node name.
auto cb2 = [this](const rclcpp::Parameter & p) {
    RCLCPP_INFO(
      this->get_logger(), "cb2: Received an update to parameter \"%s\" of type: %s: \"%.02lf\"",
      p.get_name().c_str(),
      p.get_type_name().c_str(),
      p.as_double());
  };
auto remote_node_name = std::string("parameter_blackboard");
auto remote_param_name = std::string("a_double_param");
cb_handle2_ = param_subscriber_->add_parameter_callback(remote_param_name, cb2, remote_node_name);
~~~

Then add another member variable, cb_handle2 for the additional callback handle:
~~~
private:
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle2_;  // Add this
};
~~~

In a terminal, navigate back to the root of your workspace, ros2_ws, and build your updated package as before:
~~~
colcon build --packages-select cpp_parameter_event_handler
~~~

Then source the setup files:
~~~
. install/setup.bash
~~~

Now, to test monitoring of remote parameters, first run the newly-built parameter_event_handler code:
~~~
ros2 run cpp_parameter_event_handler parameter_event_handler
~~~

Next, from another teminal (with ROS initialized), run the parameter_blackboard demo application, as follows:
~~~
ros2 run demo_nodes_cpp parameter_blackboard
~~~

Finally, from a third terminal (with ROS initialized), let’s set a parameter on the parameter_blackboard node:
~~~
ros2 param set parameter_blackboard a_double_param 3.45
~~~
![monitoring para 2](https://user-images.githubusercontent.com/91989561/197110299-04cbab1c-1be7-4789-ad51-0c28cfa05b5e.png)

<br />
# Creating a launch file
Goal: Create a launch file to run a complex ROS 2 system.
## step 1 Setup
Create a new directory to store your launch files:
~~~
mkdir launch
~~~

## step 2 Write the launch file
Let’s put together a ROS 2 launch file using the turtlesim package and its executables. As mentioned above, this can either be in Python, XML, or YAML.
~~~
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
~~~
## step 3 ros2 launch
To run the launch file created above, enter into the directory you created earlier and run the following command:
~~~
cd launch
ros2 launch turtlesim_mimic_launch.py
~~~

Two turtlesim windows will open, and you will see the following [INFO] messages telling you which nodes your launch file has started:
~~~
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [turtlesim_node-1]: process started with pid [11714]
[INFO] [turtlesim_node-2]: process started with pid [11715]
[INFO] [mimic-3]: process started with pid [11716]
~~~
![creating lunch file ](https://user-images.githubusercontent.com/91989561/197110865-eaa788ab-4bfe-45f8-a48c-1420f961cbed.png)

To see the system in action, open a new terminal and run the ros2 topic pub command on the /turtlesim1/turtle1/cmd_vel topic to get the first turtle moving:
~~~
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
~~~
![topivk pub turtle cycle](https://user-images.githubusercontent.com/91989561/197111011-9a9fe823-6bbd-4ede-a926-b626614c620a.png)

## step 4 Introspect the system with rqt_graph
Run the command:
~~~
rqt_graph
~~~
![rqt_graph](https://user-images.githubusercontent.com/91989561/197111195-7135e442-0918-43a0-bdec-f485ff9412e9.png)

<br />

# Integrating launch files into ROS 2 packages
Goal: Add a launch file to a ROS 2 package

Create a workspace for the package to live in:
~~~
mkdir -p launch_ws/src
cd launch_ws/src
~~~

next 
~~~
ros2 pkg create py_launch_example --build-type ament_python
~~~

## step 2 Creating the structure to hold launch files
For Python packages, the directory containing your package should look like this:
~~~
src/
  py_launch_example/
    package.xml
    py_launch_example/
    resource/
    setup.py
    setup.cfg
    test/
~~~

Inside our setup.py file:
~~~
import os
from glob import glob
from setuptools import setup

package_name = 'py_launch_example'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
~~~
## step 3 Writing the launch file
Your launch file should define the generate_launch_description() function which returns a launch.LaunchDescription() to be used by the ros2 launch verb.
~~~
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'),
  ])
~~~

## 4 Building and running the launch file
Go to the top-level of the workspace, and build it:
~~~
colcon build
~~~

After the colcon build has been successful and you’ve sourced the workspace, you should be able to run the launch file as follows:
~~~
ros2 launch py_launch_example my_script_launch.py
~~~
![integrating launch](https://user-images.githubusercontent.com/91989561/197111875-504f0cc5-b0d5-49d4-9c93-7321066c4ff7.png)

# Using substitutions
Tutorial level: Intermediate
## step 1 Create and setup the package
Create a new package of build_type ament_python called launch_tutorial:
~~~
ros2 pkg create launch_tutorial --build-type ament_python
~~~

Inside of that package, create a directory called launch:
~~~
mkdir launch_tutorial/launch
~~~

Finally, make sure to add in changes to the setup.py of the package so that the launch files will be installed:
~~~
import os
from glob import glob
from setuptools import setup

package_name = 'launch_tutorial'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
~~~
## step 2 Parent launch file
Let’s create a launch file that will call and pass arguments to another launch file. To do this,
create an example_main.launch.py file in the launch folder of the launch_tutorial package.
~~~
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'example_substitutions.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
~~~
## step 3 Substitutions example launch file
Now create an example_substitutions.launch.py file in the same folder.
~~~
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle,
        change_background_r,
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])
~~~
## step 4 Build the package
Go to the root of the workspace, and build the package:
~~~
colcon build
~~~
after lunching need step 5
## step 5Launching example
Now you can launch the example_main.launch.py file using the ros2 launch command.
~~~
ros2 launch launch_tutorial example_main.launch.py
~~~

## step 6 Modifying launch arguments
If you want to change the provided launch arguments, you can either update them in launch_arguments dictionary in the example_main.launch.py
or launch the example_substitutions.launch.py with preferred arguments.
To see arguments that may be given to the launch file, run the following command:
~~~
ros2 launch launch_tutorial example_substitutions.launch.py --show-args
~~~

This will show the arguments that may be given to the launch file and their default values.
~~~
Arguments (pass arguments as '<name>:=<value>'):

    'turtlesim_ns':
        no description given
        (default: 'turtlesim1')

    'use_provided_red':
        no description given
        (default: 'False')

    'new_background_r':
        no description given
        (default: '200')
~~~
![lunch argument ](https://user-images.githubusercontent.com/91989561/197112686-1ca76b64-5fb0-4160-822e-00c1362687df.png)

Now you can pass the desired arguments to the launch file as follows:
~~~
ros2 launch launch_tutorial example_substitutions.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
~~~
![lunvh example](https://user-images.githubusercontent.com/91989561/197112974-d0364786-0e42-4772-803d-6dbebbe1ed4d.png)
