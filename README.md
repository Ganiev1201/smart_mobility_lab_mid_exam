# SMEL_project Task-1 |
>as we alredy done with the installation some of parts can be skiped!
## Step 1: Turtle installation
<br />

As always, start by sourcing your setup files in a new terminal

<br />

~~~
source /opt/ros/humble/setup.bash
~~~

<br />

First we need to make sure our software is up to date

<br />

~~~
sudo apt update
sudo apt install ros-humble-turtlesim
~~~
![1]![uodate ](https://user-images.githubusercontent.com/91989561/196800280-ae74bba8-550d-4a37-a818-d861d9852382.png)

<br />
Lets install turtleism
<br />

~~~
sudo apt install ros-rolling-turtlesim
~~~
![2]![baqani in](https://user-images.githubusercontent.com/91989561/196802519-e1074818-84e0-49f3-a2c5-feeeee3516fc.png)

<br />

starting turtlesim
~~~
ros2 pkg executables turtlesim
~~~
![3]![toshbaqa 1](https://user-images.githubusercontent.com/91989561/196801759-35ab6d92-214b-40c6-b443-f0d51f69976e.png)

<br />

## Install and Run

<br />

After finishing that things we can run turtle

<br />

~~~
ros2 run turtlesim turtle_teleop_node
~~~

<br />

for the moving the tortle by using arrow keys we should give a code bellow
~~~
ros2 run turtlesim turtle_teleop_key
~~~
<br />

You can see the nodes and their associated services, topics, and actions  "using"  the list command:
~~~
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
~~~

<br />

### Step 3 installing rqt

<br />

For the running rqt we need code bellow 
~~~
rqt
~~~
<br />
When you run rqt for the first time window will be blank. but no need to worry you just go **Plugins > Services > Service Caller from the menu bar at the top.**
<br />

![5](https://user-images.githubusercontent.com/91989561/196808536-04eda257-c0a8-4957-a9cd-6b5169f88efe.jpg)
<br />

Use the refresh button to the left of the Service dropdown list to ensure all the services of your turtlesim node are available.

<br />

## step 4 spawn service

<br />

We can create another turtle using the RQT tool. In order to do that, we should go to the tab Plugins > Services > Service cellar. Then, choose /spawn from the dropdown options and give a name and coordinates for our second turtle.

<br />

![photo_2022-10-03_20-31-34](https://user-images.githubusercontent.com/91989561/196809707-0599bc7e-f102-4332-8ddc-8ae595719a5f.jpg)
<br />

##traying set_pen service

<br />

Lets give to the tortle 1 set_pen service 

<br />

![set_pen](https://user-images.githubusercontent.com/91989561/196810590-25a4714a-f5f7-4a77-96bd-f5da5d851d4d.jpg)

<br />

If you return to the terminal where turtle_teleop_key is running and press the arrow keys, you will see turtle1’s pen has changed.

<br />

![7](https://user-images.githubusercontent.com/91989561/196804703-b895415a-0385-4a27-bc01-1c6b2fb54eea.jpg)

<br />

## Remapping
<br />

In a new terminal, source ROS 2, and run:
~~~
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
~~~

<br />
