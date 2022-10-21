# Introducing tf2
Tutorial level: Intermediate

## step 1 Installing the demo
Let’s start by installing the demo package and its dependencies.
~~~
sudo apt-get install ros-humble-turtle-tf2-py ros-humble-tf2-tools ros-humble-tf-transformations
~~~
![arguments lunch](https://user-images.githubusercontent.com/91989561/197113764-316bbf6e-dd34-4b25-8e88-ed8fd049b22b.png)

## Running the demo
Now that we’ve installed the turtle_tf2_py tutorial package let’s run the demo. First, open a new terminal and source your ROS 2 installation so that ros2 commands will work. 
Then run the following command:
~~~
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py
~~~
![lunch turtle](https://user-images.githubusercontent.com/91989561/197113895-2abe6e09-563b-4b53-b7fa-441eec443c60.png)

In the second terminal window type the following command:
~~~
ros2 run turtlesim turtle_teleop_key
~~~
![tf2 tortle run](https://user-images.githubusercontent.com/91989561/197114020-0d354a4d-57db-4c1a-826a-58af58f87305.png)

## What is happening?
the second turtle following second one 

## tf2 tools
Now let’s look at how tf2 is being used to create this demo. We can use tf2_tools to look at what tf2 is doing behind the scenes.

### step 1  Using view_frames
view_frames creates a diagram of the frames being broadcasted by tf2 over ROS.
~~~
ros2 run tf2_tools view_frames
~~~
 we are able to see 
 ~~~
 Listening to tf data during 5 seconds...
Generating graph in frames.pdf file...
 ~~~
 ![using viwe frame](https://user-images.githubusercontent.com/91989561/197114290-e4b0ce59-9c2b-413f-b75c-108b3feb7d07.png)
 
 ### step 2 Using tf2_echo
 tf2_echo reports the transform between any two frames broadcasted over ROS.
 ~~~
 ros2 run tf2_ros tf2_echo [reference_frame] [target_frame]
 ~~~
 
 Let’s look at the transform of the turtle2 frame with respect to turtle1 frame which is equivalent to:
 ~~~
 ros2 run tf2_ros tf2_echo turtle2 turtle1
 ~~~
 
 You will see the transform displayed as the tf2_echo listener receives the frames broadcasted over ROS2.
 ~~~
 At time 1622031731.625364060
- Translation: [2.796, 1.039, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.202, 0.979]
At time 1622031732.614745114
- Translation: [1.608, 0.250, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.032, 0.999]
~~~
![turtle frame](https://user-images.githubusercontent.com/91989561/197114652-bc9ebd83-3701-403e-8fc1-622979ed64d7.png)
![transform display](https://user-images.githubusercontent.com/91989561/197114793-0b47e68a-d85b-40c5-a272-6979740986d7.png)

## rviz and tf2
rviz is a visualization tool that is useful for examining tf2 frames. Let’s look at our turtle frames using rviz.
Let’s start rviz with the turtle_rviz.rviz configuration file using the -d option:
~~~
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz
~~~
![rviz and tf2](https://user-images.githubusercontent.com/91989561/197115045-a5235e30-51e5-4b1c-a278-1935332fd2b5.png)


# Building a visual robot model from scratch
Tutorial level: Intermediate

## One Shape
First, we’re just going to explore one simple shape. Here’s about as simple as a urdf as you can make. [Source: 01-myfirst.urdf]
~~~
<?xml version="1.0"?>
<robot name="myfirst">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>
</robot>
~~~

To examine the model, launch the display.launch.py file:
~~~
ros2 launch urdf_tutorial display.launch.py model:=urdf/01-myfirst.urdf
~~~

A slightly modified argument allows this to work regardless of the current working directory:\
~~~
ros2 launch urdf_tutorial display.launch.py model:=`ros2 pkg prefix --share urdf_tutorial`/urdf/01-myfirst.urdf
~~~
![urd 1](https://user-images.githubusercontent.com/91989561/197115504-5636cf2c-9cb9-469b-a738-a5cd6f530b6b.png)

## Multiple Shapes
Now let’s look at how to add multiple shapes/links. If we just add more link elements to the urdf, the parser won’t know where to put them.
So, we have to add joints. Joint elements can refer to both flexible and inflexible joints. 
We’ll start with inflexible, or fixed joints. [Source: 02-multipleshapes.urdf]
~~~
<?xml version="1.0"?>
<robot name="multipleshapes">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
  </joint>

</robot>
~~~
![urd 2](https://user-images.githubusercontent.com/91989561/197115686-89c90efb-d777-4160-948e-8222a7bb35b7.png)

## Origins
R2D2’s leg attaches to the top half of his torso, on the side. So that’s where we specify the origin of the JOINT to be. Also, 
it doesn’t attach to the middle of the leg, it attaches to the upper part, so we must offset the origin for the leg as well. 
We also rotate the leg so it is upright. [Source: 03-origins.urdf]
~~~
<?xml version="1.0"?>
<robot name="origins">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0 -0.22 0.25"/>
  </joint>

</robot>
~~~

Let’s start by examining the joint’s origin. It is defined in terms of the parent’s reference frame. So we are -0.22 meters in the y direction 
(to our left, but to the right relative to the axes) and 0.25 meters in the z direction (up). This means that the origin for the child link will
be up and to the right, regardless of the child link’s visual origin tag. Since we didn’t specify a rpy (roll pitch yaw) attribute, the child frame 
will be default have the same orientation as the parent frame.
~~~
ros2 launch urdf_tutorial display.launch.py model:=urdf/03-origins.urdf
~~~
![urdf 3](https://user-images.githubusercontent.com/91989561/197116000-58103c0a-bdfd-47c7-9f72-fdcb78272587.png)


Now, looking at the leg’s visual origin, it has both a xyz and rpy offset. This defines where the center of the visual element should be, relative to its origin.
Since we want the leg to attach at the top, we offset the origin down by setting the z offset to be -0.3 meters.
And since we want the long part of the leg to be parallel to the z axis, we rotate the visual part PI/2 around the Y axis.

## Material Girl
“Alright,” I hear you say. “That’s very cute, but not everyone owns a B21. My robot and R2D2 are not red!” That’s a good point. 
Let’s take a look at the material tag. [Source: 04-materials.urdf] 
~~~
<?xml version="1.0"?>
<robot name="materials">

  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>

  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <material name="white"/>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0 -0.22 0.25"/>
  </joint>

  <link name="left_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <material name="white"/>
    </visual>
  </link>

  <joint name="base_to_left_leg" type="fixed">
    <parent link="base_link"/>
    <child link="left_leg"/>
    <origin xyz="0 0.22 0.25"/>
  </joint>

</robot>
~~~

the next step is 
~~~
ros2 launch urdf_tutorial display.launch.py model:=urdf/04-materials.urdf
~~~
![urdf 4](https://user-images.githubusercontent.com/91989561/197116456-fba74161-7ec1-4d7e-ba0e-c5ad806b5cb1.png)

## Finishing the Model

Now we finish the model off with a few more shapes: feet, wheels, and head. Most notably, we add a sphere and a some meshes.
We’ll also add few other pieces that we’ll use later. [Source: 05-visual.urdf]
~~~
<?xml version="1.0"?>
<robot name="visual">

  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <material name="white"/>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0 -0.22 0.25"/>
  </joint>

  <link name="right_base">
    <visual>
      <geometry>
        <box size="0.4 0.1 0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>

  <joint name="right_base_joint" type="fixed">
    <parent link="right_leg"/>
    <child link="right_base"/>
    <origin xyz="0 0 -0.6"/>
  </joint>

  <link name="right_front_wheel">
    <visual>
      <origin rpy="1.57075 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.035"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  <joint name="right_front_wheel_joint" type="fixed">
    <parent link="right_base"/>
    <child link="right_front_wheel"/>
    <origin rpy="0 0 0" xyz="0.133333333333 0 -0.085"/>
  </joint>

  <link name="right_back_wheel">
    <visual>
      <origin rpy="1.57075 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.035"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  <joint name="right_back_wheel_joint" type="fixed">
    <parent link="right_base"/>
    <child link="right_back_wheel"/>
    <origin rpy="0 0 0" xyz="-0.133333333333 0 -0.085"/>
  </joint>

  <link name="left_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <material name="white"/>
    </visual>
  </link>

  <joint name="base_to_left_leg" type="fixed">
    <parent link="base_link"/>
    <child link="left_leg"/>
    <origin xyz="0 0.22 0.25"/>
  </joint>

  <link name="left_base">
    <visual>
      <geometry>
        <box size="0.4 0.1 0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>

  <joint name="left_base_joint" type="fixed">
    <parent link="left_leg"/>
    <child link="left_base"/>
    <origin xyz="0 0 -0.6"/>
  </joint>

  <link name="left_front_wheel">
    <visual>
      <origin rpy="1.57075 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.035"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  <joint name="left_front_wheel_joint" type="fixed">
    <parent link="left_base"/>
    <child link="left_front_wheel"/>
    <origin rpy="0 0 0" xyz="0.133333333333 0 -0.085"/>
  </joint>

  <link name="left_back_wheel">
    <visual>
      <origin rpy="1.57075 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.035"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  <joint name="left_back_wheel_joint" type="fixed">
    <parent link="left_base"/>
    <child link="left_back_wheel"/>
    <origin rpy="0 0 0" xyz="-0.133333333333 0 -0.085"/>
  </joint>

  <joint name="gripper_extension" type="fixed">
    <parent link="base_link"/>
    <child link="gripper_pole"/>
    <origin rpy="0 0 0" xyz="0.19 0 0.2"/>
  </joint>

  <link name="gripper_pole">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.01"/>
      </geometry>
      <origin rpy="0 1.57075 0 " xyz="0.1 0 0"/>
    </visual>
  </link>

  <joint name="left_gripper_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0.2 0.01 0"/>
    <parent link="gripper_pole"/>
    <child link="left_gripper"/>
  </joint>

  <link name="left_gripper">
    <visual>
      <origin rpy="0.0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_tip_joint" type="fixed">
    <parent link="left_gripper"/>
    <child link="left_tip"/>
  </joint>

  <link name="left_tip">
    <visual>
      <origin rpy="0.0 0 0" xyz="0.09137 0.00495 0"/>
      <geometry>
        <mesh filename="package://urdf_tutorial/meshes/l_finger_tip.dae"/>
      </geometry>
    </visual>
  </link>
  <joint name="right_gripper_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0.2 -0.01 0"/>
    <parent link="gripper_pole"/>
    <child link="right_gripper"/>
  </joint>

  <link name="right_gripper">
    <visual>
      <origin rpy="-3.1415 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
      </geometry>
    </visual>
  </link>

  <joint name="right_tip_joint" type="fixed">
    <parent link="right_gripper"/>
    <child link="right_tip"/>
  </joint>

  <link name="right_tip">
    <visual>
      <origin rpy="-3.1415 0 0" xyz="0.09137 0.00495 0"/>
      <geometry>
        <mesh filename="package://urdf_tutorial/meshes/l_finger_tip.dae"/>
      </geometry>
    </visual>
  </link>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.2"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>
  <joint name="head_swivel" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <link name="box">
    <visual>
      <geometry>
        <box size="0.08 0.08 0.08"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <joint name="tobox" type="fixed">
    <parent link="head"/>
    <child link="box"/>
    <origin xyz="0.1814 0 0.1414"/>
  </joint>
</robot>
~~~
![urdf 5](https://user-images.githubusercontent.com/91989561/197116672-b19652d0-5733-4ef2-b60a-02e4e45ad81b.png)

How to add the sphere should be fairly self explanatory:
~~~
<link name="head">
  <visual>
    <geometry>
      <sphere radius="0.2"/>
    </geometry>
    <material name="white"/>
  </visual>
</link>
~~~

The meshes here were borrowed from the PR2. They are separate files which you have to specify the path for. 
You should use the package://NAME_OF_PACKAGE/path notation.
The meshes for this tutorial are located within the urdf_tutorial package, in a folder called meshes.
~~~
<link name="left_gripper">
  <visual>
    <origin rpy="0.0 0 0" xyz="0 0 0"/>
    <geometry>
      <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
    </geometry>
  </visual>
</link>
~~~


