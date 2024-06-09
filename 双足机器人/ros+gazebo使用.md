# ros control 自定义控制插件

![ros control](https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/ros_control)

**Hardware Resource Interface Layer:**

由hardware_Interface（里面包含各种Interface）中的Interface组成的集合，控制器和硬件之间的桥梁。

结构：**hardware_interface** 		大的命名空间
				**xxxInterface**  				其中有各种各样的具体的Interface如JointStateInterface
						**xxxHandle**  			每一个Interface由多种Handle组成

​	**Handle：**

​	可以认为是一种数据结构，用来存储状态，命令变量的地址。

​	图中hardware_interface::JointStateInterface就是由JointStateHandle组成，用于存储关节位置速度等状态
​	hardware_interface::EffortJointInterface由JointHandle和其他组成，用于存储关节命令。而JointHandle继承自JointStateHandle，故JointHandle中既有状态也有指令。

## 1. 通用硬件接口

**Handle创建：**

**HybridJoinHandle:**关节硬件接口，继承自**hardware_interface::JointStateHandle**，父类主要用于获取关节的当前状态（使用gazebo仿真时，通过DefaultRobotHWSim::init()即可根据urdf创建相应的关节句柄；实机上，需要手动创建JointStateHandle）。
**功能：**Handle中存放关节指令的**指针**。

**FootSensorHandle:**足端位姿传感器。仿照**hardware_interface::ImuSensorHandle**创建，用于存放足端的位姿。

**BaseSensorHandle:**base位姿传感器。仿照**hardware_interface::ImuSensorHandle**创建，用于存放base的位姿。

**Interface创建：**

在xxxHandle.h的文件末尾加上：

```c++
class HybridJointInterface : public hardware_interface::HardwareResourceManager<HybridJointHandle, hardware_interface::ClaimResources> {};

class FootSensorInterface : public hardware_interface::HardwareResourceManager<FootSensorHandle> {};
```

## 2. 仿真控制插件

### 2.1 仿真硬件接口

![img](https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/Gazebo_ros_transmission.png)

ROS和Gazebo仿真与控制实物类似。

在控制实物时，需要继承hardware_interface::RobotHW，并实现readHW()和writeHW()的函数。

而在Gazebo中，需要通过hardware_interface::DefaultRobotHWSim，实现相同的功能。若控制器，硬件接口均使用ros默认，如EffortJointController, EffortJointInterface，gazebo可使用默认的DefaultRobotHWSim作为urdf中gazebo的控制器插件。

**HitRobotHWSim创建：继承自hardware_interface::DefaultRobotHWSim，关键在于各种硬件接口xxxInterface注册Handle和initSim(),readSim(),writeSim()这三个函数的实现**

**initSim():**先完成DefaultRobotHWSim::initSim()，此步完成了一些基本的Interface和handle的创建。再通过**registerInterface()**注册硬件接口，注册后的接口可认为是**全局变量**；最后通过**xxxInterface.registerHandle()**注册接口中的句柄，例如hybridJointInterface内就有12个hybridJointHandle；footSensorInterface就有2个footSensorHandle。

**readSim():**通过gazebo提供的接口获取机器人状态，并将其存入Handle中。

**writeSim():**将关节指令写入JointHandle，通过DefaultRobotHWSim::writeSim()连接gazebo.

### 2.2 仿真硬件接口编译、加载

**CMakeLists.txt:**

```yaml
cmake_minimum_required(VERSION 3.10)
project(hit_robot_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        gazebo_dev
        gazebo_ros_control
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        hit_robot_hw_sim
        CATKIN_DEPENDS
        roscpp
        gazebo_ros_control
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

add_library(hit_robot_hw_sim
        src/hit_robot_gazebo/HitRobotHWSim.cpp
        )

target_link_libraries(hit_robot_hw_sim
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target hit_robot_hw_sim" )
    add_clang_tooling(
            TARGETS hit_robot_hw_sim
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS hit_robot_hw_sim
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )


install(FILES hit_robot_hw_sim_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

```

**hit_robot_hw_sim_plugins.xml**

```yaml
<library path="lib/libhit_robot_hw_sim">
    <class
            name="hit_robot_hw/HitRobotHWSim" type="hit_robot::HitRobotHWSim"
            base_class_type="gazebo_ros_control::RobotHWSim">
        <description>
            The Hit robot simulation interface which constructs joint handles from an SDF/URDF.
        </description>
    </class>
</library>

```

**package.xml**

```yaml
<?xml version="1.0"?>
<package format="2">
  <name>hit_robot_hw</name>
  <version>0.0.0</version>
  <description>The hit_robot_hw package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="liudachuan@todo.todo">liudachuan</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/hit_robot_hw</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>gazebo_dev</build_depend>
  <build_depend>gazebo_ros_control</build_depend>
  <build_depend>gazebo_ros</build_depend>

  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>gazebo_dev</build_export_depend>
  <build_export_depend>gazebo_ros_control</build_export_depend>
  <build_export_depend>gazebo_ros</build_export_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>gazebo_dev</exec_depend>
  <exec_depend>gazebo_ros_control</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->
    <gazebo_ros_control plugin="${prefix}/hit_robot_hw_sim_plugins.xml"/>
    <gazebo_ros gazebo_model_path="${prefix}"/>
  </export>
</package>

```

**urdf**

```yaml
  <!-- 控制插件载入: -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libhit_robot_hw_sim.so">
   	 <!-- 读取参数服务器参数名称的前缀: -->
      <robotNamespace>/</robotNamespace> 
      <robotParam>robot_description</robotParam>
      <robotSimType>hit_robot_hw/HitRobotHWSim</robotSimType>
      <!-- <legacyModeNS>true</legacyModeNS> -->
    </plugin>
  </gazebo>
```

**empty_world.launch**

```yaml
<launch>

    <!-- 仿真相关参数 -->
    <arg name="paused" default="true"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>

    <!-- 向参数服务器载入urdf/xacro -->
    <param name="robot_description" textfile="$(find hit_robot_description)/urdf/hit_robot_description.urdf" />

    <rosparam file = "$(find hit_robot_hw)/config/hw_sim.yaml" command="load" />

      <!-- 载入地图资源，启动gazebo -->
    <!-- <include file="$(find hit_robot_ele_gazebo)/launch/empty_world.launch"> -->
      <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="debug" value="$(arg debug)" />
        <arg name="gui" value="$(arg gui)" />
        <arg name="paused" value="$(arg paused)"/>
        <arg name="use_sim_time" value="$(arg use_sim_time)"/>
        <arg name="headless" value="$(arg headless)"/>
        <!-- <arg name="real_time_factor" value = "0.5" /> -->
      </include>

    <!-- push robot_description to factory and spawn robot in gazebo -->
    <!-- 将urdf加载进gazebo -->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
          args="-urdf -model hit_robot_ele -param robot_description 
                -z 0.89683174
                -J left_hip_yaw_joint 0
                -J left_hip_roll_joint 0.0277177
                -J left_hip_pitch_joint -0.806343
                -J left_knee_pitch_joint  1.4168
                -J left_ankle_pitch_joint -0.610459
                -J left_ankle_roll_joint -0.0277177
                -J right_hip_yaw_joint 0
                -J right_hip_roll_joint -0.0423705
                -J right_hip_pitch_joint -0.807508
                -J right_knee_pitch_joint 1.41875
                -J right_ankle_pitch_joint -0.611238
                -J right_ankle_roll_joint 0.0423705"/> 

</launch>

```



### 2.3 控制器

需要创建一个mpc线程，在控制器进行初始化时，创建。

## 3. 实物控制插件

