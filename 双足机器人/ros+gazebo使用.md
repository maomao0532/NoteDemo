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

Handle创建：



Interface创建：



## 2. 仿真控制插件

### 2.1 仿真硬件接口

![img](https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/Gazebo_ros_transmission.png)

ROS和Gazebo仿真与控制实物类似。

在控制实物时，需要继承hardware_interface::RobotHW，并实现readHW()和writeHW()的函数。

而在Gazebo中，需要通过hardware_interface::RobotHWSim，实现相同的功能。若控制器，硬件接口均使用ros默认，如EffortJointController, EffortJointInterface，gazebo可使用默认的DefaultRobotHWSim作为urdf中gazebo的控制器插件。

xxxHWSim创建：

### 2.2 控制器

需要创建一个mpc线程，在控制器进行初始化时，创建。

## 3. 实物控制插件

