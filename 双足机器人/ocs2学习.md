# OCS2学习

## 1. 安装

### 1.1 完成前置依赖项下载

![image-20231224123306454](https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20231224123306454.png)

==特别注意，Eigen库使用v3.3.7版本否则会报错==

### 1.2 传家宝build

```
1.新建工作空间
mkdir -p ocs2_ws
2.将传家宝复制进ocs2_ws
3.catkin build
4.针对出现的错误，安装相应的ros包
sudo apt install ros-noetic-xxx
```

## 2. 官网start

### 2.1 确定模型

常用三种模型：

-   运动学模型：适用于底座固定的机械臂，规划末端轨迹，运动学求解关节角度。
-   动力学模型：考虑全部的运动学和动力学，准确但复杂。
-   质心模型：考虑运动学和外力的质心动力学，常用于腿式机器人。

人形机器人，选择**质心模型**。

### 2.2 最优问题通式

$$
\begin{cases}
\min\limits_{u(\cdot)}\sum_i\phi_i(\mathbf{x}(t_{i+1}))+\int_{t_i}^{t_{i+1}}l_i(\mathbf{x}(t),\mathbf{u}(t),t)dt\\\mathrm{s.t.~}\mathbf{x}(t_0)=\mathbf{x}_0&\text{initial state}\\\mathbf{\dot{x}}(t)=\mathbf{f}_i(\mathbf{x}(t),\mathbf{u}(t),t)&\text{system flow map}\\\mathbf{x}(t_{i+1})=\mathbf{j}(\mathbf{x}(t_{i+1}))&\text{system jump map}\\\mathbf{g}_{1i}(\mathbf{x}(t),\mathbf{u}(t),t)=\mathbf{0}&\text{state input equality constraints}\\\mathbf{g}_{2i}(\mathbf{x}(t),t)=\mathbf{0}&\text{state only equality constraints}\\\mathbf{h}_i(\mathbf{x}(t),\mathbf{u}(t),t)\geq0&\text{inequality constraints}\\\mathrm{for~}t_i<t<t_{i+1}\mathrm{~and~}i\in\{0,1,\cdots,I-1\}&\end{cases}
$$

**价值函数**可以看成两部分(类比双足MPC)：

-   状态部分(实际状态和期望状态的差)
-   控制部分(希望在控制量最小的情况下，达到更好的控制效果)

**限制条件：**

-   初始状态
-   系统流程图 即状态空间公式
-   状态跳转图（==不理解==）
-   状态和输入相关的等式约束
-   自身状态约束
-   状态和输入的不等式约束：如摩擦力约束这些

### 2.3 创建MPC问题并求解

通过建立系统动力学模型，构建**2.2**中的优化问题，设置MPC即要在每个控制时刻重复解决此问题。但由于MPC的求解速度相对较慢，ocs2提供了一些接口的同步机制。

**MPC接口**：**使用最新的观测值去更新求解器**。状态会被放入缓冲区，以保证使用时，状态始终为最新状态。

**MRT接口**：**安全访问求解器的结果**。两种访问模式：

-   **基于时间**：对求解出来的状态和输入进行插值，根据所需时间，得到结果。
-   **基于状态**：根据反馈的时间、状态，去得到最佳收入。

## 3. 小车倒立摆MPC案例

### 3.1 文件组成

<img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20231224220910497.png" alt="image-20231224220910497" style="zoom:50%;" />

主要由两个功能包组成：

-   ocs2_cartpole：用于存储MPC问题、创建动力学、创建接口等。
    -   auto_generated:用于存放cppAD的文件，自动生成
    -   config：用于存放MPC的taskfile，其中有问题相关物理参数、求解器设置、MPC设置、初末状态、各项权重等。
    -   include：存有动力学头文件、接口头文件、参数头文件等。
    -   src：接口函数的具体实现。
-   ocs2_catrpole_ros：
    -   待补充

### 3.2 倒立摆动力学模型

<img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20231224222631176.png" alt="image-20231224222631176" style="zoom:15%;" />

对杆分析，设车对杆的力为向右的$\vec{H}$，和向上的$\vec{V}$，假设杆是均匀质量，不是图示集中在上方：
$$
\frac{1}{12}ml^2\ddot{\theta}=V\cdot\frac{1}{2}l\text{sin}\theta+
H\cdot\frac{1}{2}l\text{cos}\theta
\\
m\ddot{x_l}=H
\\
m\ddot{y_l}=V-mg
\\
M\ddot{x}=F-H
$$
又有$x_l=x-\frac{1}{2}l\text{sin}\theta$,$y_l=y+\frac{1}{2}l\text{cos}\theta$则有：
$$
\begin{align}
\ddot{x_l}&=\ddot{x}-\frac{1}{2}l(\text{cos}\theta\cdot\dot{\theta})'
\\
&=
\ddot{x}-\frac{1}{2}l(-\text{sin}\theta\cdot\dot{\theta}^2+\text{cos}\theta\cdot\ddot{\theta})
\end{align}
$$

$$
\begin{align}
\ddot{y_l}&=\ddot{y}+\frac{1}{2}l(-\text{sin}\theta\cdot\dot{\theta})'
\\
&=
\ddot{y}+\frac{1}{2}l(-\text{cos}\theta\cdot\dot{\theta}^2-\text{sin}\theta\cdot\ddot{\theta})
\end{align}
$$



带入动力学公式：
$$
m\ddot{x}+\frac{1}{2}ml\text{sin}\theta\cdot\dot{\theta}^2-\frac{1}{2}ml\text{cos}\theta\cdot\ddot{\theta}=H
\\
\Rightarrow
(M+m)\ddot{x}+\frac{1}{2}ml\text{sin}\theta\cdot\dot{\theta}^2=F+\frac{1}{2}ml\text{cos}\theta\cdot\ddot{\theta}
\\
m\ddot{y}-\frac{1}{2}ml\text{cos}\theta\cdot\dot{\theta}^2-\frac{1}{2}ml\text{sin}\theta\cdot\ddot{\theta}=V-mg,\ddot{y}=0
$$
将$V,H$带入角动量微分方程：
$$
\frac{1}{3}ml^2\ddot{\theta}=\frac{1}{2}m\ddot{xl}\text{cos}\theta+\frac{1}{2}mgl\text{sin}\theta
$$
总上，构建状态空间方程：
$$
\begin{bmatrix}
\frac{1}{3}ml^2 & -\frac{1}{2}ml\text{cos}\theta \\
-\frac{1}{2}ml\text{cos}\theta & M+m
\end{bmatrix}
\begin{bmatrix}
\ddot{\theta} \\
\ddot{x}
\end{bmatrix}
=
\begin{bmatrix}
\frac{1}{2}mgl\text{sin}\theta \\
F-\frac{1}{2}ml\text{sin}\theta\cdot\dot{\theta}^2
\end{bmatrix}
$$
在ocs2原代码中的$\theta$和上式$\theta$稍有不同，动力学改成上式，也可以正常求解。

对应代码动力学代码中：

```c++
ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const 								  					  ad_vector_t& input, const ad_vector_t& parameters) const override {
    const ad_scalar_t cosTheta = cos(state(0));
    const ad_scalar_t sinTheta = sin(state(0));

    // Inertia tensor
    Eigen::Matrix<ad_scalar_t, 2, 2> I;
    I << static_cast<ad_scalar_t>(param_.poleSteinerMoi_), 
    	 static_cast<ad_scalar_t>(-param_.poleMass_ * param_.poleHalfLength_ * cosTheta),
         static_cast<ad_scalar_t>(-param_.poleMass_ * param_.poleHalfLength_ * cosTheta),
         static_cast<ad_scalar_t>(param_.cartMass_ + param_.poleMass_);

    // RHS
    Eigen::Matrix<ad_scalar_t, 2, 1> 
        rhs(param_.poleMass_ * param_.poleHalfLength_ * param_.gravity_ * sinTheta,
            input(0) - param_.poleMass_ * param_.poleHalfLength_ * pow(state(2), 2) * sinTheta);

    // dxdt
    ad_vector_t stateDerivative(STATE_DIM);
    stateDerivative << state.tail<2>(), I.inverse() * rhs;
    return stateDerivative;
  }
```

### 3.3 ocs2_cartpole文件解析

<img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20231225210832101.png" alt="image-20231225210832101" style="zoom:50%;" />

自动生成，只需在ocs2_cartpole_ros中的MPC节点中，给定路径即可。

<img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20231225211208236.png" alt="image-20231225211208236" style="zoom:40%;" />

-   **task.info**为taskfile为求解器提供相关参数。
-   multiplot中为rqt_multiplot的相关参数设置，此包主要用于一些状态量的绘图。

<img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20231225211417868.png" alt="image-20231225211417868" style="zoom:40%;" />

-   **dynamics**：用于创建CartPoleSytemDynamics类，内有：构造函数、析构函数、克隆函数、动力学模型函数（输入当前状态，求解状态微分）
-   **CartPoleInterface.h**：用于创建CartPoleInterface类。
    -   创建时，传入taskFile，libraryFolder，verbose，通过构造函数，可以加载taskFile中信息initialState_， xFinal_赋值；ddpSettings_，mpcSettings_赋值；创建最优问题，给problem_中一些参数赋值：设置代价函数中的权重（costPtr，finalCostPtr）、设置动力学指针（dynamicsPtr）、不等式约束指针（inequalityLagrangianPtr）、初始化cartPoleInitializerPtr、设置输出指针（rolloutPtr_）
    -   创建相关函数，用于获取接口中各变量的信息
-   **CartPoleParameters.h**：用于创建CartPoleParameters类。计算并存储相关的物理参数
-   definitions.h：输入输出维度变量创建。

<img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20231225213510813.png" alt="image-20231225213510813" style="zoom:50%;" />

实现CartPoleSytemDynamics类的构造函数：**检查taskfile是否存在、创建libraryFolder、加载初始状态、加载ddp和mpc设置、设置优化问题权重、创建中间代价和末代价、加载物理参数并创建动力学指针、加载前向输出参数并创建指针、创建约束、初始化。**

### 3.4 ocs2_cartpole_ros文件解析

<img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20231225222608818.png" alt="image-20231225222608818" style="zoom:50%;" />

创建一个**虚拟仿真观测器**，继承于DummyObserver类，主要在src文件中复写update函数，实现类似强化学习中step的功能，通过动力学模型进行下一时刻状态计算。

<img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20231225222836229.png" alt="image-20231225222836229" style="zoom:50%;" />

-   **CartpoleDummyVisualization**：主要用于复写update函数
-   **CartpoleMpcNode**：初始化ros节点、创建接口实例（ocs2_cartpole中的CartPoleInterface类）、创建MPC实例（通过接口实例向其传入mpc设置等信息）、向mpc添加观测器模块、创建MPC接口实例、启动MPC节点。
-   **DummyCartpoleNode**：初始化ros节点、创建接口实例（ocs2_cartpole中的CartPoleInterface类）、创建MRT接口实例、创建虚拟仿真器、启动虚拟仿真器。

<img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20231226153559721.png" alt="image-20231226153559721" style="zoom:50%;" />

-   visualize：用于启动rviz，可视化作用。
-   multiplot：启动rqt_multiplot插件，绘制图像
-   **cartpole：启动mpc节点和mrt节点。**

### 3.5 流程分析

![..](https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20231226155513941.png)

主要启动了五个节点：

1.  **/cartpole_mpc**：MPC节点，接收/cartpole_dummy_test发布的观测信息，进行求解MPC问题，并将求解结果发布出去。
2.  **/cartpole_dummy_test**：MRT节点，创建MRT跟踪模型状态，并发布观测信息和关节状态（供rviz可视化）；创建虚拟仿真器，得到模型状态（使用gazebo仿真时，模型状态通过订阅gazebo得到？）
3.  /multiplot_remap：绘图节点
4.  /robot_state_publisher：订阅/joint_states信息，将其转换成/tf，发布给/Cartpole（rviz可视化)

<<<<<<< HEAD
### 4. 四足机器人案例
=======
## 4. 四足机器人案例
>>>>>>> 4498bfbecbc36eec61b5d9d096dda66f84008ca5

### 4.1 动力学建模

参考文献：

```
A Unified MPC Framework for Whole-Body Dynamic Locomotion and Manipulation 2019
Centroidal dynamics of a humanoid robot 2013
```

<img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20231228102837138.png" alt="image-20231228102837138" style="zoom:50%;" />

图中存在三种坐标系，世界坐标系$\sum\{\mathcal{I}\}$，baselink（躯干）的坐标系$\sum\{\mathcal{B}\}$，质心坐标系$\sum\{\mathcal{G}\}$。

$f_{ci}$为地面在足端的反作用合力。

对质心处使用牛顿欧拉公式：
$$
\dot{\boldsymbol{h}}_{com}=\begin{bmatrix}\sum_{i=1}^{n_c}\boldsymbol{f}_{c_i}+m\boldsymbol{g}\\\sum_{i=1}^{n_c}\boldsymbol{r}_{com,c_i}\times\boldsymbol{f}_{c_i}+\boldsymbol{\tau}_{c_i}\end{bmatrix}
$$
其中${\boldsymbol{h}}_{com}=\{\boldsymbol{p}_{com},\boldsymbol{l}_{com}\}$，$\boldsymbol{p}_{com},\boldsymbol{l}_{com}$分别为质心在**质心坐标系**下的线动量和角动量。

参考文献中，讨论了baselink的位姿变化率对质心动量的影响：
$$
\boldsymbol{h}_{com}=\underbrace{[\boldsymbol{A}_b(\boldsymbol{q})\quad\boldsymbol{A}_j(\boldsymbol{q})]}_{\boldsymbol{A}(\boldsymbol{q})}\begin{bmatrix}\dot{\boldsymbol{q}}_b\\\dot{\boldsymbol{q}}_j\end{bmatrix}
$$
其中$A(q)$在参考文献中有详细说明，总之，是和$q_b,q_j$相关。

<<<<<<< HEAD
$\boldsymbol{q}_{b},\boldsymbol{q}_{j}$分别为baselink在**世界坐标系下**的位姿$\begin{aligned}\boldsymbol{q}_b=(\boldsymbol{r}_{IB},\boldsymbol{\Phi}_{IB}^{zyx})\end{aligned}$，各关节角度







```
├─auto_generated
├─config
│  ├─command
│  │      gait.info------------------------------------步态设置
│  │      reference.info-------------------------------初始状态设置
│  │
│  └─mpc
│          task.info-----------------------------------MPC参数设置
│
├─include
│  └─ocs2_biped_robot
│      │  BipedRobotInterface.h&cpp--------------------接口文件
│      │  BipedRobotPreComputation.h&cpp---------------预计算
│      │  package_path.h.in
│      │
│      ├─common
│      │      ModelSettings.h--------------------------模型信息
│      │      Types.h----------------------------------变量格式
│      │      utils.h
│      │
│      ├─constraint
│      │      EndEffectorLinearConstraint.h&cpp--------末端约束
│      │      FrictionConeConstraint.h&cpp-------------摩擦锥约束
│      │      JointTorqueConstraint.h&cpp--------------关节力矩约束
│      │      NormalVelocityConstraintCppAd.h&cpp------法向速度约束
│      │      ZeroForceConstraint.h&cpp----------------游脚约束
│      │      ZeroVelocityConstraintCppAd.h&cpp--------接触脚约束
│      │
│      ├─cost
│      │      BipedRobotQuadraticTrackingCost.h&cpp----代价函数
│      │
│      ├─dynamics
│      │      BipedRobotDynamicsAD.h&cpp---------------动力学模型
│      │
│      ├─foot_planner
│      │      CubicSpline.h&cpp------------------------三次样条插值
│      │      SplineCpg.h&cpp
│      │      SwingTrajectoryPlanner.h&cpp-------------游脚轨迹规划
│      │
│      ├─gait
│      │      Gait.h&cpp
│      │      GaitSchedule.h&cpp-----------------------步态规划
│      │      LegLogic.h&cpp
│      │      ModeSequenceTemplate.h&cpp---------------状态模板
│      │      MotionPhaseDefinition.h&cpp
│      │
│      ├─initialization
│      │      BipedRobotInitializer.h&cpp--------------初始化
│      │
│      └─reference_manager
│              SwitchedModelReferenceManager.h&cpp-----模式切换
│
├─src
│  │  BipedRobotInterface.cpp
│  │  BipedRobotPreComputation.cpp
│  │
│  ├─common
│  │      ModelSettings.cpp
│  │
│  ├─constraint
│  │      EndEffectorLinearConstraint.cpp
│  │      FrictionConeConstraint.cpp
│  │      JointTorqueConstraint.cpp
│  │      NormalVelocityConstraintCppAd.cpp
│  │      ZeroForceConstraint.cpp
│  │      ZeroVelocityConstraintCppAd.cpp
│  │
│  ├─dynamics
│  │      BipedRobotDynamicsAD.cpp
│  │
│  ├─foot_planner
│  │      CubicSpline.cpp
│  │      SplineCpg.cpp
│  │      SwingTrajectoryPlanner.cpp
│  │
│  ├─gait
│  │      Gait.cpp
│  │      GaitSchedule.cpp
│  │      LegLogic.cpp
│  │      ModeSequenceTemplate.cpp
│  │
│  ├─initialization
│  │      BipedRobotInitializer.cpp
│  │
│  └─reference_manager
│          SwitchedModelReferenceManager.cpp
```

```
├─include
│  └─ocs2_biped_robot_ros
│      ├─gait
│      │      GaitKeyboardPublisher.h----------键盘控制步态
│      │      GaitReceiver.h-------------------步态发送
│      │      ModeSequenceTemplateRos.h
│      │
│      └─visualization
│              BipedRobotVisualizer.h----------可视化曲线
│
├─launch
│      biped_robot_ddp.launch
│      biped_robot_sqp.launch------------------SQP求解NMPC
│
├─rviz
│      biped_robot_rviz.rviz
│
└─src
    │  BipedRobotDdpMpcNode.cpp
    │  BipedRobotDummyNode.cpp-----------------MRT节点
    │  BipedRobotGaitCommandNode.cpp-----------步态切换节点
    │  BipedRobotPoseCommandNode.cpp-------------目标点发送节点
    │  BipedRobotSqpMpcNode.cpp---------------------MPC节点
    │
    ├─gait
    │      GaitKeyboardPublisher.cpp
    │      GaitReceiver.cpp
    │
    └─visualization
            BipedRobotVisualizer.cpp
```
=======
$\boldsymbol{q}_{b},\boldsymbol{q}_{j}$分别为baselink在**世界坐标系下**的位姿$\begin{aligned}\boldsymbol{q}_b=(\boldsymbol{r}_{IB},\boldsymbol{\Phi}_{IB}^{zyx})\end{aligned}$，各关节角度。

**约束**



### 4.2 ocs2_legged_robot分析

#### 4.2.1 config

-   **command**

    -   **gait.info**

    存放**步态列表**，和各步态的**ModeSequenceTemplate**（模式序列模板）

    <img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20240105141001233.png" alt="image-20240105141001233" style="zoom:50%;" />

    <img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20240105141120316.png" alt="image-20240105141120316" style="zoom:50%;" />

    trot：步态名称；modeSequence：模式序列；switchingTimes：模式切换时间；给了一个周期的相关信息。

    -   **reference.info**

    存放默认（初始）关节角度**defaultJointState**；初始模式序列**initialModeSchedule**（实际使用中会将其根据模板扩展成全时间序列）；默认模式序列模板**defaultModeSequenceTemplate**；目标速度和目标角速度**targetDisplacementVelocity**、**targetRotationVelocity**。

-   **mpc**

    -   **task.info**

    存放质心模型类型选择**centroidalModelType**、机器人接口参数（是否打印相关参数信息；是否使用分析梯度动力学、约束）**legged_robot_interface**、模型设置参数（位置误差增益、步态切换时中间STANCE持续时间。。）**model_settings**、游脚轨迹设置参数（抬腿速度、落脚速度、游脚高度、游脚持续时间）**swing_trajectory_config**、SQP求解参数设置**sqp**、IPM求解参数设置**ipm**、DDP求解设置**ddp**、动力学求解器相关设置**rollout**、mpc问题相关设置**mpc**、初始状态（质心六维动量、质心位姿、各关节角度）**initialState**、状态量、控制量权重矩阵**Q、R**、摩擦锥（松弛）参数设置（摩擦系数、松弛系数）**frictionConeSoftConstraint**。

-   **multiplot**：绘图相关设置

#### 4.2.2 include/src

##### 4.2.2.1 common

-   **ModelSettings.h/cpp**

创建**ModelSettings**结构体：位置误差增益**positionErrorGain**、步态切换站立时间**phaseTransitionStanceTime**、关节名称向量**jointNames**、六维度接触点（包含力和力矩）link向量**contactNames6DoF**、三维度接触点（包含力）link向量**contactNames3DoF**。
定义ocs2::legged_robot下的一个函数loadModelSettings，用于**载入task.info中的model_settings的信息**。

-   **Types.h/cpp**

创建一些变量类型如：feet_array_t**四维向量**、contact_flag_t**接触标志**（四维bool类型向量）、vector3_t三维向量、matrix3_t**三维矩阵**、quaternion_t**四元数**。

-   **utils.h/cpp**

在ocs2::legged_robot下的两个函数：**numberOfClosedContacts**，传入**接触标志的引用**，返回**接触的足端数量int类型**。**weightCompensatingInput**，传入**质心模型信息引用**和**接触标志引用**，返回在静止时，由支撑脚**均分**机器人重力，**控制量**的值。

##### 4.2.2.2 constraint

-   **EndEffectorLinearConstraint.h/cpp**

创建类**EndEffectorLinearConstraint**，其中有结构体**Config：包含向量b，矩阵Ax和Ay（向量、矩阵的行数和约束个数有关）**。

**属性**：末端运动学指针**endEffectorKinematicsPtr_**；约束个数**numConstraints\_**;配置参数**config\_**

**方法**：**构造函数**（传入末端运动学指针、约束个数和配置参数，直接通过参数列表**传参**）；
			**void configure(Config&& config)**传入config，设置新的约束系数；
			**void configure(const Config& config)**设置新的约束函数（**一般用这个**）；
			**EndEffectorKinematics<scalar_t>& getEndEffectorKinematics()**返回末端运动学指针；
			**size_t getNumConstraints(scalar_t time)**获取time时刻约束数量；
			**vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp)**计算约束的值：$A_x*x_{ee}+A_v*v_{ee}+b$
			**VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,const PreComputation& preComp)**计算约束的值$f$，对状态的微分$\frac{df}{dx}$矩阵（$n_{constraint},n_{state}$）和对输入（控制量）的微分$\frac{df}{du}$矩阵（$n_{constraint},n_{input}$）。将这些值存在**VectorFunctionLinearApproximation**的实例中返回。

-   **FrictionConeConstraint.h/cpp**

创建类**FrictionConeConstraint**，其中有结构体**Config**：**包含摩擦系数frictionCoefficient，正则化系数frictionCoefficient，狗上机械臂的抓紧力gripperForce，Hessian矩阵对角偏移量。**

**属性**：模式和目标轨迹切换管理器**referenceManagerPtr_**，约束参数**config\_**，接触足端索引**contactPointIndex\_**，质心模型信息**info\_**，世界到支撑面的旋转矩阵$\sideset{^t}{_w}R$ **t_R_w**。

**方法**：**构造函数**设置约束类型为二次、简单传参；
			**bool isActive(scalar_t time)** 判断time时刻，此足端是否与地面接触，激活摩擦锥约束；
			**size_t getNumConstraints(scalar_t time)** 获取约束个数，针对单腿，返回1；
			**vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp)** 根据当前状态和控制量计算约束的值，$h=frictionCoefficient * (F_z + gripperForce) - \sqrt{F_x * F_x + F_y * F_y + regularization}$；
			**VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,const PreComputation& preComp)** 计算约束的值，约束对状态的微分和对输入的微分；
			**VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp)** 计算约束的值，约束对状态的微分和对输入的微分，约束对状态、输入、状态输入的二阶导；
			**void setSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld)** 设置旋转矩阵；
			**vector_t coneConstraint(const vector3_t& localForces)** 根据接触力计算摩擦锥；
			**LocalForceDerivatives computeLocalForceDerivatives(const vector3_t& forcesInBodyFrame)** 返回LocalForceDerivatives结构体：dF_du F_local（局部坐标系下的足地接触力）相对输入（F_world，世界坐标系下的足地接触力)的导数即旋转矩阵；
			**ConeLocalDerivatives computeConeLocalDerivatives(const vector3_t& localForces)** 返回ConeLocalDerivatives结构体：dCone_dF h相对于F_local的一阶导、d2Cone_dF2 h相对于F_local的二阶导；
			**ConeDerivatives computeConeConstraintDerivatives(const ConeLocalDerivatives& coneLocalDerivatives, const LocalForceDerivatives& localForceDerivatives)** 返回ConeDerivatives结构体：dCone_du h相对于F_world的一阶导、 d2Cone_du2 h相对于F_world的二阶导；
			**matrix_t frictionConeInputDerivative(size_t inputDim, const ConeDerivatives& coneDerivatives)**计算h相对输入的一阶导；
$$
\begin{bmatrix}
\frac{dh}{du_1} & \frac{dh}{du_2} & \cdots & \frac{dh}{du_{n_u}}
\end{bmatrix}
$$
​			**matrix_t frictionConeSecondDerivativeInput(size_t inputDim, const ConeDerivatives& coneDerivatives)**计算h相对输入的二阶导；
$$
\begin{bmatrix}
\frac{d^2h}{du_1^2} & \frac{d^2h}{du_2du_1} & \cdots & \frac{d^2h}{du_{n_u}du_1} \\
\frac{d^2h}{du_1du_2} & \frac{d^2h}{du_2^2} & \cdots & \frac{d^2h}{du_{n_u}du_2} \\
\vdots & \vdots & \ddots & \vdots \\
\frac{d^2h}{du_1du_n} & \frac{d^2h}{du_2du_n} & \cdots & \frac{d^2h}{du_{n_u}^2}
\end{bmatrix}
$$
​			**matrix_t frictionConeSecondDerivativeState(size_t stateDim, const ConeDerivatives& coneDerivatives)**计算Cone相对状态的二阶导。理论上均为0，加上偏置为了便于求解。
$$
\begin{bmatrix}
hessianDiagonalShift & 0 & \cdots & 0 \\
0 & hessianDiagonalShift & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \cdots & hessianDiagonalShift
\end{bmatrix}
$$

-   **NormalVelocityConstraintCppAd.h&cpp**

**属性**：模式和目标轨迹切换管理器**referenceManagerPtr_**，末端执行器线性约束指针**eeLinearConstraintPtr_**（用于求解约束的值和微分等），接触点索引**contactPointIndex_**。

**方法**：**NormalVelocityConstraintCppAd**：构造函数、简单传参；
			**isActive**判断足端是否接触；
			**getValue**借助eeLinearConstraintPtr_计算约束的值；
			**getLinearApproximation**计算约束对状态和输入的微分。

-   **ZeroForceConstraint.h&cpp**

**属性**：模式和目标轨迹切换管理器**referenceManagerPtr_**，接触点索引**contactPointIndex_**，质心模型信息**info\_**。

**方法**：

##### 4.2.2.3 cost

##### 4.2.2.4 dynamics

##### 4.2.2.5 foot_planner

-   foot_planner.h&cpp：

创建CubicSpline类用于三次样条规划，类中创建结构体Node用于存放端点信息。构造函数中，求解三次多项式的系数，**注意**：先将端点的时间信息进行归一化后进行求解。

```c++
CubicSpline::CubicSpline(Node start, Node end) {
  assert(start.time < end.time);
  t0_ = start.time;
  t1_ = end.time;
  dt_ = end.time - start.time;

  scalar_t dp = end.position - start.position;
  scalar_t dv = end.velocity - start.velocity;

  // 表示三次多项式系数和速度相关的项
  dc0_ = 0.0;
  dc1_ = start.velocity;
  dc2_ = -(3.0 * start.velocity + dv);
  dc3_ = (2.0 * start.velocity + dv);
  // 三次多项式系数
  c0_ = dc0_ * dt_ + start.position;
  c1_ = dc1_ * dt_;
  c2_ = dc2_ * dt_ + 3.0 * dp;
  c3_ = dc3_ * dt_ - 2.0 * dp;
}
```

$$
x(t)=c3\_t_n^3+c2\_t_n^2+c1\_t_n+c0\_,t_n=\frac{t-t_0}{dt}\\
x'(t)=3c3\_t_n^2\cdot dt+2c2\_t_n\cdot dt+c1\_t_n\cdot dt
$$

将初始条件带入刚好等于代码中求解的值

## 5. ocs2+WBC 双足

### 5.1 节点配置

![image-20240306151138732](https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20240306151138732.png)

-   /gazebo节点发送的/legged_robot/xx信息供rviz读取可视化；/legged_robot_mpc_observation提供MPC输入。
-   /legged_robot_target发送/gait_type步态信息；/cmd_vel_filtered速度指令缓存；/legged_robot_mpc_target求解MPC结果。
-   /move_base_simple目标位置输入；/cmd_vel目标速度；

### 5.2 文件架构

-   hit_robot_ele_control
    -   legged_common
        -   hardware_interface
            -   ContactSensorInterface.h---接触传感器硬件接口
            -   HybridJointInterface.h--------自定义力位混合控制器
        -   output_color.h------------------------
    -   

## 2024.01.09 Debug

1.问题1：面足四点摩擦锥约束，如何实现？
方法：设置支撑点为八个点，一脚各四个。设置约束时，对于每个接触点只设置一个摩擦锥，游脚和支撑脚通过将力合成到单个点，进行约束，简化约束个数。

2.问题2：积分时，函数调用次数达到最大，原因？
ddp求解器的问题，换成sqp解决。

