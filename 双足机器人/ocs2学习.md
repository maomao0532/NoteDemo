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

### 4. 四足机器人案例

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

$\boldsymbol{q}_{b},\boldsymbol{q}_{j}$分别为baselink在**世界坐标系下**的位姿$\begin{aligned}\boldsymbol{q}_b=(\boldsymbol{r}_{IB},\boldsymbol{\Phi}_{IB}^{zyx})\end{aligned}$，各关节角度
