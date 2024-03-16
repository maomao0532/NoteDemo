# 双足ZMP

## 1 LQR+ZMP

### 1.1 ZMP原理

将机器人看成桌子-小车模型，对其进行受力分析：

<img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20240128131759197.png" alt="image-20240128131759197" style="zoom:25%;" />

小车受重力$Mg$，惯性力$M\ddot{x}$，在ZMP处无力矩，可得方程：
$$
Mg(x-p_x)=M\ddot{x}z_c \\
p_x = x-\frac{z_c}{g}\ddot{x} \\
p_y = y-\frac{z_c}{g}\ddot{y}
$$
进而构建系统状态方程，以$[x, \dot{x}, \ddot{x}]^T$为状态变量，$\dddot{x}$为控制量：
$$
\frac{d}{dt}
\begin{bmatrix}
x \\
\dot{x} \\
\ddot{x} 
\end{bmatrix} = 
\begin{bmatrix}
0 & 1 & 0 \\
0 & 0 & 1 \\
0 & 0 & 0
\end{bmatrix}
\begin{bmatrix}
x \\
\dot{x} \\
\ddot{x} 
\end{bmatrix}
+
\begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix}u\\
p_x=
\begin{bmatrix}
1 & 0 & -\frac{z_c}{g}
\end{bmatrix}
\begin{bmatrix}
x \\
\dot{x} \\
\ddot{x} 
\end{bmatrix}
$$
对其进行离散化：
$$
x_{k+1} = Ax_k+Bu_k \\
p_k = Cx_k \\ 其中，
A =
\begin{bmatrix}
1 & T & T^2/2 \\
0 & 1 & T \\
0 & 0 & 1
\end{bmatrix},
B=
\begin{bmatrix}
T^3/6 \\
T^2/2 \\
T
\end{bmatrix},
C=
\begin{bmatrix}
1 & 0 & -z_c/g
\end{bmatrix}
$$
为了使输出$p_k$更好的跟踪参考位置$p_x^{ref}$，构建跟踪性能指标，当其最小时，为所需控制量：
$$
J=\sum_{j=1}^{\infty} \{ Q(p_j^{ref}-p_j)^2+Ru_j^2 \}
$$
Q,R为正的加权系数，当$J$极小时，有
$$
u_k=-Kx_k+
\begin{bmatrix}
f_1,f_2,\cdots,f_N
\end{bmatrix}
\begin{bmatrix}
p_{k+1}^{ref} \\ 
p_{k+2}^{ref} \\
\vdots \\
p_{k+N}^{ref}
\end{bmatrix}
$$
先规划出参考的ZMP位置（等差数列，样条插值等等），通过上式得到最优控制量，进而使用状态方程求得状态值，即质心位置（轨迹），再根据足端位置进行足端轨迹规划，通过逆运动学求出各关节角度。

###  1.2 流程

1.  设置发布频率、步长，创建PreviewControl类实例；
2.  设置支撑腿位置和ZMP位置；
3.  通过PreviewControl生成质心轨迹和左右脚轨迹；
4.  将每个轨迹通过逆运动学求出关节位置，再发布。

其中有两个循环：

-   外层为设置的行走步数w，执行w次，每一次会通过PreviewControl生成一步的所有轨迹；
-   内层为发布关节位置，将生成的轨迹中每一点，通过逆运动学求解再发布关节位置。

### 1.3 想法

1.  python的ZMP控制没有反馈，生成轨迹时，认为质心位置、足端位置都在理想位置；可以在每次生成轨迹前，**将PreviewControl中的质心、足端位置修改为实际位置，达到一定程度的修正**。

2.  不使用ros提供的位置控制，pid参数不好调；借用mpc中**游脚pd控制**，进行位置->力矩控制。

    补充：若双脚都采用pd控制，需要事先考虑上半身的重量，以保证机器人能正常站立，可以使用接触力和雅可比映射得到；但是运动起来单脚的又是需要借助mpc。。

3.  现代码中，每一步只算一次轨迹，可不可以多线程？**一个线程专门用来算轨迹，算完一次，紧接算下一次，提高控制频率，改善稳定性。**

下午任务：使用ros的pd进行控制；动起来；

明天：加入实际脚的落点；实现行走

修改落脚点高度 减少一点，保证落脚贴地面

### 1.4 仿真效果

<video id="video" controls="" src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/zmp.mp4" preload="none" >



## 2 MPC+ZMP

### 2.1 原理

根据ZMP的状态方程，构建MPC问题，用于跟踪x、y方向的ZMP位置，进而求出质心轨迹。
$$
X = A_{qp}x_0+B_{qp}U \\
Y = A_{qp}'x_0+B_{qp}'U
$$
其中：
$$
A_{qp}=
\begin{bmatrix}
{A} \\ {A}^2 \\ \vdots \\ {A}^k
\end{bmatrix},
B_{qp}=
\begin{bmatrix}
{B} & 0 & 0 & \cdots & 0 \\
{A}{B} & {B} & 0 & \cdots & 0\\
{A}^2{B} & {A}{B} & {B} & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
{A}^{k-1}{B} & {A}^{k-2}{B} & {A}^{k-3}{B} & \cdots & {B} 
\end{bmatrix} \\
A_{qp}'=
\begin{bmatrix}
C{A} \\ C{A}^2 \\ \vdots \\ C{A}^k
\end{bmatrix},
B_{qp}'=
\begin{bmatrix}
C{B} & 0 & 0 & \cdots & 0 \\
C{A}{B} & C{B} & 0 & \cdots & 0\\
C{A}^2{B} & C{A}{B} & C{B} & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
C{A}^{k-1}{B} & C{A}^{k-2}{B} & C{A}^{k-3}{B} & \cdots & C{B} 
\end{bmatrix} 
$$

构建二次规划问题，$R、Q$为权重系数：
$$
\min_{U} \frac{1}{2}U^THU+U^Tf\\
H=2({B_{qp}'}^TQB_{qp}'+R),
f=2{B_{qp}'}^TQ({A_{qp}'}x_0-Y_{ref})
$$
约束条件：

-   x方向：质心位置 速度 均大于0：

$$
\hat{A}_{qp}x_0+\hat{B}_{qp}U>0
$$

-   y方向：质心位置 速度  在一定范围内：

$$
-0.5<\hat{A}_{qp}x_0+\hat{B}_{qp}U<0.5
$$

### 2.2 仿真效果

<video id="video" controls="" src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/MPC%2BZMP.mp4" preload="none" >
