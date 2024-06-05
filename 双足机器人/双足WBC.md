# 双足WBC

```
参考文献：
[1] Kim D, Di Carlo J, Katz B, et al. Highly dynamic quadruped locomotion via whole-body impulse control and model predictive control[J]. arXiv preprint arXiv:1909.06586, 2019.
```

## 1. 相关基础知识

### 1.1 浮动基机器人动力学

由各活动关节角度$q_{joint}$和浮动基位姿$q_{float}$组成广义位置向量$q=\begin{bmatrix} q_{float} \\ q_{joint} \end{bmatrix}$其维度为$n_q=n_{joint}+6$，构建机器人动力学方程：
$$
A\ddot{q} + b + g = S_a^T\tau+J_c^Tf_r
$$
其中$A$为广义质量矩阵，$\ddot{q}=\begin{bmatrix} \ddot{q_{base}} \\ \ddot{q_{joint}} \end{bmatrix}$，$b,g$分别为科里奥利力和重力，$S_a=\begin{bmatrix} 0_{n_j\times 6}&I_{n_j\times n_j} \end{bmatrix}$，$J_c$为接触雅可比矩阵，可将接触力映射为关节力矩。假设有$n_c$个足端接触地面，每个接触力均为$f_{ri}=[T_x\ T_y\ T_z\ f_x\ f_y\ f_z]^T$，则$f_r=[f_{r1} ...f_{rn_c}]$为$6n_c\times 1$的向量，此时$J_c$为$6n_c \times n_q$的矩阵，转置后与$f_r$相乘，结果前六行为接触力的力矩和力的累加，后$n_{joint}$行为接触力映射的关节力矩，其中$J_c$是由$n_c$个接触雅可比拼接而成：$J_c=\begin{bmatrix} J_{c1} \\ \vdots \\ J_{cn_c} \end{bmatrix}$，每一个$J_{ci}$都是$6 \times n_q$的雅可比矩阵。

### 1.2 零空间投影矩阵

假设需要末端保持不动，转动某些关节，而不影响末端，则这些可转动的关节构成**零空间**。

产生条件：

- 机器人自由度大于空间自由度（如7自由度机械臂，末端只有六个自由度）
- 机器人自由度大于被控制自由度（如6自由度机械臂，只控制末端位置3自由度）

以速度零空间投影为例：
$$
\dot{x}_{des}=J(\theta) \cdot \dot{\theta}
$$
一般情况下，$J(\theta)$不可逆，故需要引入伪逆$J(\theta)^{\dagger}$，此为SVD伪逆，通过SVD分解进行计算，满足$JJ^{\dagger}=I$。可使得：
$$
\dot{\theta}=J(\theta)^{\dagger} \cdot \dot{x}_{des}
$$
而上式只是一个特解，通解的形式为：
$$
\dot{\theta}=J^{\dagger} \dot{x}_{des}+(I-J^{\dagger}J)z
$$
此时$z$可以随意取得，因为将上式代入方程可发现：
$$
\dot{x}_{des}=JJ^{\dagger}\dot{x}_{des}+(J-JJ^{\dagger}J)z=\dot{x}_{des}
$$
无论z取何值，上式始终成立，故令$N=I-J^{\dagger}J$成为$J$的零空间投影矩阵。满足$JN=0$，即$N$中的每一个列向量，都与$J$的行向量正交，$N$也可以看成是$Jx=0$的基础解系的部分构成的矩阵、$J$的行向量的正交补子空间。故$N$的求法有很多，上式只是一个简单方法。

可以借助上述原理，进行任务的分级操作，以两个任务为例：
$$
\left\{\begin{matrix} 
  \dot{x}_1=J_1\dot{q} \\  
  \dot{x}_2=J_2\dot{q} 
\end{matrix}\right.
$$
任务1优先级大于任务2，故先写出任务1的通解：
$$
\dot{q_1}=J_1^{\dagger} \dot{x}_{1}+N_1z
$$
将上式代入任务2，求得$z$，进而得到最终解：
$$
\dot{q_2}=J_1^{\dagger} \dot{x}_{1}+({J_2N_1})^{\dagger}(\dot{x}_2-J_2J_1^{\dagger}\dot{x}_1)
$$
如再有任务3：$\dot{x}_3=J_3\dot{q}$，就需要在$\dot{q}_2$后再加上一个零空间投影矩阵，此矩阵应满足即使$J_1$的零空间投影矩阵，也是$J_2$的零空间投影矩阵，故可以通过$\begin{bmatrix} J_1 \\ J_2 \end{bmatrix}x=0$的基础解系构成零空间投影矩阵，若设为$N_2$，则有：
$$
\dot{q}_3=\dot{q}_2+N_2z
$$
$z$通过将此式代入任务三求得。

如有n个任务，具体迭代形式，可见后续**位置、速度、加速度WBC**部分。

## 2. MIT WBC

首要任务，接触点保持接触，即$x_0^{des}=x_0, \dot{x_0^{des}}=0, \ddot{x_0^{des}}=0$。

### 2.1 位置速度WBC-KINWBC

#### 2.1.1 位置WBC

末端微小位移和关节微小角度之间的关系为：$\Delta x=J\cdot \Delta q$，位置WBC中每个任务都是上述表示方式。

已知：各任务的当前位置$x_0, x_1, x_2,...$，关节当前角度$\vec{q}$，期望位置$x_0^{des}, x_1^{des}, x_2^{des},...$

写成数学公式为：
$$
\left\{\begin{matrix}
x_0^{des}-x_0=J_0\cdot \Delta q \\
x_1^{des}-x_1=J_1\cdot \Delta q \\
\vdots \\
x_i^{des}-x_i=J_i\cdot \Delta q
\end{matrix}\right.
$$
求解得：
$$
\left\{\begin{matrix}
\begin{equation}
\begin{aligned}
\Delta q_0 &= 0 \\
\Delta q_1 &= \Delta q_0+(J_1N_0)^{\dagger}(x_1^{des}-x_1-J_1\Delta q_0) \\
          &= \Delta q_0+J_{1|pre}^{\dagger}(x_1^{des}-x_1-J_1\Delta q_0) \\
&\vdots \\
\Delta q_i &= \Delta q_{i-1}+(J_{i}N_{i-1})^{\dagger}(x_i^{des}-x_i-J_i\Delta q_{i-1}) \\
          &= \Delta q_{i-1}+J_{i|pre}^{\dagger}(x_i^{des}-x_i-J_i\Delta q_{i-1})
\end{aligned}
\end{equation}
\end{matrix}\right.
$$
其中：
$$
J_{i|pre}=J_iN_{i-1} \\
N_c \text{为} J_0(J_c),J_1,...J_i \text{和空间的零空间投影矩阵，满足} N_c\cdot J_k=0,k=0,1,..,i \\
N_{i-1} = N_0 N_{1|0} \cdots N_{i-1|i-2} \\
N_0 = I-J_c^{\dagger}J_c \\
N_{i|i-1} = I-J_{i|pre}^{\dagger}J_{i|pre}
$$

#### 2.1.2 速度WBC

末端速度与关节速度的关系：$\dot{x}=J \cdot \dot{q}$

已知：期望速度$\dot{x}_0^{des}, \dot{x}_1^{des}, \dot{x}_2^{des},...$

任务列表：
$$
\left\{\begin{matrix}
\dot{x}_0^{des}=J_0\cdot \dot{q} \\
\dot{x}_1^{des}=J_1\cdot \dot{q} \\
\vdots \\
\dot{x}_i^{des}=J_i\cdot \dot{q}
\end{matrix}\right.
$$
求解得：
$$
\left\{\begin{matrix}
\begin{equation}
\begin{aligned}
\dot{q}_0 &= 0 \\
\dot{q}_1 &= \dot{q}_0+(J_1N_0)^{\dagger}(\dot{x}_1^{des}-J_1\dot{q}_0) \\
          &= \dot{q}_0+J_{1|pre}^{\dagger}(\dot{x}_1^{des}-J_1\dot{q}_0) \\
&\vdots \\
\dot{q}_i &= \dot{q}_{i-1}+(J_{i}N_{i-1})^{\dagger}(\dot{x}_i^{des}-J_i\dot{q}_{i-1}) \\
          &= \dot{q}_{i-1}+J_{i|pre}^{\dagger}(\dot{x}_i^{des}-J_i\dot{q}_{i-1})
\end{aligned}
\end{equation}
\end{matrix}\right.
$$
其中$J_{i|pre}$求法与位置WBC相同。

### 2.2 加速度-力矩WBC-WBIC

#### 2.2.1 加速度WBC

末端加速度与关节加速度关系：$\dot{x}=J\dot{q} \rightarrow \ddot{x}=\dot{J}\dot{q}+J\ddot{q}$。

已知：期望加速度$\ddot{x}_0^{des}, \ddot{x}_1^{des}, \ddot{x}_2^{des},...$，关节速度$\dot{q}$，各任务的当前位置$x_0, x_1, x_2,...$速度$\dot{x}_0, \dot{x}_1, \dot{x}_2,...$

任务列表：
$$
\left\{\begin{matrix}
\ddot{x}_0^{cmd}=\dot{J}_0\cdot \dot{q} + J_0 \cdot \ddot{q} \\
\ddot{x}_1^{cmd}=\dot{J}_1\cdot \dot{q} + J_1 \cdot \ddot{q} \\
\vdots \\
\ddot{x}_i^{cmd}=\dot{J}_i\cdot \dot{q} + J_i \cdot \ddot{q}
\end{matrix}\right.
$$
求解得：
$$
\left\{\begin{matrix}
\begin{equation}
\begin{aligned}
\ddot{q}_0 &= \overline{J_c^{dyn}} (-\dot{J_c}\dot{q})\\
\ddot{q}_1 &= \ddot{q}_0+\overline{(J_1N_0^{dyn})}(\ddot{x}_1^{cmd}-\dot{J}_1\dot{q} - J_1\ddot{q}_0) \\
          &= \ddot{q}_0+\overline{J_{1|pre}^{dyn}}(\ddot{x}_1^{cmd}-\dot{J}_1\dot{q} - J_1\ddot{q}_0) \\
&\vdots \\
\dot{q}_i &= \ddot{q}_{i-1}+\overline{(J_{i}N_{i-1}^{dyn})}(\ddot{x}_i^{cmd}-\dot{J}_i\dot{q} - J_i\ddot{q}_{i-1}) \\
          &= \ddot{q}_{i-1}+\overline{J_{i|pre}^{dyn}}(\ddot{x}_i^{cmd}-\dot{J}_i\dot{q} - J_i\ddot{q}_{i-1})
\end{aligned}
\end{equation}
\end{matrix}\right.
$$
其中：
$$
J_{i|pre}^{dyn}=J_iN_{i-1}^{dyn} \\
N_i^{dyn} \text{为} J_0(J_c),J_1,...J_i \text{和空间的零空间投影矩阵，满足} N_i^{dyn}\cdot J_k=0,k=0,1,..,i \\
N_{i-1}^{dyn} = N_0^{dyn} N_{1|0}^{dyn} \cdots N_{i-1|i-2}^{dyn} \\
N_0^{dyn} = I-\overline{J_c}J_c \\
N_{i|i-1} = I-\overline{J_{i|pre}^{dyn}}J_{i|pre} \\
\ddot{x}_i^{cmd} = \ddot{x}_i^{des} + k_p(x_i^{des}-x_i)+k_d(\dot{x}_i^{des}-\dot{x_i})
$$
综上，根据机器人状态和各末端期望的状态，求解出了关节处的控制量：位置指令$q^{cmd}=q+\Delta q^{cmd}$，速度指令$\dot{q}^{cmd}$，加速度指令$\ddot{q}^{cmd}$。

#### 2.2.2 计算关节力矩

已知由MPC求出的接触反力$f_r^{MPC}$和由加速度WBC求出的关节加速度指令$\ddot{q}^{cmd}$。

理论上，求出的接触反力$f_r^{MPC}$和加速度指令$\ddot{q}^{cmd}$代入动力学方程应满足$A\ddot{q} + b + g = \begin{bmatrix} 0_{6\times6} \\  \tau \end{bmatrix}+J_c^Tf_r$

然而实际上并不满足，因此需要对$f_r$和$\ddot{q}^{cmd}$进行优化，以得到最终的指令。

构建优化代价函数：
$$
\min_{\boldsymbol{\delta}_{\mathbf{f}_r},\boldsymbol{\delta}_f}\quad\boldsymbol{\delta}_{\mathbf{f}_r}^\top\boldsymbol{Q}_1\boldsymbol{\delta}_{\mathbf{f}_r}+\boldsymbol{\delta}_f^\top\boldsymbol{Q}_2\boldsymbol{\delta}_f
\\
\begin{aligned}
\boldsymbol{S}_f\left(\boldsymbol{A}\ddot{\mathbf{q}}+\mathbf{b}+\mathbf{g}\right)&=\boldsymbol{S}_f\boldsymbol{J}_c^\top\mathbf{f}_r &\text{(floating base dyn.)} \\
\ddot{\mathbf{q}}&=\ddot{\mathbf{q}}^{\mathrm{cmd}}+\begin{bmatrix}\boldsymbol{\delta}_f\\\boldsymbol{0}_{n_j}\end{bmatrix} &\text{(acceleration)} \\
\mathbf{f}_r&=\mathbf{f}_r^\text{MPC}+\boldsymbol{\delta}_{\mathbf{f}_r} &\text{(reaction forces)} \\
\boldsymbol{W}\mathbf{f}_r&\geq\mathbf{0}, &\text{(contact force constraints)} 

\end{aligned}
$$
其中$S_f=[I_{6\times6}\ \  0]$，用于选取浮动基动力学。

最后，使用修正后的$f_r$和$\ddot{q}$，计算关节力矩$\begin{bmatrix} 0_{6\times6} \\  \tau \end{bmatrix}=A\ddot{q} + b + g -J_c^Tf_r$

使用得到的关节力矩，关节位置，关节速度，对电机进行力位混合控制。

<video id="video" controls="" src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/MPC%2BMITWBC%E8%B8%8F%E6%AD%A5.mp4" preload="none" >

<video id="video" controls="" src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/MPC%2BWBC%E7%AB%99%E7%AB%8B.mp4" preload="none" >

## 3 权重WBC

### 3.1 任务表现形式

$$
\left\{\begin{matrix}
\begin{equation}
\begin{aligned}
Ax-b=0 \\
Dx-f \le 0
\end{aligned}
\end{equation}
\end{matrix}\right.
$$

其中，优化变量$x=[q'' \ \ F_r \ \ \tau]$，分别为广义坐标加速度（6+12在**局部坐标系**中）、足端作用反力（6+6）、关节力矩（12）。

### 3.2 任务列表

#### 3.2.1 动力学等式

$$
M\ddot{q}+C\dot{q}+g=
\begin{bmatrix}
0 \\
\tau
\end{bmatrix}
+ J_c^{T} F_r
$$

写成任务形式为：
$$
A=[M \ \  -J_c^{T} \ \ -s^T];
J_c=\begin{bmatrix}
J_{left} \\
J_{right}
\end{bmatrix};
s = [0_{12\times6} \ \ \ I_{12\times12}] \\
b=-C\dot{q}-g
$$
上述变量可通过pinocchio和当前状态计算得到。

#### 3.2.2 关节力矩不等式

$$
-\tau_{max} \le \tau \le \tau_{max}
$$

写成任务形式为：
$$
D = 
\begin{bmatrix}
0_{18\times12} & 0_{12\times12} & I_{12\times12} \\
0_{18\times12} & 0_{12\times12} & -I_{12\times12}                          
\end{bmatrix},
f=\begin{bmatrix}
\tau_{max} \\
\tau_{max}                          
\end{bmatrix}
$$
相关变量由wbc初始化时给出。

#### 3.2.3 接触点加速度等式

$$
a=(J\dot{q})'=\dot{J}\dot{q}+J\ddot{q}=0
$$

写成任务形式为：
$$
A=[J\ \ 0\ \ 0];b=-\dot{J}\dot{q}
$$
$J$为接触脚的雅可比矩阵，其他变量可通过pinocchio计算。

#### 3.2.4 摩擦锥

**等式：游脚接触力为0**
$$
A=[0 \ \ I \ \ 0];b=0
$$
$I$为游脚所对应的列。

**支撑脚：满足摩擦锥**
$$
D=[0\ \ f_{ri} \ \ 0];f=0
$$
其中$f_{ri}$对应接触的脚：
$$
f_{ri}=
\begin{bmatrix}
0 & 0 & -1 \\
1 & 0 & -\mu \\
-1 & 0 & -\mu \\
0 & 1 & -\mu \\
0 & -1 & -\mu \\
\end{bmatrix}
$$

#### 3.2.5 浮动基线加速度等式

$$
A=[R \ \ 0];b=a_{base}+K_perr_{pos}+K_derr_{vel}
$$

其中$R$为浮动基的旋转矩阵，左乘$R$可将局部坐标系转换成世界坐标系。

期望的位置和速度可使用MPC优化的结果。

#### 3.2.6 浮动基角加速度等式

$$
A=[R \ \ 0];b=a_{base}+K_perr_{pos}+K_derr_{vel}
$$

与线加速度类似。

#### 3.2.7 游脚等式

$$
a_{swing} = \dot{J}\dot{q}+J\ddot{q} = a_{swing}+K_perr_{swing}+K_derr_{swing}
$$

写成任务形式：
$$
A=[J \ \ 0\ \ 0];b=a_{swing}+K_perr_{swing}+K_derr_{swing}-\dot{J}\dot{q}
$$
游脚的位置和速度通过规划获得。

#### 3.2.8 接触力等式

优化变量中的接触力$F_r$等于MPC的求解结果，任务形式：
$$
A=[0 \ \ I \ \ 0],b=F_{rdes}
$$

<video id="video" controls="" src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/MPC%2B%E6%9D%83%E9%87%8DWBC%E8%B8%8F%E6%AD%A5.mp4" preload="none" >

<video id="video" controls="" src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/MPC%2B%E6%9D%83%E9%87%8DWBC%E8%A1%8C%E8%B5%B0.mp4" preload="none" >
## 4. 缓冲WBC

尝试：将MPC的权重也进行缓慢变化；缓冲MPC+缓冲WBC；比较MPC+WBC的足底反力和稳定性。



