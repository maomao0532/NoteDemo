# 双足MPC 

## 1. 正逆运动学推导

### 1.1 正运动学

参数设置：

-   $\vec{b_{ij}}$表示关节$i$相对于关节$j$的位置；
-   $\vec{a_i}$关节$i$转轴方向；
-   $q_i$关节$i$转角，$q_1$恒为0。

以左腿为例：从上至下依次为：Com, plevis_yaw $\vec{a_1}$, left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee_pitch, left_ankle_pitch, left_ankle_roll.
$$
\vec{a_1} = \begin{bmatrix}  
  0  \\  
  0  \\    
  1   
\end{bmatrix},
\vec{a_2} = \begin{bmatrix}  
  0  \\  
  0  \\    
  1   
\end{bmatrix},
\vec{a_3} = \begin{bmatrix}  
  1  \\  
  0  \\    
  0   
\end{bmatrix},
\vec{a_4} = \begin{bmatrix}  
  0  \\  
  1  \\    
  0   
\end{bmatrix},
\vec{a_5} = \begin{bmatrix}  
  0  \\  
  1  \\    
  0   
\end{bmatrix},
\vec{a_6} = \begin{bmatrix}  
  0  \\  
  1  \\    
  0   
\end{bmatrix},
\vec{a_7} = \begin{bmatrix}  
  1  \\  
  0  \\    
  0   
\end{bmatrix}
$$
left_ankle_roll相对于Com的其次变换矩阵为$\sideset{^0}{_7}T=\sideset{^0}{_1}T\sideset{^1}{_2}T\sideset{^2}{_3}T\sideset{^3}{_4}T\sideset{^4}{_5}T\sideset{^5}{_6}T\sideset{^6}{_7}T$，其中具体为：
$$
\sideset{^0}{_1}T=\begin{bmatrix}  
  1 & 0 & 0 & \\
  0 & 1 & 0 & \vec{b_{10}} \\
  0 & 0 & 1 & \\
  0 & 0 & 0 & 1 
\end{bmatrix} ,
\sideset{^1}{_2}T=\begin{bmatrix}  
  cos(q_2) & -sin(q_2) & 0 & \\
  sin(q_2) & cos(q_2) & 0 & \vec{b_{21}} \\
  0 & 0 & 1 & \\
  0 & 0 & 0 & 1 
\end{bmatrix},
\sideset{^2}{_3}T=\begin{bmatrix}  
  1 & 0 & 0 & 0\\
  0 & cos(q_3) & -sin(q_3) & 0 \\
  0 & sin(q_3) & cos(q_3) & 0\\
  0 & 0 & 0 & 1 
\end{bmatrix},
\sideset{^3}{_4}T=\begin{bmatrix}  
  cos(q_4) & 0 & sin(q_4) & \\
  0 & 1 & 0 & \vec{b_{43}} \\
  -sin(q_4) & 0 & cos(q_4) & \\
  0 & 0 & 0 & 1 
\end{bmatrix},
$$

$$
\sideset{^4}{_5}T=\begin{bmatrix}  
  cos(q_5) & 0 & sin(q_5) & \\
  0 & 1 & 0 & \vec{b_{54}} \\
  -sin(q_5) & 0 & cos(q_5) & \\
  0 & 0 & 0 & 1 
\end{bmatrix},
\sideset{^5}{_6}T=\begin{bmatrix}  
  cos(q_6) & 0 & sin(q_6) & \\
  0 & 1 & 0 & \vec{b_{65}} \\
  -sin(q_6) & 0 & cos(q_6) & \\
  0 & 0 & 0 & 1 
\end{bmatrix},
\sideset{^6}{_7}T=\begin{bmatrix}  
  1 & 0 & 0 & \\
  0 & cos(q_7) & -sin(q_7) & \vec{b_{76}} \\
  0 & sin(q_7) & cos(q_7) & \\
  0 & 0 & 0 & 1 
\end{bmatrix}
$$

**给定$q_{2-7}$可求出ankle_roll相对质心的位姿，右腿正运动学改变$\vec{b_{ij}}$即可。**

### 1.2 逆运动学解析解

类似六自由度机械臂，需满足**Pieper条件**：三个连续的关节轴交于一点或平行，腿部有三个平行的pitch。

输入：ankle_roll相对于质心的位姿（**注意是在质心坐标系下**）
输出：各关节的角度

给定位姿，相当于已知$\sideset{^0}{_7}T$，由于$\sideset{^0}{_1}T$固定，故已知$\sideset{^1}{_7}T$。
考虑4、5、6为平行的pitch轴:
$$
\sideset{^1}{_7}T=\sideset{^1}{_2}T\sideset{^2}{_3}T\sideset{^3}{_4}T\sideset{^4}{_5}T\sideset{^5}{_6}T\sideset{^6}{_7}T\Rightarrow \sideset{^3}{_7}T=\sideset{^3}{_6}T\sideset{^6}{_7}T=(\sideset{^2}{_3}T)^{-1}(\sideset{^1}{_2}T)^{-1}(\sideset{^1}{_7}T)
$$
其中：
$$
\sideset{^1}{_7}T=
\begin{bmatrix}  
  n_x & o_x & a_x & p_x \\
  n_y & o_y & a_y & p_y \\
  n_z & o_z & a_z & p_z\\
  0 & 0 & 0 & 1 
\end{bmatrix}
$$

$$
\begin{align*}
\sideset{^3}{_6}T&=\begin{bmatrix}  
  cos(q_4) & 0 & sin(q_4) & 0 \\
  0 & 1 & 0 & y_4 \\
  -sin(q_4) & 0 & cos(q_4) & z_4\\
  0 & 0 & 0 & 1 
\end{bmatrix}
\begin{bmatrix}  
  cos(q_5) & 0 & sin(q_5) & 0 \\
  0 & 1 & 0 & y_5 \\
  -sin(q_5) & 0 & cos(q_5) & z_5\\
  0 & 0 & 0 & 1 
\end{bmatrix}
\begin{bmatrix}  
  cos(q_6) & 0 & sin(q_6) & 0 \\
  0 & 1 & 0 & y_6 \\
  -sin(q_6) & 0 & cos(q_6) & z_6\\
  0 & 0 & 0 & 1 
\end{bmatrix}
\\
&=
\begin{bmatrix}  
  cos(q_4+q_5+q_6) & 0 & sin(q_4+q_5+q_6) & sin(q_4+q_5)z_6+sin(q_4)z_5 \\
  0 & 1 & 0 & y_4+y_5+y_6 \\
  -sin(q_4+q_5+q_6) & 0 & cos(q_4+q_5+q_6) & cos(q_4+q_5)z_6+cos(q_4)z_5+z_4\\
  0 & 0 & 0 & 1 
\end{bmatrix}
\end{align*}
$$

$$
(\sideset{^2}{_3}T)^{-1}=
\begin{bmatrix}  
  1 & 0 & 0 & 0 \\
  0 & cos(q_3) & sin(q_3) & 0 \\
  0 & -sin(q_3) & cos(q_3) & 0\\
  0 & 0 & 0 & 1 
\end{bmatrix},
(\sideset{^1}{_2}T)^{-1}=
\begin{bmatrix}  
  cos(q_2) & sin(q_2) & 0 & -y_2sin(q_2) \\
  -sin(q_2) & cos(q_2) & 0 & -y_2cos(q_2) \\
  0 & 0 & 1 & -z_2\\
  0 & 0 & 0 & 1 
\end{bmatrix}
$$

带入得：
$$
\begin{align*}
\sideset{^3}{_6}T\sideset{^6}{_7}T
&=
\begin{bmatrix}  
  cos(q_4+q_5+q_6) & 0 & sin(q_4+q_5+q_6) & sin(q_4+q_5)z_6+sin(q_4)z_5 \\
  0 & 1 & 0 & y_4+y_5+y_6 \\
  -sin(q_4+q_5+q_6) & 0 & cos(q_4+q_5+q_6) & cos(q_4+q_5)z_6+cos(q_4)z_5+z_4\\
  0 & 0 & 0 & 1 
\end{bmatrix}
\begin{bmatrix}  
  1 & 0 & 0 & 0\\
  0 & cos(q_7) & -sin(q_7) & 0 \\
  0 & sin(q_7) & cos(q_7) & z_7\\
  0 & 0 & 0 & 1 
\end{bmatrix}
\\ &=
\begin{bmatrix}  
  cos(q_4+q_5+q_6) & sin(q_4+q_5+q_6)sin(q_7) & sin(q_4+q_5+q_6)cos(q_7) & sin(q_4+q_5+q_6)z_7+sin(q_4+q_5)z_6+sin(q_4)z_5 \\
  0 & cos(q_7) & -sin(q_7) & y_4+y_5+y_6 \\
  -sin(q_4+q_5+q_6) & cos(q_4+q_5+q_6)sin(q_7) & cos(q_4+q_5+q_6)cos(q_7) & cos(q_4+q_5+q_6)z_7+cos(q_4+q_5)z_6+cos(q_4)z_5+z_4\\
  0 & 0 & 0 & 1 
\end{bmatrix}
\end{align*}
$$

$$
\begin{align*}
(\sideset{^2}{_3}T)^{-1}(\sideset{^1}{_2}T)^{-1}\sideset{^1}{_7}T
&=
\begin{bmatrix}  
  1 & 0 & 0 & 0 \\
  0 & cos(q_3) & sin(q_3) & 0 \\
  0 & -sin(q_3) & cos(q_3) & 0\\
  0 & 0 & 0 & 1 
\end{bmatrix}
\begin{bmatrix}  
  cos(q_2) & sin(q_2) & 0 & -y_2sin(q_2) \\
  -sin(q_2) & cos(q_2) & 0 & -y_2cos(q_2) \\
  0 & 0 & 1 & -z_2\\
  0 & 0 & 0 & 1 
\end{bmatrix}
\begin{bmatrix}  
  n_x & o_x & a_x & p_x \\
  n_y & o_y & a_y & p_y \\
  n_z & o_z & a_z & p_z\\
  0 & 0 & 0 & 1 
\end{bmatrix}\\
&=
\begin{bmatrix}  
c_2n_x+s_2n_y & c_2o_x+s_2o_y & c_2a_x+s_2a_y & c_2p_x+s_2p_y-s_2y_2 \\
-c_3s_2n_x+c_3c_2n_y+s_3n_z & -c_3s_2o_x+c_3c_2o_y+s_3o_z & -c_3s_2a_x+c_3c_2a_y+s_3a_z & -c_3s_2p_x+c_3c_2p_y+s_3p_z-c_2c_3y_2-s_3z_2 \\
s_2s_3n_x-s_3c_2n_y+c_3n_z & s_2s_3o_x-s_3c_2o_y+c_3o_z & s_2s_3a_x-s_3c_2a_y+c_3a_z & s_2s_3p_x-s_3c_2p_y+c_3p_z+c_2c_3y_2-c_3z_2\\
0 & 0 & 0 & 1
\end{bmatrix}
\end{align*}
$$

其中$c_2=cos(q_2)$

根据上两个矩阵第二行相等可得方程组：
$$
\begin{align}
\left\{\begin{matrix} 
  -c_3s_2n_x+c_3c_2n_y+s_3n_z &=& 0 \\  
  -c_3s_2o_x+c_3c_2o_y+s_3o_z &=& c_7  \\
-c_3s_2a_x+c_3c_2a_y+s_3a_z &=& -s_7 \\
-c_3s_2p_x+c_3c_2p_y+s_3p_z-c_2c_3y_2-s_3z_2 &=& y_4+y_5+y_6 
\end{matrix}\right.
\end{align}
$$
由于姿态矩阵永远是正交阵，令$R_1=[n_x,n_y,n_z]^T,R_2=[o_x,o_y,o_z]^T,R_3=[a_x,a_y,a_z]^T$，则$R_1,R_2,R_3$正交。由上式第一个方程可以看出，令$g=[-c_3s_2,c_3c_2,s_3]^T$，则有$<g,R_1>=0$，故$g \perp R_1$，则$g$可由$R_2,R_3$线性表示。

令$g=mR_2+nR_3$ ，又$||g||=1$，故$m^2+n^2=1$，可构建以下方程：
$$
\begin{align}
\left\{\begin{matrix} 
 (mo_x+na_y)p_x+(mo_y+na_y)(p_y-y_2)+(mo_z+na_z)(p_z-z_2) &=& y_4+y_5+y_6 \\
 m^2+n^2 &=& 1
\end{matrix}\right. 
\end{align}
$$
可解出$m,n$，进而得到$g$，即$-c_3s_2,c_3c_2,s_3$已知，可得到$q_2$(两个解，结合关节约束条件，则只剩一个解)：
$$
q_{21} = atan({c_3s_2}/{c_3c_2}),q_{22}=q_{21} \pm \pi(q_{21}>0时为-，反之为+)
$$
将解出的$q_2$带入$-c_3s_2,c_3c_2$，可解出$c_3$，结合$g$中的$s_3$，可得$q_3$：
$$
q_3 = atan2(s_3, c_3)
$$

将$q_2,q_3$回代上面的“四元”方程组中，求出$c_7,s_7$，可得$q_7$：
$$
q_7 = atan2(s_7, c_7)
$$
下面考虑$\sideset{^3}{_6}T=\sideset{^3}{_7}T(\sideset{^6}{_7}T)^{-1}$，由于$q_2,q_3,q_7$已经求得，故$\sideset{^3}{_7}T,(\sideset{^6}{_7}T)^{-1}$均已知，则$\sideset{^3}{_6}T$已知：
$$
\begin{align}
\sideset{^3}{_6}T
&=
\begin{bmatrix}  
  cos(q_4+q_5+q_6) & 0 & sin(q_4+q_5+q_6) & sin(q_4+q_5)z_6+sin(q_4)z_5 \\
  0 & 1 & 0 & y_4+y_5+y_6 \\
  -sin(q_4+q_5+q_6) & 0 & cos(q_4+q_5+q_6) & cos(q_4+q_5)z_6+cos(q_4)z_5+z_4\\
  0 & 0 & 0 & 1 
\end{bmatrix}\\
&=
\begin{bmatrix}  
  A & O & B & D \\
   &  &  & E \\
   &  &  & F\\
  0 & 0 & 0 & 1 
\end{bmatrix}
\end{align}
$$
比较对应位置，构建方程组：
$$
\begin{align}
\left\{\begin{matrix} 
 sin(q_4+q_5)z_6+sin(q_4)z_5 &=& D \\
 cos(q_4+q_5)z_6+cos(q_4)z_5 &=& G &=& F-z_4
\end{matrix}\right. 
\end{align}
$$
可以看出：
$$
D^2+G^2=z_6^2+z_5^2+2z_6z_5sin(q_4+q_5)sin(q_4)+2z_6z_5cos(q_4+q_5)cos(q_4)\\

D^2+G^2-z_6^2-z_5^2=2z_6z_5cos(q_4+q_5-q_4)=2z_6z_5cos(q_5)\\

q_5=acos(\frac{D^2+G^2-z_6^2-z_5^2}{2z_6z_5})
$$
$acos$的返回值是$[0,\pi)$，恰好在$q_5$限制的范围内，刚好唯一解。
$$
\left.\begin{matrix} 
  [cos(q_5)z_6+z_5]sin(q_4)+sin(q_5)cos(q_4)z_6 = D\\ 
  [cos(q_5)z_6+z_5]cos(q_4)+sin(q_5)sin(q_4)z_6 = G 
\end{matrix}\right\}\Rightarrow 
\left\{\begin{matrix} 
  sin(q_4)\\
  cos(q_4)
\end{matrix}\right.
\Rightarrow
q_4=atan[sin(q_4),cos(q_4)]
$$
最后解得$q_6$：
$$
\left.\begin{matrix} 
  cos(q_4+q_5+q_6)=A\\ 
  sin(q_4+q_5+q_6)=B 
\end{matrix}\right\}\Rightarrow 
q_4+q_5+q_6
\Rightarrow
q_6
$$
此逆运动学恰好有唯一解，为确保也可通过正运动学再进行验证。

## 2. 动力学建模

<img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image-20231221210631965.png" alt="image-20231221210631965" style="zoom:20%;" />

上图为创建的动力学模型：假设质量集中在上半身，两腿为轻杆，线接触，地面反作用力（矩）集中在一个作用点处，受$F_x,F_y,F_z,M_y,M_z$，且$M_x=0$。

设机器人加速度（质心）加速度为$\ddot{\vec{p_c}}$，质心角动量变化率为$\dot{\vec{H_c}}=I_c\cdot\dot{\vec{w_c}}$，接触力为$u=[\vec{F_1},\vec{F_2},\vec{M_1},\vec{M_2}]^T$，其中$\vec{F_i}=[F_{ix},F_{iy},F_{iz}]^T,M_i=[M_{ix},M_{iy},M_{iz}]^T$，1表示右腿，2表示左腿，由牛二有：
$$
\begin{bmatrix}  
  D_1 \\
  D_2 \\
\end{bmatrix}u
=
\begin{bmatrix}
m(\ddot{p_c}+g) \\
\dot{\vec{H}}
\end{bmatrix}
$$
其中：
$$
D_1=
\begin{bmatrix}
I_{3\times3} & I_{3\times3} & 0_{3\times3} & 0_{3\times3}  
\end{bmatrix}\\
D_2=
\begin{bmatrix}
(\vec{p_1}-\vec{p_c})\times & (\vec{p_2}-\vec{p_c})\times & I_{3\times3} & I_{3\times3}
\end{bmatrix}
$$
具体解释为：
$$
D_1u=\vec{F_1}+\vec{F_2}=
\begin{bmatrix}
F_{1x}+F_{2x} \\
F_{1y}+F_{2y} \\
F_{1z}+F_{2z}
\end{bmatrix}
=
m(\ddot{p_c}+g)
=
\begin{bmatrix}
m\ddot{x} \\
m\ddot{y} \\
m(\ddot{z}+g)
\end{bmatrix}
$$

$$
\begin{align}
D_2u
&=
(\vec{p_1}-\vec{p_c})\times\vec{F_1}+(\vec{p_2}-\vec{p_c})\times\vec{F_2}+
L\cdot\vec{M_1}+L\cdot\vec{M_2}\\
&=
\begin{bmatrix}
M_{F_1x}+M_{F_2x}+M_{1x}+M_{2x} \\
M_{F_1y}+M_{F_2y}+M_{1y}+M_{2y} \\
M_{F_1z}+M_{F_2z}+M_{1z}+M_{2z}
\end{bmatrix}
=
I_c\cdot\dot{\vec{w_c}}
=
\begin{bmatrix}
I_{xx}w_x+I_{xy}w_y+I_{xz}w_z \\
I_{yx}w_x+I_{yy}w_y+I_{yz}w_z \\
I_{zx}w_x+I_{zy}w_y+I_{zz}w_z
\end{bmatrix}
\end{align}
$$

设欧拉角RPY为$\phi,\theta,\psi$，则欧拉角速度和角速度关系为：（均在全局坐标系$\sum W$下）
$$
\dot{\Theta}=
\begin{bmatrix}
\dot{\phi} \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix}
=
\begin{bmatrix}
cos\psi/cos\theta & sin\psi/cos\theta & 0 \\
-sin\psi & cos\psi & 0 \\
sin\theta cos\psi/cos\theta & sin\theta sin\psi/cos\theta & 1
\end{bmatrix}
\begin{bmatrix}
w_{ox} \\
w_{oy} \\
w_{oz}
\end{bmatrix}
=
R_zw
$$
**上式补充证明如下：**

设欧拉角为$\phi(t),\theta(t),\psi(t)$，旋转矩阵为$R(\phi(t),\theta(t),\psi(t))$。
一刚体上一点坐标为$x_o(t)$，有$x_o(0)=x_0$，则有$x_o(t)=R(t)x_0$（仅转动）：
$$
v_o(t)=\frac{dx_o(t)}{dt}=
\frac{dR}{dt}x_0+R\frac{dx}{dt}=\frac{dR}{dt}x_0
$$
进而，该点角速度有：
$$
v_o(t)=w_o(t)\times x_o(t)=\Omega_ox_o(t)=\Omega_0R(t)x_0
,其中
\Omega_0=
\begin{bmatrix}
0 & -w_{oz} & w_{oy} \\
w_{oz} & 0 & -w_{ox} \\
-w_{oy} & w_{ox} & 0
\end{bmatrix}
$$
故有$\Omega_0R(t)=\frac{dR}{dt}$。

综上，构建状态空间方程：
$$
\frac{d}{dt}
\begin{bmatrix}
\Theta \\
p_c\\
w\\
\dot{p_c}
\end{bmatrix}
=
A
\begin{bmatrix}
\Theta \\
p_c\\
w\\
\dot{p_c}
\end{bmatrix}
+Bu
+
\begin{bmatrix}
0\\
0\\
0\\
g
\end{bmatrix}
\\
A=
\begin{bmatrix}
0_{3\times3} & 0_{3\times3} & R_z & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & I_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3}
\end{bmatrix},
B=
\begin{bmatrix}
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
{I_G}^{-1}(\vec{p_1}-\vec{p_c})\times & {I_G}^{-1}(\vec{p_2}-\vec{p_c})\times & {I_G}^{-1} & {I_G}^{-1} \\
I_{3\times3}/m & I_{3\times3}/m & 0_{3\times3} & 0_{3\times3}
\end{bmatrix}
$$
其中$I_G$为$\sum W$中body相对于质心，轴为$\sum W$轴的转动惯量矩阵，可使用$I_G=R_zI_bR_z^T$求得，其中，$I_b$为body相对自身固连质心坐标系的转动惯量（定值）。

为将状态空间线性化



## 3. MPC构建





## 4. Debug

### 2023.12.20

问题1：MPC控制足端单脚支撑时，支撑腿的yaw角会发生转动。
~~解决：PD切换时，存在PD和MPC作用在同一支脚上？？~~
解决：在一只脚抬到一半时，MPC会在hip_yaw产生较大的力矩？？不是较大，是有个==突变==。
突变原因：==MPC计算时间太长==；~~订阅的关节角度不连续，导致雅可比的突变~~

### 2023.12.21

问题1：MPC控制足端单脚支撑时，支撑腿的yaw角会发生转动。
解决：==MPC计算时间太长，通过降低gazebo仿真速率，可解决问题==，上限速度$0.5m/s$==暂时==，可以通过提高踏脚频率，提高速度，最终上限未测。

<img src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/image-20231221200641155.png" alt="image-20231221200641155" style="zoom:33%;" />

问题2：转向能力欠缺，无法按照给定角速度转弯。
原因：期望速度是在世界坐标系下的（==尚未修改==），但单独给角速度也无法实现期望速度。

**踏步效果：**

<video id="video" controls="" src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/move.mp4" preload="none" >


**步频$0.7s$，速度$0.3m/s$：**

<video id="video" controls="" src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/move_0.3.mp4" preload="none" >

**步频$0.6s$，速度$0.5m/s$：**

<video id="video" controls="" src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/move_0.5.mp4" preload="none" >
**后退$0.2m/s$效果：**

<video id="video" controls="" src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/move_0.2.mp4" preload="none" >
### 2024.1.25

在一定高度上进行踏步

平地时，质心高度0.988761；baselink高度0.89683174；

方法：通过设置**质心高度为支撑脚上方0.988761的位置**，达到在一定高度进行踏步的目的。

### 2024.1.26

问题：转圈走会在0.4左右位置卡住，yaw不增加
方法：转速设为0.1会出现上述问题，**增大转速**可解决。

问题：一边转圈，一边走
方法：在更新机器人欧拉角等状态时，将设置的速度（如x方向0.2，y方向为0，z方向为0）**左乘旋转矩阵**，**将质心坐标系下的速度，转换成世界坐标系下。**

**下坡**

<video id="video" controls="" src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/%E5%A6%82%E4%B8%8B.mp4" preload="none" >

**上坡**

<video id="video" controls="" src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/%E6%96%9C%E5%9D%A1.mp4" preload="none" >

**转圈**

<video id="video" controls="" src="https://typora-picture-01.oss-cn-shenzhen.aliyuncs.com/image/%E8%BD%AC%E5%9C%88.mp4" preload="none" >
