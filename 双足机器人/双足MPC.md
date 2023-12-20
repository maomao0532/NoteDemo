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





## 3. MPC构建





## 4. Debug

### 2023.12.20

问题1：MPC控制足端单脚支撑时，支撑腿的yaw角会发生转动。
~~解决：PD切换时，存在PD和MPC作用在同一支脚上？？~~
解决：在一只脚抬到一半时，MPC会在hip_yaw产生较大的力矩？？不是较大，是有个==突变==。
突变原因：MPC计算时间太长；~~订阅的关节角度不连续，导致雅可比的突变~~