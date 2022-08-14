# 差分动态规划-DDP

An introduction of DDP and implementation in python

# Differential Dynamic Programming

This project details the formulation of Differential Dynamic Programming in discrete time. Implementation of the algorithms for various systems including an inverted pendulum and a cart-pole have also been documented in this report. 
该项目详细介绍了离散时间的微分动态编程的表述。本报告还将尝试在python中实现各种系统的算法，包括倒立摆和车杆的算法。

## Overview

The optimal control problem is defined as: 

$$
    V(x(t_0), t_0) = \underset{u}{min}\left[\phi(\mathbf{x}(t_f), t_f) + \int_{t_0}^{t_N} l(\mathbf{x, u}, t)dt \right]
$$

where x and u represent the state and the controls with dimensionalities n and m respectively.  The general dynamics of the system are represented as follows: 

$$
    \frac{dx}{dt} = f(\mathbf{x, u}, t)
$$

The Bellman principle in discrete time is given as: 

$$
V(\mathbf{x}(t_k),t_k)=\underset{u}{min}\left[L(\mathbf{x}(t_k),\mathbf{u}(t_k),t_k) + V(\mathbf{x}(t_{k+1}),t_{k+1})\right]
$$

where L and V represent the running cost in discrete time and value function at $t= t_k$. The state value function is defined as follows: 

$$
Q(\mathbf{x}(t_k), \mathbf{u}(t_k)) =  \left[L(\mathbf{x}(t_k),\mathbf{u}(t_k),t_k) + V(\mathbf{x}(t_{k+1}),t_{k+1})\right]
$$

## DDP Implimentation

The steps needed to derive the Differential Dynamic Programming scheme are detailed in this [source](https://ieeexplore.ieee.org/document/5530971).

This section details the implementation of the DDP scheme derived in the previous section. DDP will be applied to an inverted pendulum, and a cart pole system. In general each implementation will utilize the  pseudocode detailed in the table below: 

推导微分动态编程方案所需的步骤详见本[来源]（https://ieeexplore.ieee.org/document/5530971）。

本节详细介绍了上一节中得出的DDP方案的实施。DDP将被应用于一个倒立摆和一个车杆系统。一般来说，每个实现都将利用下表详述的伪代码。

| Steps | Description                                                  | |
| ----- | ------------------------------------------------------------ |------------------------------------------------|
| 1     | Given the nominal states and controls, determine the linearized dynamics of the system |给出名义状态和控制，确定系统的线性化动力学特性 |
| 2     | Calculate the second order expansion of the state value function |计算状态值函数的二阶扩展|
| 3     | Back-propagate the value function, its gradient and its hessian | 反向传播价值函数、其梯度和其海西。|
| 4     | Update the controls using the optimal control correction     | 使用最佳控制修正来更新控制 |
| 5     | Calculate the new optimal trajectory by applying the new controls to the system dynamics | 通过对系统动力学应用新的控制，计算新的最佳轨迹|
| 6     | Set the new nominal trajectory and control as the calculated optimal trajectory and control | 将新的额定轨迹和控制设置为计算出的最佳轨迹和控制|
| 7     | Check for convergence, repeat steps 1-7 until converged      |  检查收敛情况，重复步骤1-7，直到收敛。|


#### Inverted Pendulum 

This section describes the implementation of our DDP derivation to an inverted pendulum system. It's dynamics are given with the equation: 

$$
I\ddot{\theta}+b\dot{\theta}+mglsin(\Theta)=u
$$

\noindent Where $I=ml^2,\,g=9.81\,m/s^2$, m is the mass, l is the length, b is damping, and f is the control. This equation is then converted to state space form:\\

$$
F(x,u,t)=\frac{dx}{dt}=\left[ \begin{array}{c} \dot{x_1} \\ \dot{x_2} \end{array}\right] = \left[ \begin{array}{c} x_2 \\ \frac{u}{ml^2} - \frac{g}{l}sin(x_1)-\frac{b}{ml^2}x_2\end{array}\right]=\left [\begin{array}{c} f(x_1,x_2) \\ g(x_1,x_2)\end{array}\right ]
$$

Where $x_1=\theta$ and $x_2=\dot{\theta}$. $\frac{dx}{dt}$ is then used in step 5 of the DDP Algorithm as described the table above with the relation $x(k+1)=x(k)+\frac{dx}{dt}(k)dt$. In order to find $\triangledown_xF$ and $\triangledown_uF$, the Hessian of the dynamics matrix above must be found with respect to x and u. 

##### Results 

For a simple case in which the pendulum starts at rest, DDP is applied to attempt the flip the pendulum by 180deg (the classic problem). The results of the algorithm are given in the figure below: 

 <img src="./invertedPendulum/results.png" width="600">


#### Cart-Pole 

This section describes the implementation of our DDP derivation to a cart-pole system. It's dynamics are given with the equations: 

$$
\begin{align*}
\ddot{x} &= \frac{f+m_psin\theta(l\dot{\theta}+gcos\theta)}{m_c+m_psin^2\theta}  
\ddot{\theta} &= \frac{-fcos\theta-m_pl\dot{\theta}^2cos\theta sin\theta-(m_c+m_p)gsin\theta}{l(m_c+m_psin^2\theta)}
\end{align*}
$$

Where $m_c=1.0\,kg\, m_p=0.01\,kg,\,and\,l=0.25\,m$. These equations are then converted to state space form:

$$
F(x,u,t)=\frac{dx}{dt}=\left[\begin{array}{c} \dot{x_1} \\ \dot{x_2} \\ \dot{x_3} \\ \dot{x_4} \end{array}\right] = \left[ \begin{array}{c}  x_2 \\ a \\ x_4 \\ b\end{array}\right]=\left [\begin{array}{c} f(x_1,x_2,x_3,x_4) \\ g(x_1,x_2,x_3,x_4) \\ h(x_1,x_2,x_3,x_4) \\ k(x_1,x_2,x_3,x_4)\end{array}\right ]
$$

where 
$$
\begin{align*}
a &= \frac{u+0.01sin(x_3)(0.25x_4^2+981cos(x_3)}{1+0.01sin^2(x_3)} \\\\
b &= \frac{-ucos(x_3)-0.01*0.25*x_4^2cos(x_3)sin(x_3)-(0.01+1)9.81sin(x_3)}{0.25(1+0.01sin^2(x_3))}
\end{align*} 
$$

where $x_1=x,\,x_2=\dot{x},\,x_3=\theta,\,x_4=\dot{\theta}$ and the coefficients were found by inputting the values for $m_c$, $m_p$, and $l$, and setting f = u. $\frac{dx}{dt}$ is then used in step 5 of the DDP Algorithm\with the relation $x(k+1)=x(k)+\frac{dx}{dt}(k)dt$. In order to find $\triangledown_xF$ and $\triangledown_uF$, the Hessian of the dynamics matrix above must be found by taking the partial derivatives with respect to x and u. 

##### Results 

The results for the cart-pole system are provided in the figure below. Here the aim was to have an idle cart-pole rotate the pole by 180deg, and maintain zero final velocities. 

 <img src="./cartPole/results.png" width="600">
