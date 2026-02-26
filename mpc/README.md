# Introduction
Model predictive contol method

    叠甲:
    1. 本项重点在于理解和使用MPC,不涉及调参
    2. 使用前向一阶模型进行模型的迭代,更为合理的选择是RK4
    3. 本项目并没有处理太多的约束条件
    4. 面向控制的项目而不是规划,想问规划的移步
    5. 实际物理系统不一样,理解后修改使用,不要直接套
    6. 本人比较懒，不想回答基础原理问题


# Kinematics Model
Refer to [Kinematics_model.md](../Kinematics_model.md)

kinematic model
$$
\begin{bmatrix}
\dot{x}  \\
\dot{y}  \\
\dot{\theta} \\
\end{bmatrix}
=
\begin{bmatrix}
    \cos\theta & 0  \\
    \sin\theta & 0  \\
    0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
    v  \\
    w
\end{bmatrix}
$$

Reference system
$$
\begin{bmatrix}
\dot{x}_r  \\
\dot{y}_r  \\
\dot{\theta}_r \\
\end{bmatrix}
=
\begin{bmatrix}
    \cos\theta_r & 0  \\
    \sin\theta_r & 0  \\
    0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
    v_r  \\
    w_r
\end{bmatrix}
$$

# MPC for Nonlinear System

Consider the following nonlinear system 
$$
\begin{bmatrix}
    x_{k+1} - x_k  \\
    y_{k+1} - y_k  \\
    \theta_{k+1} - \theta_k  \\
\end{bmatrix}
=
\begin{bmatrix}
    \dot{x}  \\
    \dot{y}  \\
    \dot{\theta} \\
\end{bmatrix}
=
\begin{bmatrix}
    \cos\theta & 0  \\
    \sin\theta & 0  \\
    0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
    v  \\
    w
\end{bmatrix}
=d_t
\begin{bmatrix}
    \cos\theta_k & 0  \\
    \sin\theta_k & 0  \\
    0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
    v_k  \\
    w_k
\end{bmatrix}
$$

we can find that
$$
\begin{bmatrix}
    x_{k+1}   \\
    y_{k+1} \\
    \theta_{k+1} \\
\end{bmatrix}
=
\begin{bmatrix}
    1 & 0 & 0 \\
    0 & 1 & 0 \\
    0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
    x_k  \\
    y_k  \\
    \theta_k  \\
\end{bmatrix}
+ 
\begin{bmatrix}
    d_t \cdot \cos\theta_k & 0  \\
    d_t \cdot \sin\theta_k & 0  \\
    0 & d_t \cdot 1 \\
\end{bmatrix}
\begin{bmatrix}
    v_k  \\
    w_k
\end{bmatrix}
=A_k \bar{x}_k + B_k \bar{u}_k
$$

Set $\xi_k = \begin{bmatrix}
    \bar{x}_k  \\
    \bar{u}_{k-1}  \\
\end{bmatrix}$ we can write the system as
$$
\xi_{k+1} = \begin{bmatrix}
    \bar{x}_{k+1}  \\
    \bar{u}_{k}  \\
\end{bmatrix} =
\begin{bmatrix}
    A_k \bar{x}_k + B_k \bar{u}_{k-1} + B_k \bar{u}_k  - B_k \bar{u}_{k-1} \\
    \bar{u}_{k-1} + \bar{u}_{k} - \bar{u}_{k-1} \\
\end{bmatrix}\\
=
\begin{bmatrix}
    A_k  & B_k \\
    0 & I\\
\end{bmatrix}
\begin{bmatrix}
    \bar{x}_k \\ 
    \bar{u}_{k-1}
\end{bmatrix}
+
\begin{bmatrix}
    {B}_k \\ 
    I
\end{bmatrix}
\begin{bmatrix}
    \bar{u}_k - \bar{u}_{k-1}
\end{bmatrix}\\
=\bar{A}_{k} \xi_k + \bar{B}_{k} \Delta \bar{u}_k
$$
where $\Delta \bar{u}_k = \bar{u}_k - \bar{u}_{k-1}$. When we consider the outputs
$$
Y_k =
\begin{bmatrix} 
    I& 0
\end{bmatrix}
\begin{bmatrix} 
    \bar{x}_k  \\
    \bar{u}_{k-1}  \\
\end{bmatrix}
={C} \xi_k
$$
then find the iterative output
$$
Y_{k+1} = {C} \xi_{k+1} = C (\bar{A}_{k} \xi_k + \bar{B}_{k} \Delta \bar{u}_k)\\
Y_{k+2} = {C} \xi_{k+2} = C (\bar{A}_{k+1} \xi_{k+1} + \bar{B}_{k+1} \Delta \bar{u}_{k+1})  = C (\bar{A}_{k+1} (\bar{A}_{k} \xi_k + \bar{B}_{k} \Delta \bar{u}_k) + \bar{B}_{k+1} \Delta \bar{u}_{k+1}) \\
=C(\bar{A}_{k+1} \bar{A}_{k} \xi_k + \bar{A}_{k+1} \bar{B}_{k} \Delta \bar{u}_k + \bar{B}_{k+1} \Delta \bar{u}_{k+1}) \\ 
Y_{k+3} = {C} \xi_{k+3} =C(\bar{A}_{k+2} \bar{A}_{k+1} \bar{A}_{k} \xi_k + \bar{A}_{k+2} \bar{A}_{k+1} \bar{B}_{k} \Delta \bar{u}_k + \bar{A}_{k+2} \bar{B}_{k+1} \Delta \bar{u}_{k+1} + \bar{B}_{k+2} \Delta \bar{u}_{k+2})\\
\vdots \\
$$

<font color="red">Set $\bar{Y}_{k+1} = [Y_{k+1}, Y_{k+2}, \cdots, Y_{k+N}]^T$, we can obtain the composed format</font>
$$
\bar{Y}_{k+1}=W_k \xi_{k} + Z_k \Delta {u}_k
$$
where
$$
W_k = \begin{bmatrix}
    C \bar{A}_{k} \\
    C \bar{A}_{k+1} \bar{A}_{k} \\
    \vdots \\
    C \bar{A}_{k+N-1} \bar{A}_{k+N-2} \cdots \bar{A}_{k} \\
\end{bmatrix} \\
Z_k = \begin{bmatrix}
    C \bar{B}_{k} & 0 & \cdots & 0 \\
    C \bar{A}_{k+1} \bar{B}_{k} & C \bar{B}_{k+1}   & \cdots & 0 \\
    \vdots & \vdots & \ddots  & 0 \\
    C \bar{A}_{k+N-1} \bar{A}_{k+N-2} \cdots \bar{A}_{k+1} \bar{B}_{k} 
    & C \bar{A}_{k+N-1} \bar{A}_{k+N-2} \cdots \bar{A}_{k+2} \bar{B}_{k+1}
    & \cdots & C \bar{B}_{k+N-1} \\
\end{bmatrix}\\
\Delta {u}_k
=
\begin{bmatrix}
    \Delta \bar{u}_k  \\
    \Delta \bar{u}_{k+1}  \\
    \vdots \\
    \Delta \bar{u}_{k+N-1}  \\
\end{bmatrix}
$$

Choose the following cost function
$$
\begin{align}
\begin{matrix}
J &=& \sum_{i=1}^{N} ||{Y}_{k+i} - {Y}_{k+i}^{r}||^2_Q + \sum_{i=0}^{N-1} ||\Delta \bar{u}_{k+i}||^2_R \\ 
\end{matrix}
\end{align}
$$

  (option): 约束条件
  - $\Delta \bar{u}_{k+i} \in [\Delta u_{min}, \Delta u_{max}]$, $i=0,1,\cdots,N-1$
  - $\bar{u}_{k+i} \in [\bar{u}_{min}, \bar{u}_{max}]$, $i=0,1,\cdots,N-1$
  - 转向时的左右轮差速条件
  
    <font color="red">note: 暂时不考虑处理这些约束条件。</font>

Set
$$
{Y}_{k}^{e} =
\begin{bmatrix}
    {Y}_{k+1} - {Y}_{k+1}^{r} \\
    {Y}_{k+2} - {Y}_{k+2}^{r} \\
    \vdots \\
    {Y}_{k+N} - {Y}_{k+N}^{r} \\
\end{bmatrix}
=\bar{Y}_{k} -\bar{Y}_{k}^{r},\\
\bar{Y}_{k}^{r} = [Y_{k+1}^{r}, Y_{k+2}^{r}, \cdots, Y_{k+N}^{r}]^T
$$

then the cost function can be rewritten as
$$
\begin{matrix}
J &=& {{Y}_{k}^{e}}^T Q {Y}_{k}^{e} + {{\Delta {u}_{k}}}^T R {{ \Delta {u}_{k}}}\\
&=&(W_k \xi_{k} + Z_k \Delta {u}_k - \bar{Y}_{k}^{r} )^T Q(W_k \xi_{k} + Z_k \Delta {u}_k - \bar{Y}_{k}^{r} ) + {{\Delta {u}_{k}}}^T R {{ \Delta {u}_{k}}}\\
\end{matrix}
$$

Set
$$
G_k = W_k \xi_{k}  - \bar{Y}_{k}^{r} 
$$

then the cost function can be rewritten as
$$
\begin{matrix}
J &=&(G_k + Z_k \Delta {u}_k)^T Q(G_k + Z_k \Delta {u}_k) + {{\Delta {u}_{k}}}^T R {{ \Delta {u}_{k}}}\\
&=& G_k^T Q G_k + 2G_k^T Q Z_k \Delta {u}_k + {{\Delta {u}_{k}}}^T (Z_k^T Q Z_k +R) {{ \Delta {u}_{k}}}\\
\end{matrix}
$$
where $G_k^T Q G_k$ is a constant term, $2G_k^T Q Z_k$ is the gradient term, and $Z_k^T Q Z_k +R$ is the hessian term.

## Note: Nonlinear system

- 这部分关于非线性系统模型的选择并不是唯一的，你可以根据自己的需求选择不同的模型。介绍和代码中基于当前和上次控制量推导的非线性系统模型目前看来并没有那么合理。

- 个人建议使用下方的非线性系统模型
  $$
    \begin{bmatrix}
        x_{k+1}   \\
        y_{k+1} \\
        \theta_{k+1} \\
    \end{bmatrix}
    =
    \begin{bmatrix}
        1 & 0 & 0 \\
        0 & 1 & 0 \\
        0 & 0 & 1 \\
    \end{bmatrix}
    \begin{bmatrix}
        x_k  \\
        y_k  \\
        \theta_k  \\
    \end{bmatrix}
    + 
    \begin{bmatrix}
        d_t \cdot \cos\theta_k & 0  \\
        d_t \cdot \sin\theta_k & 0  \\
        0 & d_t \cdot 1 \\
    \end{bmatrix}
    \begin{bmatrix}
        v_k  \\
        w_k
    \end{bmatrix}
    =A_k \bar{x}_k + B_k \bar{u}_k
    $$
    代入<font color="red">MPC for Linear System</font>的代码, 这样较为合理.

- 代码中这种增量形式的非线性系统模型在仿真的时候相当依赖调参.

## Code
Check `./nmpcCasAdi` for the code.

# MPC for Linear System

Consider the linearized system at the equilibrium point
$$
\begin{bmatrix}
    x_{k+1} - x_k  \\
    y_{k+1} - y_k  \\
    \theta_{k+1} - \theta_k  \\
\end{bmatrix}
=
\begin{bmatrix}
    \dot{x}  \\
    \dot{y}  \\
    \dot{\theta} \\
\end{bmatrix}
=
\begin{bmatrix}
    v_r\cos\theta_r  \\
    v_r\sin\theta_r  \\
    w_r  \\
\end{bmatrix}
+
\begin{bmatrix}
    0 & 0 & -v_r\sin\theta_r  \\
    0 & 0 & v_r\cos\theta_r  \\
    0 & 0 & 0  \\
\end{bmatrix}
\begin{bmatrix}
    x-x_r  \\
    y-y_r  \\
    \theta-\theta_r  \\
\end{bmatrix}
+
\begin{bmatrix}
    \cos\theta_r & 0  \\
    \sin\theta_r & 0  \\
    0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
    v-v_r  \\
    w-w_r
\end{bmatrix}\\
=
\begin{bmatrix}
    0 & 0 & -v_r\sin\theta_r  \\
    0 & 0 & v_r\cos\theta_r  \\
    0 & 0 & 0  \\
\end{bmatrix}
\begin{bmatrix}
    x-x_r  \\
    y-y_r  \\
    \theta-\theta_r  \\
\end{bmatrix}
+
\begin{bmatrix}
    \cos\theta_r & 0  \\
    \sin\theta_r & 0  \\
    0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
    v \\
    w
\end{bmatrix}
$$
then the tracking error system is
$$
\begin{align}
\begin{bmatrix}
    x_{k+1} - x_r  \\
    y_{k+1} - y_r  \\
    \theta_{k+1} - \theta_r  \\
\end{bmatrix}
=
\begin{bmatrix}
    1 & 0 & -v_r\sin\theta_r \cdot d_t  \\
    0 & 1 & v_r\cos\theta_r \cdot d_t  \\
    0 & 0 & 1  \\
\end{bmatrix}
\begin{bmatrix}
    x_k-x_r  \\
    y_k-y_r  \\
    \theta_k-\theta_r  \\
\end{bmatrix}
+
\begin{bmatrix}
    \cos\theta_r  \cdot d_t & 0  \\
    \sin\theta_r  \cdot d_t & 0  \\
    0 & 1 \cdot d_t \\
\end{bmatrix}
\begin{bmatrix}
    v_k \\
    w_k \\
\end{bmatrix}
\end{align}
$$
where $x_r$, $y_r$, and $\theta_r$ are the reference states at time frame $k$,
$v_r$ and $w_r$ are the reference controls at time frame $k$,
$x_k$, $y_k$, and $\theta_k$ are the current states at time frame $k$,
$v_k$ and $w_k$ are the controls to be solved at time frame $k$.

Set 
$$
X_k =
\begin{bmatrix}
    x_{k} - x_r  \\
    y_{k} - y_r  \\
    \theta_{k} - \theta_r  \\
\end{bmatrix}
$$
we can rewrite the tracking error system as
$$
X_{k+1} = A_k X_k + B_k u_k
$$
then we have
$$
\begin{matrix}
X_{k+2}&=&A_{k+1} X_{k+1} + B_{k+1} u_{k+1}\\
   &=&A_{k+1} (A_k X_k + B_k u_k) + B_{k+1} u_{k+1}\\
    &=&A_{k+1}A_k X_k + A_{k+1} B_k u_k + B_{k+1} u_{k+1}
\end{matrix}\\
\begin{matrix}
X_{k+3}&=&A_{k+2} X_{k+2} + B_{k+2} u_{k+2}\\
   &=&A_{k+2} (A_{k+1}A_k X_k + A_{k+1} B_k u_k + B_{k+1} u_{k+1}) + B_{k+2} u_{k+2}\\
    &=&A_{k+2}A_{k+1}A_k X_k + A_{k+2}A_{k+1} B_k u_k + A_{k+2} B_{k+1} u_{k+1} + B_{k+2} u_{k+2}
\end{matrix}\\
\vdots\\
\begin{matrix}
X_{k+N}&=&A_{k+N-1} A_{k+N-2} \cdots  A_{k} X_{k} + \\
&&A_{k+N-1} A_{k+N-2} \cdots  A_{k+1} B_{k} u_{k} + \\
&&A_{k+N-1} A_{k+N-2} \cdots  A_{k+2} B_{k+1} u_{k+1} + \\
&& \vdots\\
&&B_{k+N-1} u_{k+N-1}\\
\end{matrix}\\
$$

<font color="red">Set $\bar{X}_{k+1} = [X_{k+1}, X_{k+2}, \cdots, X_{k+N}]^T$, we can obtain the composed format</font>
$$
\begin{align}
\bar{X}_{k+1} =
\begin{bmatrix}
    X_{k+1} \\
    X_{k+2} \\
    \vdots \\
    X_{k+N} \\
\end{bmatrix}
=
\bar{A}_{k} {X}_{k} + \bar{B}_{k} \bar{u}_{k}
\end{align}
$$
where $\bar{A}_{k} \subset \mathbb{R}^{3N \times 3}$, $\bar{B}_{k} \subset \mathbb{R}^{3N \times 2N}$ and $\bar{u}_{k} \subset \mathbb{R}^{2N}$. Specifically,
$$
\bar{A}_{k} =
\begin{bmatrix}
    {A}_{k} \\
    A_{k+1}A_k \\
    A_{k+2}A_{k+1}A_k\\
    \vdots \\
    A_{k+N-1} A_{k+N-2} \cdots  A_{k} \\
\end{bmatrix}
$$ 
$$
\bar{B}_{k} =
\begin{bmatrix}
    B_k  & 0 & \cdots & 0 \\
    A_{k+1} B_k   & B_{k+1} & \cdots & 0 \\
    \vdots  & \vdots & \vdots & \vdots \\
    A_{k+N-1} A_{k+N-2} \cdots  A_{k+1} B_{k}  & A_{k+N-1} A_{k+N-2} \cdots  A_{k+2} B_{k+1} & \cdots & B_{k+N-1} \\
\end{bmatrix}
$$ 
$$
\bar{u}_{k} =
\begin{bmatrix}
    u_{k} \\
    u_{k+1} \\
    \vdots \\
    u_{k+N-1} \\
\end{bmatrix}
% % jevon:better choose the below eq.
% \\\bar{u}_{k} =
% \begin{bmatrix}
%     u_{k}-u^r_{k} \\
%     u_{k+1}-u^r_{k+1} \\
%     \vdots \\
%     u_{k+N-1}-u^r_{k+N-1} \\
% \end{bmatrix}
$$ 

Choose a quadratic cost function
$$
\begin{align}
J &= \frac{1}{2} \bar{X}_{k+1}^T Q \bar{X}_{k+1} + \frac{1}{2} \bar{u}_{k}^T R \bar{u}_{k} \\
\end{align}
$$
where $\bar{X}_{k+1} \subset \mathbb{R}^{3N \times 1}$, ${X}_{k} \subset \mathbb{R}^{3 \times 1}$, $\bar{u}_{k} \subset \mathbb{R}^{2N \times 1}$, $Q \subset \mathbb{R}^{3N \times 3N}$ and $R \subset \mathbb{R}^{2N \times 2N}$.

We have
$$
\begin{matrix}
J&=&\frac{1}{2} \bar{X}_{k+1}^T Q \bar{X}_{k+1} + \frac{1}{2} \bar{u}_{k}^T R \bar{u}_{k} \\
&=&\frac{1}{2} ((\bar{A}_{k} {X}_{k} + \bar{B}_{k} \bar{u}_{k})^T Q (\bar{A}_{k} {X}_{k} + \bar{B}_{k} \bar{u}_{k}) + \bar{u}_{k}^T R \bar{u}_{k}) \\
   &=&\frac{1}{2} ({X}_{k}^T \bar{A}_{k}^T Q \bar{A}_{k} {X}_{k} + 2 {X}_{k}^T \bar{A}_{k}^T Q \bar{B}_{k} \bar{u}_{k} + \bar{u}_{k}^T \bar{B}_{k}^T Q \bar{B}_{k} \bar{u}_{k} + \bar{u}_{k}^T R \bar{u}_{k} ) \\
   &=&\frac{1}{2} ({X}_{k}^T \bar{A}_{k}^T Q \bar{A}_{k} {X}_{k} + 2 {X}_{k}^T \bar{A}_{k}^T Q \bar{B}_{k} \bar{u}_{k} + \bar{u}_{k}^T (\bar{B}_{k}^T Q \bar{B}_{k} + R )\bar{u}_{k}  ) \\
\end{matrix}\\
$$
where ${X}_{k}^T \bar{A}_{k}^T Q \bar{A}_{k} {X}_{k}$ is the constant term in the cost function, 
${X}_{k}^T \bar{A}_{k}^T Q \bar{B}_{k} \bar{u}_{k}$ is the linear term in the cost function,
and $\bar{u}_{k}^T (\bar{B}_{k}^T Q \bar{B}_{k} + R )\bar{u}_{k}$ is the quadratic term in the cost function.

## Note: Linear system

- 这部分关于系统模型的推导可能存在问题，如果你需要100%模型正确，建议你自行重新推导，这边的重心在于如何推导和实现mpc控制器。

- If we choose the cost function as bellow
    $$
    \begin{align}
    J &= \frac{1}{2} \bar{X}_{k+1}^T Q \bar{X}_{k+1} + \frac{1}{2} (\bar{u}_{k}^T - \bar{u}^r_{k})^T R (\bar{u}_{k} - \bar{u}^r_{k}) \\
    \end{align}
    $$
    where 
    $$
    \bar{u}^r_{k} =
    \begin{bmatrix}
        u_{k}^r \\
        u_{k+1}^r \\
        \vdots \\
        u_{k+N-1}^r \\
    \end{bmatrix}
    $$
    then  we can get
    $$
    \begin{matrix}
    J&=&\frac{1}{2} \bar{X}_{k+1}^T Q \bar{X}_{k+1} + \frac{1}{2} (\bar{u}_{k}^T - \bar{u}^r_{k})^T R (\bar{u}_{k} - \bar{u}^r_{k}) \\
    &=&\frac{1}{2} ({X}_{k}^T \bar{A}_{k}^T Q \bar{A}_{k} {X}_{k} + {\bar{u}^r_{k}}^T R {\bar{u}^r_{k}} + 2 ( {X}_{k}^T \bar{A}_{k}^T Q \bar{B}_{k} - {\bar{u}^r_{k}}^T R )\bar{u}_{k} + \bar{u}_{k}^T (\bar{B}_{k}^T Q \bar{B}_{k} + R )\bar{u}_{k}  ) \\
    \end{matrix}\\
    $$

## Code
Check `./ipopt` or `./OSQP` for the code.

# Reference
[Ipopt](https://coin-or.github.io/Ipopt/index.html)

[CasADi](https://web.casadi.org/)

[OSQP-Eigen](https://github.com/gbionics/osqp-eigen)

[Bilibili DR_CAN](https://space.bilibili.com/230105574)

[CMU 16-745 2025](https://optimalcontrol.ri.cmu.edu/)

[Matlab Differential Drive MPPI Controller](https://ww2.mathworks.cn/help/robotics/ref/differentialdrivemppicontroller.html)

[Matlab Bicycle MPPI Controller](https://ww2.mathworks.cn/help/robotics/ref/bicyclemppicontroller.html)
