# Introduction
Differential flatness based method

# Kinematics Model
Refer to [Kinematics_model.md](../Kinematics_model.md)

# Differential Flatness

$$
\begin{bmatrix}
\dot{x}  \\
\dot{y}  \\
\dot{\theta} \\
\end{bmatrix}
=
\begin{bmatrix}
    cos\theta & 0  \\
    sin\theta & 0  \\
    0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
    v  \\
    w
\end{bmatrix}
$$

considering flat outputs $s_1=x$, $s_2=y$, we have
$$
\dot{h} = 
\begin{bmatrix}
    \dot{s}_1  \\
    \dot{s}_2  \\
\end{bmatrix}
 = 
\begin{bmatrix}
    cos\theta & 0  \\
    sin\theta & 0  \\
\end{bmatrix}
\begin{bmatrix}
    v  \\
    w
\end{bmatrix}
=
A_1
\begin{bmatrix}
    v  \\
    w
\end{bmatrix}
$$
where $rank(A_1) = 1$ $<$ $2 = dim(h)$. Inverse matrix of $A_1$ does not exist. So, for $\ddot{h}$, we have

$$
\ddot{h}
=
\begin{bmatrix}
    cos\theta & -v\sin\theta  \\
    sin\theta & v\cos\theta  \\
\end{bmatrix}
\begin{bmatrix}
    a_v  \\
    w
\end{bmatrix}
=
A_2
\begin{bmatrix}
    a_v  \\
    w
\end{bmatrix}
$$
where $a_v = \dot{v}$.

## controller

- Assumption 1: $v = \xi \neq 0$.
  
Based on the inverse matrix of $A_2$ and the flat outputs $s_1, s_2$, we have
$$
x=s_1\\
y=s_2\\
\theta=arctan2(\dot{s}_1,\dot{s}_2)\\
v = \sqrt{\dot{s}_1^2+\dot{s}_2^2}\\
\dot{v} = a_v = \frac{\dot{s}_1\ddot{s}_1+\dot{s}_2\ddot{s}_2}{\sqrt{\dot{s}_1^2+\dot{s}_2^2}}\\
\dot{\theta} = w =  \frac{\ddot{s}_1\dot{s}_2-\dot{s}_1\ddot{s}_2}{\dot{s}_1^2+\dot{s}_2^2}\\
$$

# Notes

- Second-order system with Assumption 1
$$
\begin{bmatrix}
\dot{x}  \\
\dot{y}  \\
\end{bmatrix}
=
\begin{bmatrix}
   1 & 0  \\
    0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
    \dot{s}_1  \\
    \dot{s}_2  \\
\end{bmatrix}
$$
$$
\begin{bmatrix}
    \ddot{s}_1  \\
    \ddot{s}_2  \\
\end{bmatrix}
=
\begin{bmatrix}
   1 & 0  \\
    0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
    u_1  \\
    u_2  \\
\end{bmatrix}
$$
$$
\begin{bmatrix}
    u_1  \\
    u_2  \\
\end{bmatrix}
=
\begin{bmatrix}
    cos\theta & -v\sin\theta  \\
    sin\theta & v\cos\theta  \\
\end{bmatrix}
\begin{bmatrix}
    a_v  \\
    w  \\
\end{bmatrix}
$$

- One of the open-loop controllers.