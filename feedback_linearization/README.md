# Introduction
Feedback linearization method

# Kinematics Model
Refer to [Kinematics_model.md](../Kinematics_model.md)

# Linearization

Different from the [second_order linearization method](../second_order/README.md), we only consider the $\dot{h}$ rather than $\ddot{h}$.
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

## controller A
(not recommend)
considering outputs $h=[x,\theta]^T$(<font color="red">$h=[y,\theta]$ is similar</font>), we have
$$
\dot{h} = 
\begin{bmatrix}
    cos\theta & 0  \\
    0 & 1  \\
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
- Assumption 1: $\theta \neq \frac{\pi}{2} \pm k \pi$, where $k$ is a integer.

Set $\dot{h} = [r_1,r_2]^T$ and
$$
r_1 = \dot{x}_r + k_x^p(x_r-x) \\
r_2 = \dot{\theta}_r + k_{\theta}^p(\theta_r-\theta)
$$
where we can make the equations above qualify the **Hurwitz conditions**, which means $\lim_{t\to\infty} (x_r-x) = 0$ and $\lim_{t\to\infty} (\theta_r-\theta) = 0$.


$$
\begin{bmatrix}
    v  \\
    w
\end{bmatrix}
=A_1^{-1}\dot{h}
=
\begin{bmatrix}
    \frac{1}{cos\theta} & 0  \\
    0 & 1  \\
\end{bmatrix}
\begin{bmatrix}
    r_1  \\
    r_2
\end{bmatrix}
=
\begin{bmatrix}
    \frac{r_1}{cos\theta}  \\
    r_2
\end{bmatrix}
$$


## controller B

Pseudo inverse matrix method.

Consider outputs $h=[x,y,\theta]^T$, we have
$$
\dot{h} = 
\begin{bmatrix}
    cos\theta & 0  \\
    sin\theta & 0  \\
    0 & 1 \\
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

瞪眼法可知其广义逆矩阵为
$$
A_1^{-1} =
\begin{bmatrix}
    \cos\theta &\sin\theta & 0  \\
    0 & 0 & 1  \\
\end{bmatrix}  
$$

When we choose $\dot{h} = [r_1,r_2, r_3]^T$ as follows
$$
\dot{h} = 
\begin{bmatrix}
    r_1  \\
    r_2  \\
    r_3
\end{bmatrix}
=
\begin{bmatrix}
\dot{x}_r + k_x^p(x_r-x) \\
\dot{y}_r + k_y^p(y_r-y) \\
\dot{\theta}_r + k_{\theta}^p(\theta_r-\theta) \\
\end{bmatrix}
$$
we have
$$
\begin{bmatrix}
    v  \\
    w
\end{bmatrix}
=A_1^{-1}\dot{h}
=
\begin{bmatrix}
    \cos\theta &\sin\theta & 0  \\
    0 & 0 & 1  \\
\end{bmatrix}
\begin{bmatrix}
    r_1  \\
    r_2  \\
    r_3
\end{bmatrix}
=
\begin{bmatrix}
    r_1{\cos\theta} + r_2\sin\theta  \\
    r_3
\end{bmatrix}
$$