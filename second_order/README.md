# Introduction
Polynomial based linearization method

# Kinematics Model
Refer to [Kinematics_model.md](../Kinematics_model.md)

# Linearization

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

considering outputs $h=[x,y]^T$, we have
$$
\dot{h} = 
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
where $rank(A_1) = 1$ $<$ $2 = dim(h)$, so we choose a auxiliary signal $\xi = v$. For $\ddot{h}$, we have

$$
\ddot{h}
=
\begin{bmatrix}
    cos\theta & -\xi\sin\theta  \\
    sin\theta & \xi\cos\theta  \\
\end{bmatrix}
\begin{bmatrix}
    \dot\xi  \\
    w
\end{bmatrix}
=
A_2
\begin{bmatrix}
    \dot\xi  \\
    w
\end{bmatrix}
$$

- Assumption 1: $v = \xi \neq 0$.

then for $\ddot{h} = [r_1,r_2]^T$, we can write
$$
r_1 = \ddot{x}_r + k_x^v(\dot{x}_r-\dot{x}) + k_x^p(x_r-x) \\
r_2 = \ddot{y}_r + k_y^v(\dot{y}_r-\dot{y}) + k_y^p(y_r-y)
$$
where we can make the equations above qualify the **Hurwitz conditions**, which means $\lim_{t\to\infty} (x_r-x) = 0$ and $\lim_{t\to\infty} (y_r-y) = 0$.

## controller A

- Assumption 2: $\theta \neq \frac{\pi}{2} \pm k \pi$, where $k$ is a integer.

we can design the controller directly as follows
$$
v = \xi \\
w = \frac{-r_1\sin\theta+r_2\cos\theta}{\xi}\\
\dot{\xi} = \frac{r_1+\sin\theta(r_2\cos\theta-r_1\sin\theta)}{\cos\theta}
$$

## controller B

Based on the inverse matrix of $A_2$, we can design the controller as follows
$$
\dot{\xi} = r_1\cos\theta+r_2\sin\theta \\
w = \frac{-r_1\sin\theta}{\xi}+\frac{r_2\cos\theta}{\xi}\\
v = \xi \\
$$