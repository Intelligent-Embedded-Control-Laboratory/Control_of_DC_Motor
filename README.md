# Control_of_DC_Motor

## Dynamic Equations
### Linear Model

\begin{equation}
J\ddot{\theta} + \alpha \dot{\theta} + F_{c}\sign(\dot{\theta}) = K u
\end{equation}

\begin{equation}
    {E}({Q}) = \begin{bmatrix}
        -{q}^T\\
        q_0{I}_3+{q}^\times
    \end{bmatrix}
    =
    \begin{bmatrix}
        -q_{1} & -q_{2} & -q_{3}\\
        q_{0}  & -q_{3} & q_{2}\\
        q_{3}  & q_{0} & -q_{1}\\
        -q_{2} & q_{1} & q_{0}
    \end{bmatrix}
\end{equation}
