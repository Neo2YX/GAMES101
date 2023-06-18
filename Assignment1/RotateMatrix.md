# 罗德里格旋转公式(Rodrigues)
已知：
k - 旋转轴，单位向量
v - 待旋转向量
$\theta$ - 旋转角度

### 对于待旋转向量与旋转轴相互垂直的情况

$v_{rot} = cos\theta \cdot v + sin\theta (k\times v)$

### 对于待旋转向量与旋转轴不垂直的情况

$v_{rot} = v_\parallel + v_{\perp rot}$

$v_\parallel = v\cdot k \cdot k$

$v_\perp = v - v_\parallel = v - v\cdot k \cdot k$

$v_{\perp rot} = cos\theta \cdot v_\perp + sin\theta (k\times v_\perp)$

$v_{rot} = cos\theta \cdot v + (1-cos\theta)\cdot v\cdot k\cdot k+sin\theta(k\times v)$
注：当垂直时，$v\cdot k=0$

### 矩阵表示
$v_{rot} = R\cdot v$

$v\cdot k\cdot k = k(k^T\cdot v) = (kk^T)v$

$k \times v = \begin{pmatrix}
                k_2v_3 - k_3v_2\\
                k_3v_1 - k_1v_3\\
                k_1v_2 - k_2v_1
                \end{pmatrix}
             = \begin{pmatrix}
                0 & -k_3 & k_2\\
                k_3 & 0 & -k_1\\
                -k_2 & k_1 & 0
                \end{pmatrix}
                \begin{pmatrix}
                v_1\\
                v_2\\
                v_3
                \end{pmatrix}$

$v_{rot} = (cos\theta + (1-cos\theta)(kk^T)+sin\theta \begin{pmatrix}
                0 & -k_3 & k_2\\
                k_3 & 0 & -k_1\\
                -k_2 & k_1 & 0
                \end{pmatrix})*v$

$R = cos\theta + (1-cos\theta)(kk^T)+sin\theta \begin{pmatrix}
                0 & -k_3 & k_2\\
                k_3 & 0 & -k_1\\
                -k_2 & k_1 & 0
                \end{pmatrix}\\\space
   = cos\theta I_{3\times 3}+(1-cos\theta)\begin{pmatrix} k_1\\k_2\\k_3\end{pmatrix}\begin{pmatrix} k_1&k_2&k_3\end{pmatrix}+sin\theta\begin{pmatrix}
                0 & -k_3 & k_2\\
                k_3 & 0 & -k_1\\
                -k_2 & k_1 & 0
                \end{pmatrix}$