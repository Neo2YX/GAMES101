# �޵������ת��ʽ(Rodrigues)
��֪��
k - ��ת�ᣬ��λ����
v - ����ת����
$\theta$ - ��ת�Ƕ�

### ���ڴ���ת��������ת���໥��ֱ�����

$v_{rot} = cos\theta \cdot v + sin\theta (k\times v)$

### ���ڴ���ת��������ת�᲻��ֱ�����

$v_{rot} = v_\parallel + v_{\perp rot}$

$v_\parallel = v\cdot k \cdot k$

$v_\perp = v - v_\parallel = v - v\cdot k \cdot k$

$v_{\perp rot} = cos\theta \cdot v_\perp + sin\theta (k\times v_\perp)$

$v_{rot} = cos\theta \cdot v + (1-cos\theta)\cdot v\cdot k\cdot k+sin\theta(k\times v)$
ע������ֱʱ��$v\cdot k=0$

### �����ʾ
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