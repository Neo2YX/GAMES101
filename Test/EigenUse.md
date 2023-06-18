# LEARN EIGEN

[TOC]

## Matrix Class
所有的Matrixs和Vectors都是由Matrix Class来的，其形式如下：
`Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>`
`typedef Matrix<float, 4, 4> Matrix4f`

也可以通过`Dynamic`定义编译时未知的矩阵大小
`typedef Matrix<double, Dynamic, Dynamic> MatrixXd`
`typedef Matrix<int, Dynamic, 1> VectorXi`

#### 初始化
行向量或者列向量可以用大括号列表初始化参数
`Matrix<int, 5, 1> a {1, 2, 3, 4, 5};`
`Matrix<int, 1, 5> b = {1, 2, 3, 4, 5};`
对于矩阵的初始化需要用大括号将每一行包起来
```
MatrixXi a  { //construct a 2x2 matrix
    {1, 2},
    {3, 4}
};
Matrix<double, 2, 3> b {
    {2, 3, 4},
    {5, 6, 7}
};
//dynamic vector use this way
VectorXd a {{1.5, 2.5, 3.5}}; //col-vector with 3 coefficients
RowVectorXd b {{1.0, 2.0, 3.0, 4.0}}; //row-vector with 4 coefficients
```

数据存取，矩阵先列后行，向量只需要一个参数
```
int main()
{
  Eigen::MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << "Here is the matrix m:\n" << m << std::endl;
  Eigen::VectorXd v(2);
  v(0) = 4;
  v(1) = v(0) - 1;
  std::cout << "Here is the vector v:\n" << v << std::endl;
}

/*************** OUTPUT **************/
Here is the matrix m:
  3  -1
2.5 1.5
Here is the vector v:
4
3
```
**对于矩阵来说`m(a)`，可以按照列来读取数据（Eigen默认为column-major）**
**对于向量来说`v[a]`同样可以使用，与`v(a)`效果相同**

也可以用`<<`进行初始化

#### resize
可以使用`cols() rows() size()`分别得到列、行与元素的个数
使用`resize()`可改变动态矩阵和向量的大小，**不可以改变固定大小的矩阵和向量**
**注：`resize()`会改变矩阵的元素值，若不想改变原有值，使用`conservativeResize()`**

#### fixed vs dynamic
当元素个数较少时，推荐使用fixed获得更好的性能，很多时或者有此需求则使用dynamic更好

#### 其他可选择的模板参数
```
Matrix<typename Scalar,
       int RowsAtCompileTime,
       int ColsAtCompileTime,
       int Option = 0,  //can set RowMajor
       int MaxRowsAtCompileTime = RowsAtCompileTime,
       int MaxColsAtCompileTime = ColsAtCompileTime>
```

## Matrix and arithmetic

支持的运算：
+ +、-、+=、-=
+ matrix\*Scalar、Scalar\*matrix、matrix/scalar、*=、/=
+ 转置`transpose()`、共轭`conjugate()`、伴随`adjoint()`
  在实数矩阵中`conjugate()`无操作，伴随矩阵等于转置矩阵
  **注：不要用`a = a.transpose()`，会在未转置结束时就在a中写数据，导致错误，应使用`a.transposeInPlace()`**
+ matrix\*matrix、matrix\*vector、vector\*matrix、vector\*vector
+ vector dot product`v.dot(w)` cross product`v.cross(w)`
  cross product只支持三维向量的叉乘，dot product则无此限制
+ 矩阵内元素操作：
  `sum()` - 元素之和
  `prod()` - 元素之乘
  `mean()` - 元素平均
  `minCoeff()` - 元素最小值 传入std::ptrdiff_t&可以获得元素位置
  `maxCoeff()` - 元素最大值
  `trace()` - 矩阵对角线元素之和 也可以写成`a.diagonal().sum()`

## Array class and coefficient-wise operation
比起设计给线性代数的Matrix Class， Array Class更支持对系数相关的操作（并不具备线性代数的意义，比如给每一个系数加上一个常量或者将两个矩阵的系数相乘）

#### 初始化
模板初始化与Matrix Class基本相同
但是几个typedef定义有所不同，一个数字为1维数组，两个为2维：
```
Array<float, Dynamic, 1>          => ArrayXf
Array<float, 3, 1>                => Array3f
Array<double, Dynamic, Dynamic>   => ArrayXXd
Array<double, 3, 3>               => Array33d
```

#### 计算
支持Array + Scalar，将Array中所有元素增加Scalar
在Array * Array中，运算为将Array中的各个元素相乘，即必须在相同大小的Array才能相乘
其他还包括：`abs() sqrt() pow() min() floor() ceil() round()`

支持使用`a.matrix() m.array()`完成两者的相互转换

## Block operations
block就是矩阵中指定的某个方块，既可以当左值又可以当右值，其语法如下：
```
//Block of size(p, q), starting at (i, j)
matrix.block(i, j, p, q); //construct dynamic 
matrix.block<p,q>(i, j); //construct fixed
```

其中`col() row()`是一种特殊的Block

还有其他的操作：
```
natrix.topLeftCornor(p,q);
matrix.bottomLeftCornor(p,q);
matrix.leftCols(p);

//vector的Block
v.head(n);
v.tail(n);
v.segment(i, n); //n element starting at i
```

## Advanced initialization

#### comma initializer
`<<`进行初始化时，可以使用Vector或者matrix作为初始化的block，如：
```
MatrixXf matA(2, 2);
matA << 1, 2, 3, 4;
MatrixXf matB(4, 4);
matB << matA, matA/10, matA/10, matA;
std::cout << matB << std::endl;
//OUTPUT
 1   2 0.1 0.2
  3   4 0.3 0.4
0.1 0.2   1   2
0.3 0.4   3   4


Matrix3f m;
m.row(0) << 1, 2, 3;
m.block(1,0,2,2) << 4, 5, 7, 8;
m.col(2).tail(2) << 6, 9;                   
std::cout << m;
//OUTPUT
1 2 3
4 5 6
7 8 9
```

#### special matrices and arrays
`Zero()` - 若用来初始化动态矩阵，则需要加入参数
`Constant(rows, cols, value)` - 用来填充矩阵
`Random()` - 用随机数填充矩阵
`Identity()` - 只适用于Matrix
`LinSpace(rows, low, high)` - 只适用于一维数组或者向量，从low到high等距填充

同时还支持：
```
m.topLeftCornor(size/2,size/2).setZero();
m.topRightCornor(size/2,size/2).setIdentity();
m.bottomLeftCornor(size/2,size/2).setIdentity();
m.bottomRightCornor(size/2,size/2).setZero();
```
