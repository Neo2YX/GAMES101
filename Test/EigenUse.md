# LEARN EIGEN

[TOC]

## Matrix Class
���е�Matrixs��Vectors������Matrix Class���ģ�����ʽ���£�
`Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>`
`typedef Matrix<float, 4, 4> Matrix4f`

Ҳ����ͨ��`Dynamic`�������ʱδ֪�ľ����С
`typedef Matrix<double, Dynamic, Dynamic> MatrixXd`
`typedef Matrix<int, Dynamic, 1> VectorXi`

#### ��ʼ��
���������������������ô������б��ʼ������
`Matrix<int, 5, 1> a {1, 2, 3, 4, 5};`
`Matrix<int, 1, 5> b = {1, 2, 3, 4, 5};`
���ھ���ĳ�ʼ����Ҫ�ô����Ž�ÿһ�а�����
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

���ݴ�ȡ���������к��У�����ֻ��Ҫһ������
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
**���ھ�����˵`m(a)`�����԰���������ȡ���ݣ�EigenĬ��Ϊcolumn-major��**
**����������˵`v[a]`ͬ������ʹ�ã���`v(a)`Ч����ͬ**

Ҳ������`<<`���г�ʼ��

#### resize
����ʹ��`cols() rows() size()`�ֱ�õ��С�����Ԫ�صĸ���
ʹ��`resize()`�ɸı䶯̬����������Ĵ�С��**�����Ըı�̶���С�ľ��������**
**ע��`resize()`��ı�����Ԫ��ֵ��������ı�ԭ��ֵ��ʹ��`conservativeResize()`**

#### fixed vs dynamic
��Ԫ�ظ�������ʱ���Ƽ�ʹ��fixed��ø��õ����ܣ��ܶ�ʱ�����д�������ʹ��dynamic����

#### ������ѡ���ģ�����
```
Matrix<typename Scalar,
       int RowsAtCompileTime,
       int ColsAtCompileTime,
       int Option = 0,  //can set RowMajor
       int MaxRowsAtCompileTime = RowsAtCompileTime,
       int MaxColsAtCompileTime = ColsAtCompileTime>
```

## Matrix and arithmetic

֧�ֵ����㣺
+ +��-��+=��-=
+ matrix\*Scalar��Scalar\*matrix��matrix/scalar��*=��/=
+ ת��`transpose()`������`conjugate()`������`adjoint()`
  ��ʵ��������`conjugate()`�޲���������������ת�þ���
  **ע����Ҫ��`a = a.transpose()`������δת�ý���ʱ����a��д���ݣ����´���Ӧʹ��`a.transposeInPlace()`**
+ matrix\*matrix��matrix\*vector��vector\*matrix��vector\*vector
+ vector dot product`v.dot(w)` cross product`v.cross(w)`
  cross productֻ֧����ά�����Ĳ�ˣ�dot product���޴�����
+ ������Ԫ�ز�����
  `sum()` - Ԫ��֮��
  `prod()` - Ԫ��֮��
  `mean()` - Ԫ��ƽ��
  `minCoeff()` - Ԫ����Сֵ ����std::ptrdiff_t&���Ի��Ԫ��λ��
  `maxCoeff()` - Ԫ�����ֵ
  `trace()` - ����Խ���Ԫ��֮�� Ҳ����д��`a.diagonal().sum()`

## Array class and coefficient-wise operation
������Ƹ����Դ�����Matrix Class�� Array Class��֧�ֶ�ϵ����صĲ����������߱����Դ��������壬�����ÿһ��ϵ������һ���������߽����������ϵ����ˣ�

#### ��ʼ��
ģ���ʼ����Matrix Class������ͬ
���Ǽ���typedef����������ͬ��һ������Ϊ1ά���飬����Ϊ2ά��
```
Array<float, Dynamic, 1>          => ArrayXf
Array<float, 3, 1>                => Array3f
Array<double, Dynamic, Dynamic>   => ArrayXXd
Array<double, 3, 3>               => Array33d
```

#### ����
֧��Array + Scalar����Array������Ԫ������Scalar
��Array * Array�У�����Ϊ��Array�еĸ���Ԫ����ˣ�����������ͬ��С��Array�������
������������`abs() sqrt() pow() min() floor() ceil() round()`

֧��ʹ��`a.matrix() m.array()`������ߵ��໥ת��

## Block operations
block���Ǿ�����ָ����ĳ�����飬�ȿ��Ե���ֵ�ֿ��Ե���ֵ�����﷨���£�
```
//Block of size(p, q), starting at (i, j)
matrix.block(i, j, p, q); //construct dynamic 
matrix.block<p,q>(i, j); //construct fixed
```

����`col() row()`��һ�������Block

���������Ĳ�����
```
natrix.topLeftCornor(p,q);
matrix.bottomLeftCornor(p,q);
matrix.leftCols(p);

//vector��Block
v.head(n);
v.tail(n);
v.segment(i, n); //n element starting at i
```

## Advanced initialization

#### comma initializer
`<<`���г�ʼ��ʱ������ʹ��Vector����matrix��Ϊ��ʼ����block���磺
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
`Zero()` - ��������ʼ����̬��������Ҫ�������
`Constant(rows, cols, value)` - ����������
`Random()` - �������������
`Identity()` - ֻ������Matrix
`LinSpace(rows, low, high)` - ֻ������һά���������������low��high�Ⱦ����

ͬʱ��֧�֣�
```
m.topLeftCornor(size/2,size/2).setZero();
m.topRightCornor(size/2,size/2).setIdentity();
m.bottomLeftCornor(size/2,size/2).setIdentity();
m.bottomRightCornor(size/2,size/2).setZero();
```
