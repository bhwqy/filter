# My implementation for Kalman Filter and Particle Filter

目前初步实现了卡尔曼滤波，扩展卡尔曼滤波，粒子滤波等滤波算法。滤波采用了一阶马尔可夫假设和状态观测独立假设。

## Kalman Filter

卡尔曼滤波是线性动态系统的最优状态估计，设运动方程与观测方程满足
$$ x_k = A_k x_{k-1} + u_k + w_k $$
$$ z_k = C_k x_k + v_k $$
其中$k=1,...,N$，所有状态与噪声满足高斯分布，假设噪声服从零均值高斯分布有
$$ w_k \thicksim N(0, R), v_k \thicksim N(0, Q) $$
则第k时刻的预测与协方差的均值为
$$ \check{x}_k = A_k \hat{x}_{k-1} + u_k $$
$$ \check{P}_k = A_k\hat{P}_{k-1}A_k^T + R_k $$
利用观测数据更新，计算卡尔曼增益
$$ K = \check{P}_kC_k^T(C_k\check{P}_kC_k^T+Q_k)^{-1} $$
计算后验概率分布
$$ \hat{x}_k = \check{x}_k + K(z_k - C_k\check{x}_k) $$
$$ \hat{P}_k = (I - KC_k)\check{P}_k $$

## Extended Kalman Filter

扩展卡尔曼滤波是考虑非线性系统在某个点附近的运动方程与观测方程的一阶泰勒展开，将非高斯分布近似假设成高斯分布。设运动方程与观测方程满足
$$ x_k \approx f(\hat{x}_{k-1}, u_k) + \frac{\partial f}{\partial x_{k-1}}(x_{k-1}-\hat{x}_{k-1}) + w_k $$
记$F= \frac{\partial f}{\partial x_{k-1}}$
$$ z_k \approx h(\check{x}_k) + \frac{\partial h}{\partial x_k} (x_k - \check{x}_k) + v_k $$
记$H=\frac{\partial h}{\partial x_k}$
从而第k时刻的预测与协方差的均值为
$$ \check{x}_k = f(\hat{x}_{k-1}, u_k) $$
$$ \check{P}_k = F\hat{P}_{k-1}F^T + R_k $$
在更新时的卡尔曼增益为
$$ K = \check{P}_kH^T(H\check{P}_kH^T + Q_k)^{-1} $$
计算后验概率分布
$$ \hat{x}_k = \check{x}_k + K(z_k - h(\check{x}_k)) $$
$$ \hat{P}_k = (I - KH)\check{P}_k $$

## Particle Filter

粒子滤波主要解决的是非线性非高斯系统的状态估计问题。实际系统中不能用一阶泰勒近似的状态估计问题其实很少，因而大多数情况下粒子滤波主要是针对传感器噪声不满足高斯分布。这类系统难以求得解析解，因而通常使用Monte-Carlo Sampling来得出近似解。

首先说明一下求解目标是$P(z_t|x_1,...,x_t)$
预测求$z_t$先验 
$$ P(z_t|x_1,...x_{t-1})=\int_{z_{t-1}}P(z_t|z_{t-1})P(z_{t-1}|x_1,...,x_{t-1})dz_{t-1} $$
更新已知$x_t$求$z_t$的后验
$$ P(z_t|x_1,...x_t) \propto P(x_t|z_t)P(z_t|x_1,...,x_{t-1}) $$

由于非线性动态系统难以求解，引入重要性采样
$$ E_{p(z)}(f(z))=\int p(z)f(z)dz = \int\frac{p(z)}{q(z)} q(z)f(z)dz\approx\frac{1}{N}\sum_{i=1}^N f(z_i)\frac{p(z_i)}{q(z_i)} $$

其中$w_i=\frac{p(z_i)}{q(z_i)}$两个分布之间的差距太大了话，总是采样采不到重要的样本，采的可能都是实际分布概率值小的部分。也就是采样效率不均匀的问题。

对于目标问题权值为$w_t^i=\frac{P(z_t^i|x_1,...,x_t)}{Q(z_t^i|x_1,...,x_t)}$但由于$P(z_t^i|x_1,...,x_t)$非常难求，因而引入SIS顺序重要性采样，即$P(z_t|x_1,...,x_t)\rightarrow P(z_1,...,z_t|x_1,...,x_t)$
此时
$$ w_t^i=\frac{P(z_1^i,...,z_t^i|x_1,...,x_t)}{Q(z_1^i,...,z_t^i|x_1,...,x_t)}\propto w_{t-1}^i\frac{P(x_t|z_t^i)P(z_t^i|z_{t-1})}{Q(z_t^i|z_1^i,...,z_{t-1}^i,x_1,...,x_t)} $$
实际程序中还需要对$w$进行归一化以提高数值鲁棒性。

但是SIS中仍然存在随机变量维度上升后出现的权值退化问题，即大量权重趋近于0，这是因为高维空间需要更多的样本才能反应空间特征。样本不够就会忽略一些高维特征，导致某些维度特征接近0的情况出现。为了解决这个问题，除了采集更多的样本，另外两种方法分别是寻找更好的采样函数$Q(z)$和SIR。

SIR采样是经过重要性采样后得到了N个样本点以及对应的权重，用权重来作为采样的概率，重新测采样出N个样本。实际程序中使用概率密度函数 (pdf) 来计算得到概率分布函数 (cdf)，通过从 [0,1] 之间进行均匀采样，来得到相应的的值。

一个较好的$Q(z)$的选择是$P(z_t|z_{t-1})$，即状态转移矩阵，此时
$$ w_t^i=w_{t-1}^iP(x_t|z_t^i) $$

## 与因子图优化的对比
1 滤波相比优化计算简单，在计算资源受限或者待估计量较为简单的场合发挥着重要作用

2 马尔可夫假设太强。在视觉SLAM中比如存在回环的话就会破坏马尔可夫性，而因子图采用条件独立性来描述，更符合实际情况。

3 滤波算法仅在工作点上做一次优化，当因为优化而使得工作点改变时，可能会失效。而优化则会多次迭代。

## TODO

Add more tests.
