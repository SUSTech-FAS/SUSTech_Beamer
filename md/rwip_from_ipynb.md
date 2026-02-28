## 动量轮立杆拉格朗日动力学建模1. 示意图及符号说明

<div style="display: flex; justify-content: space-around; align-items: center;">
  <img src="./lib/pic/动量轮立棍-有坐标系/动量轮立棍-无偏角-有坐标系.svg" width="400">
</div>

建立如图所示动量轮立杆及其坐标系，**假设动量轮安装无偏角**，即 $\alpha = 0$。
同时**不考虑装置自旋（yaw 方向）**，令 $\psi = 0$、$\dot{\psi}=0$，因此 $R_z(\psi)=I$。
由此做出如下符号定义：

- $m_p$: 整个装置的重量（包含动量轮）
- ${}^{\mathbf{b}}{M}_p$: 在 $\{\mathbf{b}\}$ 坐标系下质心位置
- ${}^{\mathbf{o}}{g}_p$: 在 $\{\mathbf{o}\}$ 坐标系下重力加速度矢量
- ${}^{\mathbf{o}}{R}_{\mathbf{b}}$: 从 $\{\mathbf{b}\}$ 坐标系到 $\{\mathbf{o}\}$ 坐标系的旋转矩阵
- ${}^{\mathbf{b}}_O{I}_{p}$: 在 $\{\mathbf{b}\}$ 坐标系下立棍端点处的转动惯量张量（对角矩阵，通过平行轴定理由质心惯量推导）
- ${}^{\mathbf{b}}{I}_{w}$: 在 $\{\mathbf{b}\}$ 坐标系下动量轮的等效惯性张量（对角矩阵）
- 传感器固定在 $\{\mathbf{b}\}$ 坐标系中，传感器获取的欧拉角及欧拉角速度
- ${}^{\mathbf{o}}{E}_{p}$: 传感器获取的整个装置欧拉角（绕 $x$ 轴滚转，绕 $y$ 轴俯仰；不考虑绕 $z$ 轴偏航）
- ${}^{\mathbf{o}}{\dot{E}}_{p} = \begin{bmatrix} \dot{\phi} & \dot{\theta} & 0 \end{bmatrix}^T$: 传感器获取的欧拉角速率
- ${}^{\mathbf{b}}{\omega}_{p} = {}^{\mathbf{b}}T_{\mathbf{o}}(\phi,\theta)\,{}^{\mathbf{o}}{\dot{E}}_{p}$: 在 $\{\mathbf{b}\}$ 坐标系下的平台角速度矢量
- ${}^{\mathbf{b}}{\omega}_{w}$: 在 $\{\mathbf{b}\}$ 坐标系下的动量轮相对角速度矢量

---

其中已知的量包括：

- $m_p$: 已知，直接通过建模软件获得
- ${}^{\mathbf{b}}{M}_{p} = \begin{bmatrix} 0 & 0 & l \end{bmatrix}^T$
- ${}^{\mathbf{o}}{g}_{p} = \begin{bmatrix} 0 & 0 & -m_p g \end{bmatrix}^T$
- ${}^{\mathbf{o}}{R}_{\mathbf{b}} = R_y(\theta) \cdot R_x(\phi) = \begin{bmatrix} \cos\theta & \sin\theta\sin\phi & \sin\theta\cos\phi \\ 0 & \cos\phi & -\sin\phi \\ -\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi \end{bmatrix}$
- ${}^{\mathbf{b}}T_{\mathbf{o}}(\phi,\theta) = \begin{bmatrix} 1 & 0 & -\sin\theta \\ 0 & \cos\phi & \sin\phi\cos\theta \\ 0 & -\sin\phi & \cos\phi\cos\theta \end{bmatrix}$
- ${}^{\mathbf{o}}{\dot{E}}_{p} = \begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ 0 \end{bmatrix}$，由此展开平台角速度：${}^{\mathbf{b}}{\omega}_{p} = {}^{\mathbf{b}}T_{\mathbf{o}}\,{}^{\mathbf{o}}{\dot{E}}_{p} = \begin{bmatrix} \dot{\phi} \\ \dot{\theta}\cos\phi \\ -\dot{\theta}\sin\phi \end{bmatrix}$
- ${}^{\mathbf{b}}_O{I}_{p} = {}^{\mathbf{b}}_M{I}_{p} + m_p\!\left(\|{}^{\mathbf{b}}{M}_{p}\|^2 E - {}^{\mathbf{b}}{M}_{p}\,{}^{\mathbf{b}}{M}_{p}^T\right)$：由平行轴定理得到端点 $O$ 处的转动惯量。由于质心位于立杆轴（$z$ 轴）上，${}^{\mathbf{b}}{M}_{p} = [0,0,l]^T$，故 ${}^{\mathbf{b}}{M}_{p}\,{}^{\mathbf{b}}{M}_{p}^T = \mathrm{diag}(0,0,l^2)$，平行轴修正项 $l^2 E - {}^{\mathbf{b}}{M}_{p}\,{}^{\mathbf{b}}{M}_{p}^T = \mathrm{diag}(l^2,l^2,0)$ 仍为对角矩阵，因此结果仍为对角矩阵：

$$
{}^{\mathbf{b}}_O{I}_{p} = \mathrm{diag}(I_{p1}+m_p l^2,\; I_{p2}+m_p l^2,\; I_{p3})
$$

- ${}^{\mathbf{b}}{I}_{w} = \begin{bmatrix} I_{w1} & 0 & 0 \\ 0 & I_{w1} & 0 \\ 0 & 0 & 0 \end{bmatrix}$
- ${}^{\mathbf{b}}{\omega}_{w} = \begin{bmatrix} p_{w1} & p_{w2} & 0 \end{bmatrix}^T$

## 2. 拉格朗日函数

### 平台动能

$$
K_p = \frac{1}{2}\,{}^{\mathbf{b}}{\omega}_{p}^T \,{}^{\mathbf{b}}_O{I}_{p}\,{}^{\mathbf{b}}{\omega}_{p}
$$

### 飞轮动能

$$
K_w = \frac{1}{2}\,{}^{\mathbf{b}}{\omega}_{w,\text{total}}^T \,{}^{\mathbf{b}}{I}_{w}\,{}^{\mathbf{b}}{\omega}_{w,\text{total}}
$$

其中飞轮总角速度（平台角速度 + 飞轮相对角速度）：

$$
{}^{\mathbf{b}}{\omega}_{w,\text{total}} = {}^{\mathbf{b}}{\omega}_{p} + {}^{\mathbf{b}}{\omega}_{w}
= \begin{bmatrix} \dot{\phi} + p_{w1} \\ \dot{\theta}\cos\phi + p_{w2} \\ -\dot{\theta}\sin\phi \end{bmatrix}
$$

### 势能

$$
P = {}^{\mathbf{o}}{g}_{p}^T \cdot {}^{\mathbf{o}}{R}_{\mathbf{b}}\,{}^{\mathbf{b}}{M}_{p} = -m_p g l \cos\phi\cos\theta
$$

### 拉格朗日函数

$$
L = K_p + K_w - P
$$

$\displaystyle K_{p} = \frac{J_{p1} \dot{\phi}^{2}}{2} + \frac{J_{p2} \dot{\theta}^{2} \cos^{2}{\left(\phi \right)}}{2} + \frac{J_{p3} \dot{\theta}^{2} \sin^{2}{\left(\phi \right)}}{2}$

$\displaystyle K_{w} = \frac{I_{w1} \dot{\phi}^{2}}{2} + I_{w1} \dot{\phi} p_{w1} + \frac{I_{w1} \dot{\theta}^{2} \cos^{2}{\left(\phi \right)}}{2} + I_{w1} \dot{\theta} p_{w2} \cos{\left(\phi \right)} + \frac{I_{w1} p_{w1}^{2}}{2} + \frac{I_{w1} p_{w2}^{2}}{2}$

$\displaystyle P = - g l m_{p} \cos{\left(\phi \right)} \cos{\left(\theta \right)}$

$\displaystyle L = \frac{I_{w1} \dot{\phi}^{2}}{2} + I_{w1} \dot{\phi} p_{w1} + \frac{I_{w1} \dot{\theta}^{2} \cos^{2}{\left(\phi \right)}}{2} + I_{w1} \dot{\theta} p_{w2} \cos{\left(\phi \right)} + \frac{I_{w1} p_{w1}^{2}}{2} + \frac{I_{w1} p_{w2}^{2}}{2} + \frac{J_{p1} \dot{\phi}^{2}}{2} + \frac{J_{p2} \dot{\theta}^{2} \cos^{2}{\left(\phi \right)}}{2} + \frac{J_{p3} \dot{\theta}^{2} \sin^{2}{\left(\phi \right)}}{2} + g l m_{p} \cos{\left(\phi \right)} \cos{\left(\theta \right)}$

## 3. Euler-Lagrange 方程

完整形式：

$$
\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_i}\right) - \frac{\partial L}{\partial q_i} = Q_i
$$

> **注**：重力通过势能 $P$ 已完整包含在 $L = K_p + K_w - P$ 中，其效果体现在 $-\dfrac{\partial L}{\partial q_i} = \dfrac{\partial P}{\partial q_i}$ 项内，因此平台方程右侧广义力 $Q_i = 0$（无额外外力矩）。飞轮方程右侧为电机输出力矩 $\tau$。

**平台欧拉角** ($\phi,\,\theta$)：

$$
\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\phi}}\right) - \frac{\partial L}{\partial \phi} = 0
$$

$$
\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}}\right) - \frac{\partial L}{\partial \theta} = 0
$$

其中 $-\dfrac{\partial L}{\partial \phi}$ 和 $-\dfrac{\partial L}{\partial \theta}$ 中包含重力恢复力矩（来自 $\dfrac{\partial P}{\partial \phi}$、$\dfrac{\partial P}{\partial \theta}$）。

**飞轮相对角速度** ($p_{w1},\,p_{w2}$)：

$$
\frac{d}{dt}\left(\frac{\partial L}{\partial p_{w1}}\right) = \tau_1
$$

$$
\frac{d}{dt}\left(\frac{\partial L}{\partial p_{w2}}\right) = \tau_2
$$

其中 $\tau_1,\,\tau_2$ 为飞轮电机输出力矩，共 4 个方程（无 yaw）。

$\displaystyle \frac{I_{w1} \dot{\theta}^{2} \sin{\left(2 \phi \right)}}{2} + I_{w1} \dot{\theta} p_{w2} \sin{\left(\phi \right)} + I_{w1} \dot{p}_{w1} + \frac{J_{p2} \dot{\theta}^{2} \sin{\left(2 \phi \right)}}{2} - \frac{J_{p3} \dot{\theta}^{2} \sin{\left(2 \phi \right)}}{2} + \ddot{\phi} \left(I_{w1} + J_{p1}\right) + g l m_{p} \sin{\left(\phi \right)} \cos{\left(\theta \right)} = 0$

$\displaystyle I_{w1} \dot{p}_{w2} \cos{\left(\phi \right)} + \ddot{\theta} \left(I_{w1} \cos^{2}{\left(\phi \right)} + J_{p2} \cos^{2}{\left(\phi \right)} + J_{p3} \sin^{2}{\left(\phi \right)}\right) - \dot{\phi} \left(2 I_{w1} \dot{\theta} \cos{\left(\phi \right)} + I_{w1} p_{w2} + 2 J_{p2} \dot{\theta} \cos{\left(\phi \right)} - 2 J_{p3} \dot{\theta} \cos{\left(\phi \right)}\right) \sin{\left(\phi \right)} + g l m_{p} \sin{\left(\theta \right)} \cos{\left(\phi \right)} = 0$

$\displaystyle I_{w1} \left(\ddot{\phi} + \dot{p}_{w1}\right) = \tau_{1}$

$\displaystyle I_{w1} \left(\ddot{\theta} \cos{\left(\phi \right)} - \dot{\phi} \dot{\theta} \sin{\left(\phi \right)} + \dot{p}_{w2}\right) = \tau_{2}$
