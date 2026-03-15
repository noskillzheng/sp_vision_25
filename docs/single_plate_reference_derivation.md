# 单目标单装甲板参考轨迹推导

本文将 `Target` 的 11 维状态显式展开为某一块装甲板的世界坐标、几何 `yaw/pitch`、弹道补偿后的控制参考，以及加速度约束下的最优跟踪标准形式。

范围限定如下：

- 固定一个 `Target`
- 固定一块装甲板 `i`
- 不处理装甲板切换
- 时间变量 $t$ 表示相对当前 `Target` 状态时间戳的预测时间

对应实现主要来自：

- `tasks/auto_aim/target.cpp`
- `tasks/auto_aim/planner/planner.cpp`
- `tasks/auto_aim/aimer.cpp`
- `tools/trajectory.cpp`
- `tools/math_tools.cpp`

## 1. 11 维状态定义

`Target` 的状态定义为

$$
x=\begin{bmatrix}
c_x & v_x & c_y & v_y & c_z & v_z & a & \omega & r & l & h
\end{bmatrix}^{\top}
$$

各分量含义如下：

- $c_x,c_y,c_z$：目标旋转中心在世界系中的位置
- $v_x,v_y,v_z$：目标旋转中心在世界系中的速度
- $a$：参考装甲板的相位角
- $\omega$：目标绕竖直轴旋转的角速度
- $r$：基础半径
- $l$：长短轴半径差
- $h$：高低装甲板高度差

这与 `target.cpp` 中注释

$$
x=[c_x,v_x,c_y,v_y,c_z,v_z,a,\omega,r,l,h]
$$

一致。

## 2. 连续时间运动模型

当前实现 `predict(dt)` 对应的连续时间模型是匀速平移加匀速转动：

$$
c_x(t)=c_x+v_x t
$$

$$
c_y(t)=c_y+v_y t
$$

$$
c_z(t)=c_z+v_z t
$$

$$
a(t)=a+\omega t
$$

其余参数视为常数：

$$
\omega(t)=\omega,\qquad r(t)=r,\qquad l(t)=l,\qquad h(t)=h
$$

因此该模型本质上是：

- 旋转中心做匀速直线运动
- 装甲板组相对旋转中心做匀角速度圆周运动
- 长短轴和高低差不随时间变化

## 3. 固定装甲板编号后的几何模型

设该目标共有 $N$ 块装甲板，其中：

- 平衡车通常 $N=2$
- 前哨站/基地通常 $N=3$
- 普通车通常 $N=4$

固定装甲板编号 $i\in\{0,\dots,N-1\}$，定义其静态相位偏置

$$
\phi_i=\frac{2\pi i}{N}
$$

对于四装甲板目标，代码中第 $1,3$ 块板会附加长短轴半径差和高度差。定义指示量

$$
\lambda_i=
\begin{cases}
1,& N=4,\ i\in\{1,3\}\\
0,& \text{otherwise}
\end{cases}
$$

进一步定义该板的等效半径和高度偏置

$$
\rho_i=r+\lambda_i l
$$

$$
\eta_i=\lambda_i h
$$

于是第 $i$ 块装甲板的相位角为

$$
\alpha_i(t)=a+\omega t+\phi_i
$$

其世界系坐标为

$$
X_i(t)=c_x+v_x t-\rho_i\cos\alpha_i(t)
$$

$$
Y_i(t)=c_y+v_y t-\rho_i\sin\alpha_i(t)
$$

$$
Z_i(t)=c_z+v_z t+\eta_i
$$

这正是 `Target::h_armor_xyz()` 的连续时间版本。

若用复数表示平面位置，则可写为

$$
P_{xy,i}(t)=\big(c_x+v_x t\big)+j\big(c_y+v_y t\big)-\rho_i e^{j\alpha_i(t)}
$$

## 4. 单板几何 yaw/pitch 函数

代码中球坐标映射 `xyz2ypd()` 为

$$
\psi_i(t)=\operatorname{atan2}(Y_i(t),X_i(t))
$$

$$
d_i(t)=\sqrt{X_i(t)^2+Y_i(t)^2}
$$

$$
\theta_i(t)=\operatorname{atan2}(Z_i(t),d_i(t))
$$

其中：

- $\psi_i(t)$ 是几何 `yaw`
- $\theta_i(t)$ 是几何 `pitch`
- $d_i(t)$ 是水平距离

如果直接把 $\psi_i(t)$ 用于控制，会遇到 $\pm\pi$ 包角点。因此理论分析时应使用连续展开角

$$
\bar\psi_i(t)=\operatorname{unwrap}(\psi_i(t))
$$

后续所有 `yaw` 导数默认都针对 $\bar\psi_i(t)$。

## 5. 板中心位置的一阶与二阶导数

先对板中心坐标求导。由 $\alpha_i(t)=a+\omega t+\phi_i$ 得

$$
\dot\alpha_i(t)=\omega
$$

于是

$$
\dot X_i(t)=v_x+\rho_i\omega\sin\alpha_i(t)
$$

$$
\dot Y_i(t)=v_y-\rho_i\omega\cos\alpha_i(t)
$$

$$
\dot Z_i(t)=v_z
$$

再求二阶导数：

$$
\ddot X_i(t)=\rho_i\omega^2\cos\alpha_i(t)
$$

$$
\ddot Y_i(t)=\rho_i\omega^2\sin\alpha_i(t)
$$

$$
\ddot Z_i(t)=0
$$

记

$$
S_i(t)=X_i(t)^2+Y_i(t)^2
$$

$$
d_i(t)=\sqrt{S_i(t)}
$$

$$
M_i(t)=X_i(t)\dot X_i(t)+Y_i(t)\dot Y_i(t)
$$

则水平距离的一阶导数为

$$
\dot d_i(t)=\frac{M_i(t)}{d_i(t)}
$$

二阶导数为

$$
\ddot d_i(t)=
\frac{\left(\dot X_i(t)^2+\dot Y_i(t)^2+X_i(t)\ddot X_i(t)+Y_i(t)\ddot Y_i(t)\right)d_i(t)^2-M_i(t)^2}
{d_i(t)^3}
$$

## 6. 几何 yaw/pitch 的导数

### 6.1 yaw 的一阶与二阶导数

由 $\bar\psi_i(t)=\operatorname{atan2}(Y_i(t),X_i(t))$ 得

$$
\dot{\bar\psi}_i(t)=\frac{X_i(t)\dot Y_i(t)-Y_i(t)\dot X_i(t)}{S_i(t)}
$$

再求导可得

$$
\ddot{\bar\psi}_i(t)=
\frac{
\left(X_i(t)\ddot Y_i(t)-Y_i(t)\ddot X_i(t)\right)S_i(t)
-2\left(X_i(t)\dot Y_i(t)-Y_i(t)\dot X_i(t)\right)\left(X_i(t)\dot X_i(t)+Y_i(t)\dot Y_i(t)\right)
}
{S_i(t)^2}
$$

上式可用于直接构造 `yaw` 参考角速度和角加速度。

### 6.2 几何 pitch 的一阶与二阶导数

由

$$
\theta_i(t)=\operatorname{atan2}(Z_i(t),d_i(t))
$$

得一阶导数

$$
\dot\theta_i(t)=\frac{d_i(t)\dot Z_i(t)-Z_i(t)\dot d_i(t)}{d_i(t)^2+Z_i(t)^2}
$$

由于 $\ddot Z_i(t)=0$，其二阶导数为

$$
\ddot\theta_i(t)=
\frac{
-Z_i(t)\ddot d_i(t)\left(d_i(t)^2+Z_i(t)^2\right)
-2\left(d_i(t)\dot Z_i(t)-Z_i(t)\dot d_i(t)\right)\left(d_i(t)\dot d_i(t)+Z_i(t)\dot Z_i(t)\right)
}
{\left(d_i(t)^2+Z_i(t)^2\right)^2}
$$

这给出了单板几何 `pitch` 的显式二阶导数。

## 7. 弹道补偿后的控制参考

当前 `Aimer` 与 `Planner` 实际使用的 `pitch` 并不是几何 $\theta_i(t)$，而是无空气阻力弹道补偿后的抬头角。

设：

- 弹速为 $v_0$
- 重力加速度为 $g$
- 水平距离为 $d_i(t)$
- 高度差为 $Z_i(t)$
- 弹道抬头角为 $\beta_i(t)$

则抛体方程为

$$
Z_i(t)=d_i(t)\tan\beta_i(t)-\frac{g\,d_i(t)^2}{2v_0^2\cos^2\beta_i(t)}
$$

令

$$
u_i(t)=\tan\beta_i(t),\qquad k=\frac{g}{2v_0^2}
$$

则 $\beta_i(t)$ 满足隐式方程

$$
F\big(u_i(t),d_i(t),Z_i(t)\big)=k\,d_i(t)^2\big(1+u_i(t)^2\big)-d_i(t)u_i(t)+Z_i(t)=0
$$

这与 `Trajectory(v0, d, h)` 的解析求解等价。

### 7.1 弹道抬头角的一阶导数

对上式做隐式求导：

$$
F_u \dot u_i + F_d \dot d_i + \dot Z_i = 0
$$

其中

$$
F_u=2k\,d_i^2 u_i-d_i
$$

$$
F_d=2k\,d_i\big(1+u_i^2\big)-u_i
$$

因此

$$
\dot u_i(t)= -\frac{F_d \dot d_i(t)+\dot Z_i(t)}{F_u}
$$

又因为

$$
\beta_i(t)=\arctan u_i(t)
$$

故

$$
\dot\beta_i(t)=\frac{\dot u_i(t)}{1+u_i(t)^2}
$$

### 7.2 弹道抬头角的二阶导数

继续对隐式方程求二阶导数：

$$
F_u \ddot u_i
+F_{uu}\dot u_i^2
+2F_{ud}\dot u_i\dot d_i
+F_{dd}\dot d_i^2
+F_d \ddot d_i
=0
$$

其中

$$
F_{uu}=2k\,d_i^2
$$

$$
F_{ud}=4k\,d_i u_i-1
$$

$$
F_{dd}=2k\big(1+u_i^2\big)
$$

所以

$$
\ddot u_i(t)=
-\frac{
F_{uu}\dot u_i(t)^2
+2F_{ud}\dot u_i(t)\dot d_i(t)
+F_{dd}\dot d_i(t)^2
+F_d \ddot d_i(t)
}{F_u}
$$

再由 $\beta_i(t)=\arctan u_i(t)$ 得

$$
\ddot\beta_i(t)=
\frac{
\ddot u_i(t)\big(1+u_i(t)^2\big)-2u_i(t)\dot u_i(t)^2
}
{\big(1+u_i(t)^2\big)^2}
$$

### 7.3 最终控制参考

设系统实际标定补偿量为：

- `yaw_offset` 对应 $\delta_y$
- `pitch_offset` 对应 $\delta_p$

则单板控制参考可写为

$$
q_{\mathrm{ref}}(t)=
\begin{bmatrix}
\bar\psi_i(t)+\delta_y\\
-\beta_i(t)-\delta_p
\end{bmatrix}
$$

其一阶导数为

$$
\dot q_{\mathrm{ref}}(t)=
\begin{bmatrix}
\dot{\bar\psi}_i(t)\\
-\dot\beta_i(t)
\end{bmatrix}
$$

其二阶导数为

$$
\ddot q_{\mathrm{ref}}(t)=
\begin{bmatrix}
\ddot{\bar\psi}_i(t)\\
-\ddot\beta_i(t)
\end{bmatrix}
$$

因此在单板假设下，`yaw/pitch` 参考轨迹、角速度参考、角加速度参考都可以直接由 11 维状态显式计算。

## 8. 加速度约束下的最优跟踪标准形式

现在考虑二维云台状态：

$$
q(t)=
\begin{bmatrix}
\mathrm{yaw}(t)\\
\mathrm{pitch}(t)
\end{bmatrix}
$$

$$
\dot q(t)=
\begin{bmatrix}
\dot{\mathrm{yaw}}(t)\\
\dot{\mathrm{pitch}}(t)
\end{bmatrix}
$$

控制输入定义为角加速度：

$$
u(t)=
\begin{bmatrix}
u_y(t)\\
u_p(t)
\end{bmatrix}
=
\begin{bmatrix}
\ddot{\mathrm{yaw}}(t)\\
\ddot{\mathrm{pitch}}(t)
\end{bmatrix}
$$

若采用双积分器模型，则云台动力学为

$$
\dot x_g(t)=A x_g(t)+B u(t)
$$

其中

$$
x_g(t)=
\begin{bmatrix}
q(t)\\
\dot q(t)
\end{bmatrix}
,\qquad
A=
\begin{bmatrix}
0&I\\
0&0
\end{bmatrix}
,\qquad
B=
\begin{bmatrix}
0\\
I
\end{bmatrix}
$$

加速度约束为

$$
|u_y(t)|\le a_{y,\max}
$$

$$
|u_p(t)|\le a_{p,\max}
$$

在给定参考轨迹 $q_{\mathrm{ref}}(t)$ 的情况下，一个自然的最优跟踪问题是

$$
\min_{u(\cdot)}\ J=
\frac{1}{2}\int_0^T
\left[
\big(q(t)-q_{\mathrm{ref}}(t)\big)^\top Q_p \big(q(t)-q_{\mathrm{ref}}(t)\big)
+
\big(\dot q(t)-\dot q_{\mathrm{ref}}(t)\big)^\top Q_v \big(\dot q(t)-\dot q_{\mathrm{ref}}(t)\big)
+
u(t)^\top R u(t)
\right]dt
$$

subject to

$$
\dot x_g(t)=A x_g(t)+B u(t)
$$

$$
|u_y(t)|\le a_{y,\max},\qquad |u_p(t)|\le a_{p,\max}
$$

### 8.1 误差系统写法

定义误差状态

$$
e(t)=
\begin{bmatrix}
q(t)-q_{\mathrm{ref}}(t)\\
\dot q(t)-\dot q_{\mathrm{ref}}(t)
\end{bmatrix}
$$

则

$$
\dot e(t)=A e(t)+B\big(u(t)-\ddot q_{\mathrm{ref}}(t)\big)
$$

若进一步定义

$$
\nu(t)=u(t)-\ddot q_{\mathrm{ref}}(t)
$$

则误差系统变成标准双积分器

$$
\dot e(t)=A e(t)+B\nu(t)
$$

但约束会变成时变输入约束

$$
-a_{\max}-\ddot q_{\mathrm{ref}}(t)\le \nu(t)\le a_{\max}-\ddot q_{\mathrm{ref}}(t)
$$

其中

$$
a_{\max}=
\begin{bmatrix}
a_{y,\max}\\
a_{p,\max}
\end{bmatrix}
$$

这说明：

- 当 $|\ddot q_{\mathrm{ref}}(t)|$ 始终低于加速度上限时，精确跟踪存在可行性
- 当某些时刻参考角加速度超过上限时，零误差跟踪必然不可行
- 此时最优控制器的任务不再是“完美跟踪”，而是“在不可行约束下最优退让”

## 9. Yaw 轴相邻切板点的局部边界模型

前文分析固定单板的连续参考轨迹。本节进一步考虑 `yaw` 轴在"当前目标板切换到相邻另一块板"时的局部边界情况。

范围限定如下：

- 只看 `yaw` 单轴
- 只考虑相邻两块板之间的切换
- 只分析切板点附近的局部结构
- 先忽略 `pitch` 与空气阻力对切板时刻的耦合影响

### 9.1 平面几何与单板 yaw 的小半径近似

只看世界系水平面。记目标旋转中心投影为

$$
C(t)=
\begin{bmatrix}
c_x+v_x t\\
c_y+v_y t
\end{bmatrix}
$$

定义其极坐标量

$$
R(t)=\|C(t)\|
$$

$$
\psi_c(t)=\operatorname{atan2}(c_y+v_y t,\ c_x+v_x t)
$$

对于第 $i$ 块装甲板，其平面相位仍记为

$$
\alpha_i(t)=a+\omega t+\phi_i
$$

若装甲板半径相对目标距离较小，即 $\rho_i \ll R(t)$，则第 $i$ 块板的 `yaw` 可作一阶近似

$$
\psi_i(t)\approx \psi_c(t)-\frac{\rho_i}{R(t)}\sin\big(\alpha_i(t)-\psi_c(t)\big)
$$

这里：

- $\psi_c(t)$ 是旋转中心的方位角
- 第二项是装甲板绕旋转中心转动带来的横向修正

定义相对中心视线的相位误差

$$
\delta_i(t)=\alpha_i(t)-\psi_c(t)
$$

则单板 `yaw` 近似可写成

$$
\psi_i(t)\approx \psi_c(t)-\frac{\rho_i}{R(t)}\sin\delta_i(t)
$$

### 9.2 相邻切板时刻

设目标共有 $N$ 块装甲板，相邻装甲板的相位间隔为

$$
\Delta=\frac{2\pi}{N}
$$

若选板规则是"选择相对中心视线角度最近的装甲板"，则第 $i$ 块板被选中的局部条件可写成

$$
\delta_i(t)\in\left[-\frac{\Delta}{2},\frac{\Delta}{2}\right)
$$

相邻切板时刻 $t_s$ 满足

$$
\delta_i(t_s)=\frac{\Delta}{2}
$$

与此同时，第 $i+1$ 块板满足

$$
\delta_{i+1}(t_s)=-\frac{\Delta}{2}
$$

这是因为

$$
\delta_{i+1}(t)=\delta_i(t)-\Delta
$$

故当第 $i$ 块板达到右边界时，第 $i+1$ 块板恰好达到左边界，两块板在切板点等距。

### 9.3 切板点的左右极限与跳变量

记

$$
R_s=R(t_s),\qquad \psi_s=\psi_c(t_s)
$$

则切板前的参考 `yaw` 左极限为

$$
\psi_s^-=\psi_s-\frac{\rho_i}{R_s}\sin\frac{\Delta}{2}
$$

切板后的参考 `yaw` 右极限为

$$
\psi_s^+=\psi_s+\frac{\rho_{i+1}}{R_s}\sin\frac{\Delta}{2}
$$

因此切板瞬间的 `yaw` 跳变量为

$$
J=\psi_s^+-\psi_s^-=
\frac{\rho_i+\rho_{i+1}}{R_s}\sin\frac{\Delta}{2}
$$

若相邻两块板等半径，即 $\rho_i=\rho_{i+1}=\rho$，则

$$
J=2\frac{\rho}{R_s}\sin\frac{\Delta}{2}
$$

典型情况：

- 四装甲板目标，$\Delta=\pi/2$：

$$
J=\sqrt{2}\frac{\rho}{R_s}
$$

- 三装甲板目标，$\Delta=2\pi/3$：

$$
J=\sqrt{3}\frac{\rho}{R_s}
$$

- 两装甲板目标，$\Delta=\pi$：

$$
J=2\frac{\rho}{R_s}
$$

这说明切板跳变量的主要决定因素是：

- 目标距离 $R_s$
- 装甲板半径
- 装甲板数量 $N$

### 9.4 切板点附近的局部锯齿模型

令局部时间变量

$$
\tau=t-t_s
$$

在切板点附近，对左右两支分别做一阶线性化：

$$
\psi_{\mathrm{ref}}(\tau)\approx
\begin{cases}
\psi_s^-+k^- \tau,& \tau<0\\
\psi_s^++k^+ \tau,& \tau>0
\end{cases}
$$

其中局部斜率为

$$
k^- \approx \dot\psi_c(t_s)-\frac{\rho_i}{R_s}\cos\frac{\Delta}{2}\Big(\omega-\dot\psi_c(t_s)\Big)
$$

$$
k^+ \approx \dot\psi_c(t_s)-\frac{\rho_{i+1}}{R_s}\cos\frac{\Delta}{2}\Big(\omega-\dot\psi_c(t_s)\Big)
$$

若相邻两块板等半径，则 $k^- \approx k^+ \approx k$，于是局部参考可进一步写成

$$
\psi_{\mathrm{ref}}(\tau)\approx
\psi_s+k\tau+\frac{J}{2}\operatorname{sgn}(\tau)
$$

这正是切板点附近的局部锯齿结构。也就是说：

- 远离切板点时，每一支近似是一条斜率为 $k$ 的直线
- 到达切板点时，参考 `yaw` 发生幅值为 $J$ 的瞬时跳跃

因此从局部上看，切板参考可理解为"线性漂移 + 周期性模运算导致的锯齿跳变"。

### 9.5 移动坐标系下的阶跃边界问题

现在只看 `yaw` 轴云台，并采用双积分器模型

$$
\ddot q(\tau)=u(\tau),\qquad |u(\tau)|\le a_{\max}
$$

其中：

- $q(\tau)$ 是云台 `yaw`
- $u(\tau)$ 是云台角加速度
- $a_{\max}$ 是该轴的最大可用角加速度

在等斜率近似 $k^- = k^+ = k$ 下，定义去掉公共线性漂移后的移动坐标

$$
z(\tau)=q(\tau)-(\psi_s+k\tau)
$$

则

$$
\ddot z(\tau)=u(\tau)
$$

参考轨迹变成一个纯阶跃：

$$
z_{\mathrm{ref}}(\tau)=
\begin{cases}
-\dfrac{J}{2},& \tau<0\\
+\dfrac{J}{2},& \tau>0
\end{cases}
$$

因此，切板点本质上被化简为：

- 一个受加速度约束的双积分器
- 跟踪一个在 $\tau=0$ 处跳变的阶跃参考

### 9.6 无缝切板的最优边界条件

若希望云台在切板时实现"无缝接入"，则需要在切板前某个时刻 $-T$ 离开旧板参考，并在 $\tau=0$ 时正好对齐到新板参考。

在移动坐标系中，其边界条件为

$$
z(-T)=-\frac{J}{2},\qquad \dot z(-T)=0
$$

$$
z(0)=+\frac{J}{2},\qquad \dot z(0)=0
$$

也就是说，系统需要在有限时间内完成从

$$
\left(-\frac{J}{2},0\right)
\longrightarrow
\left(+\frac{J}{2},0\right)
$$

的状态转移。

这正是标准受限双积分器的最小时间转移问题。

### 9.7 时间最优解与最小预瞄时间

对上述双积分器边界问题，时间最优控制是经典 bang-bang 形式：

$$
u^*(\tau)=
\begin{cases}
\operatorname{sgn}(J)\,a_{\max},& -T\le \tau<-\dfrac{T}{2}\\
-\operatorname{sgn}(J)\,a_{\max},& -\dfrac{T}{2}\le \tau\le 0
\end{cases}
$$

其最小可行转移时间为

$$
T_{\min}=2\sqrt{\frac{|J|}{a_{\max}}}
$$

这给出了切板点附近最核心的理论边界：

- 若留给云台的剩余时间小于 $T_{\min}$，则不可能无缝完成切板
- 若提前时间达到 $T_{\min}$，则存在极限意义下的时间最优切换轨迹

这也是文档中"提前减速"策略的严格化表达。

### 9.8 何时开始切换：时间判据与角度判据

设当前仍在旧板参考支上运动，距切板点的剩余时间为

$$
t_{\mathrm{remain}}=t_s-t
$$

则一个自然的最优切换判据为

$$
t_{\mathrm{remain}}\le T_{\min}
$$

即

$$
t_s-t\le 2\sqrt{\frac{|J|}{a_{\max}}}
$$

若进一步用局部斜率 $k$ 把时间判据改写为旧支上的角度距离判据，可得

$$
\Delta\psi_{\mathrm{remain}}\le |k|\,T_{\min}
$$

即

$$
\Delta\psi_{\mathrm{remain}}
\le
2|k|\sqrt{\frac{|J|}{a_{\max}}}
$$

其中 $\Delta\psi_{\mathrm{remain}}$ 表示当前旧板参考点到切板点的剩余角度距离。

这比经验式"看到切板点快到了就减速"更严格，因为它同时显式依赖：

- 切板跳变量 $J$
- 云台最大角加速度 $a_{\max}$
- 当前参考支斜率 $k$

### 9.9 左右斜率不等时的边界推广

若相邻两块板的等效半径不同，则一般有

$$
k^- \neq k^+
$$

此时切板不再是"纯阶跃 + 公共线性项"，而是"位置跳变 + 速度跳变"的组合。

原坐标下的边界条件为

$$
q(-T)=\psi_s^- - k^- T,\qquad \dot q(-T)=k^-
$$

$$
q(0)=\psi_s^+,\qquad \dot q(0)=k^+
$$

因此系统需要完成从

$$
\big(0,0\big)
\longrightarrow
\big(J,\Delta k\big)
$$

的转移，其中

$$
\Delta k=k^+-k^-
$$

这仍然是受限双积分器的最优转移问题，只是终端速度不再为零。

在该情形下，时间最优控制仍保持 bang-bang 结构，但切换时刻不再必然位于时间中点，需由终端速度条件共同决定。

### 9.10 对后续理论化 MPC 的意义

这一节得到的不是数值算法，而是切板点的解析边界结构：

1. 参考 `yaw` 在切板点附近可写成局部锯齿模型
2. 跳变量 $J$ 可由目标距离、板半径和板数近似显式给出
3. 切板的本质是受限双积分器对阶跃参考的最优转移
4. 最小预瞄时间满足

$$
T_{\min}=2\sqrt{\frac{|J|}{a_{\max}}}
$$

因此，后续若要把 `Planner` 理论化，不应只把切板视作参考轨迹中的一个数值跳点，而应把它看成：

- 一个可解析估计跳变量的混合切换边界
- 一个带显式最小过渡时间约束的最优轨迹拼接问题

## 10. 结论

在“固定单个 `Target` + 固定单块装甲板”的假设下，可以得到完整的解析模型：

1. 11 维状态可显式映射到装甲板世界坐标 $X_i(t),Y_i(t),Z_i(t)$
2. 世界坐标可显式映射到几何参考 $\bar\psi_i(t),\theta_i(t)$
3. 弹道方程可通过隐式函数得到补偿后的控制参考 $q_{\mathrm{ref}}(t)$
4. 该参考轨迹的一阶、二阶导数都可显式计算
5. 在此基础上，可把加速度约束下的最优跟踪写成标准双积分器最优控制问题

这一步完成后，后续的理论化工作主要有两条路线：

- 路线 A：把上述连续时间问题离散化，替代当前 `Planner` 的采样差分参考构造
- 路线 B：继续分析单轴受限双积分器跟踪解析参考轨迹的最优控制结构，再扩展到双轴和装甲板切换
