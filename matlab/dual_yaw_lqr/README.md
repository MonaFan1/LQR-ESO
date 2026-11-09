# 双 Yaw LQR MATLAB / Simulink 模型

在仓库根目录启动 MATLAB，然后在命令窗口执行：

```matlab
cd(fullfile(pwd, 'matlab', 'dual_yaw_lqr'))
result = run_dual_yaw_lqr;
```

会打开可编辑的 `dual_yaw_lqr.slx`，运行仿真并显示跟踪曲线。也可以直接双击
SLX 后按 Run：默认参数和参考信号已保存在模型工作区中，不依赖基础工作区变量。
需要 MATLAB、Simulink 和 Control System Toolbox，生成环境为 R2024a。

## 参数修改

```matlab
p = dual_yaw_parameters();
p.J1 = 0.178592;  % 换成你的大轴载体惯量
p.J2 = 0.044374;  % 换成你的小轴组件惯量
p.Q = diag([120 1.5 300 2]);
p.R = diag([1 0.6]);
result = run_dual_yaw_lqr(p);
```

每次运行会重新计算 A、B 和 K，再把参数写入模型工作区。请通过这个入口修改参数，
避免只改了惯量却仍沿用旧的 K。两轴角度、角速度和力矩的单位分别是 rad、rad/s、N·m。

## 来源及复原范围

来源：[RoboMaster 原文，2.3.2.1 双 yaw 轴控制](https://bbs.robomaster.com/wiki/36827278/1647282?source=7#head8)，
以及用户提供的 `gimbal_model` 截图。原站网页正文读取受限，通过原站文章 API 和先前保存的正文核对。

| 参数或功能 | 本工程的取值或处理 | 依据 |
|---|---|---|
| J1 / J2 | 0.178592 / 0.044374 kg·m² | 文章提供的 CAD 最大惯量，**不是你的机器实测值** |
| 力矩限幅 | 大轴 ±7、小轴 ±1.2 N·m | 文章仿真设置；不等于任意硬件的连续额定力矩 |
| 固定积分步长 | 0.001 s | 文章仿真设置 |
| Q / R | 参数文件内的演示值 | 原文未公开数值 |
| B1 / B2 | 默认 0 | 原文未公开双轴阻尼值，可填入辨识值 |
| 死区 | 默认 0，关闭 | 原文提到死区，但未给阈值 |
| 参考波形 | 大轴正弦位置且速度参考为 0，小轴平滑复位锯齿及其解析速度 | 按文章描述重建，幅值、频率和复位比例为演示设置 |
| 前馈 | 默认关闭，可启用完整逆动力学前馈 | 截图的小轴 J2 前馈块看起来未连线，未声称复原其未知源码 |

原文的单轴辨识图片另出现 J=0.014035、c=0.029946，但没有标明对应哪一轴，
因此未用它覆盖双轴惯量。原文也未提供可直接下载的完整 MATLAB 工程。

## 状态与动力学

**所有脚本、矩阵、信号和图表统一使用交错状态顺序：**

```text
x = [theta1; omega1; theta2; omega2]
u = [tau_DM4310; tau_GM6020]
e = x_ref - x
u_command = K*e + u_ff
```

原文建模部分使用 `[theta1 theta2 omega1 omega2]`，后文却改成交错排列。
本工程同步重排 A、B、Q、K 和参考信号，避免这种混用。

默认 `model_mode='article'`：theta1 是大轴相对固定底盘的角度，theta2 是小轴
**相对大轴**的角度。小轴的绝对指向为 `theta1+theta2`。底盘在本模型中固定。

```text
q = [theta1; theta2]
M = [J1+J2, J2; J2, J2]
D = diag([B1 B2])
M*q_ddot + D*q_dot = u
```

`CoupledInertia` 块内使用由 M 推导出的完整力矩到加速度矩阵，两路力矩经死区、
限幅后才进入动力学。因此小轴的实际力矩会反作用于大轴。
这里 J1 是大轴载体自身惯量；如果 CAD 给的是两轴锁死后的总惯量，需要核查组成。

提供两个可选模式，便于核对截图与实际机械：

- `dual_yaw_parameters('screenshot')`：两轴都采用绝对角，按截图的近似接线，
  `J1*a1=tau1-tau2-B1*w1`，`J2*a2=tau2-B2*w2`。
- `dual_yaw_parameters('physical')`：两轴也采用绝对角，但小轴摩擦取
  `B2*(w2-w1)`，其反作用同时施加于大轴。此模型与默认相对坐标模型严格等价。

更换模式时第二轴的角度含义也随之改变，不能直接把同一组编码器角度混用。
仿真采用连续展开角，不模拟编码器 ±pi 跳变。

## 采样控制、前馈与验证

```matlab
% 1000 Hz 离散 LQR：零阶保持 + dlqr，连续机械模型
p = dual_yaw_parameters('article', 'discrete');
run_dual_yaw_lqr(p);

% 自洽的平滑扫频轨迹 + 逆动力学前馈
p = dual_yaw_parameters();
p.ref_kind = 'chirp';
p.enable_feedforward = true;
run_dual_yaw_lqr(p);

% 检查状态转换、动力学接线、闭环恢复、限幅与解析轨迹导数
validate_dual_yaw_lqr;
```

默认使用连续 LQR（`lqr`）；0.001 s 是数值积分步长，并不自动表示控制器以
1000 Hz 采样。离散选项明确使用 `c2d`、`dlqr` 和零阶保持。

文章工况故意令大轴速度参考为 0，所以位置和速度参考并非一条自洽轨迹；出现相位
滞后属于这个测试的预期现象。完整前馈使用位置轨迹的真实解析导数。

输出包括：`dual_yaw_response.png`（响应曲线）、`dual_yaw_model.png`（框图预览）、
`dual_yaw_result.mat`（参数、状态、参考、误差和限幅前后力矩）。仿真只建立控制模型，
没有接入或修改机器人的固件。
