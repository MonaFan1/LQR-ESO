# LQR-ESO

基于 LibXR/xrobot 的云台 Yaw 轴控制模块：LQR 状态反馈、角度误差积分、三阶线性 ESO 扰动估计与补偿。

## 来源

核心代码来自本地 `LQR-ESO TEST/Modules/LqrEso/LqrEso.hpp`。
原工程 README 注明参考南京理工大学步兵云台开源实现 `yaw_auto_lqr_eso_controller`，作者戴子轩；本仓库保留该来源说明。
本次上传保留源工程现有 C++ 控制实现和参数接口。

## 文件

- `LqrEso.hpp`：控制器类、参数结构、调试输出、xrobot 模块清单及所有内联方法。
- `CMakeLists.txt`：暴露模块头文件路径，兼容 LibXR 模块加载。
- `SOURCES.md`：本次上传的原文件校验信息。

当前控制器为单头文件实现，无额外的 `yaw_auto_lqr_eso_controller.c/.h` 文件。
`k_theta`、`k_omega` 等增益由调用方提供，头文件中不包含 LQR/Riccati 增益求解器。

## 集成

依赖 LibXR 的 `app_framework.hpp`、`cycle_value.hpp`、`libxr_def.hpp`，使用 C++17 或更新标准。
仓库名是 `LQR-ESO`，模块目录及类名是 **`LqrEso`**。在机器人工程根目录执行：

```sh
git clone https://github.com/MonaFan1/LQR-ESO.git Modules/LqrEso
```

LibXR CMake 扫描模块目录并加载 `CMakeLists.txt`，xrobot 读取 `LqrEso.hpp` 中的清单。

```cpp
#include "LqrEso.hpp"

// hw、app、param 由调用方提供。
LqrEso controller(hw, app, param);
```

## 接口

```cpp
float Calc(const LibXR::CycleValue<float>& target, float pos, float vol,
           float target_vol, float target_ddot, float tau_meas, float dt_s);
void Reset(float theta, float omega);
const Output& last_output() const;
```

- `target` / `pos`：目标角度和反馈角度，单位 rad；内部维护连续角度用于控制及 ESO。
- `vol` / `target_vol`：反馈和目标角速度，单位 rad/s。
- `target_ddot`：目标角加速度，单位 rad/s²。
- `tau_meas`：电机反馈力矩，单位 N·m，用于力矩滤波及偏置补偿。
- `dt_s`：实际控制周期，单位 s。
- 返回值：经过软限幅、硬限幅和可选斜率限幅的力矩命令，单位 N·m。

构造函数使用 `(hw, app, param)`。参数字段及顺序见 `Param` 与模块清单。
首次调用或 `Reset()` 后首次调用 `Calc()` 用于热启动并返回零力矩。
失能或重新开始控制时应重置状态；`last_output()` 提供各前馈、反馈、ESO 补偿及限幅中间量。

## 控制流程

模型前馈 `J * alpha_ref + B * omega_ref + CoulombComp(omega_ref)`，
叠加角度/角速度反馈、误差积分、门控 ESO 补偿和力矩偏置补偿，
随后依次执行软限幅、硬限幅、可选力矩斜率限幅。
ESO 使用上一拍力矩命令更新观测状态；反馈力矩用于滤波及偏置补偿。

代码保留原工程的控制逻辑，本次整理未修改控制律。
