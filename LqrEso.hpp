#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 云台 Yaw 轴 LQR + 三阶 ESO 控制器（南理工 yaw_auto_lqr_eso_controller 的单头文件 C++ 实现）
constructor_args:
    param:
      J: 0.0                  # 转动惯量 kg·m²
      B: 0.0                  # 粘滞阻尼 N·m·s/rad
      k_theta: 0.0            # 角度反馈增益（=LQR 的 Kp）
      k_omega: 0.0            # 角速度反馈增益（=LQR 的 Kd）
      k_i: 0.0                # 角度误差积分增益
      theta_integral_limit: 0.0   # 积分限幅（抗饱和）
      tau_coulomb: 0.0        # 库仑摩擦力矩幅值 N·m
      coulomb_smooth: 0.0     # 库仑摩擦 tanh 平滑速度 rad/s
      eso_bandwidth: 0.0      # ESO 观测器带宽 w0（rad/s）
      eso_comp_gain: 0.0      # ESO 扰动补偿接入增益 g（0~1）
      eso_comp_limit: 0.0     # ESO 补偿限幅 N·m
      eso_omega_gate: 0.0     # ESO 补偿转速门控（|ω|≤此值才开补偿）
      eso_alpha_gate: 0.0     # ESO 补偿角加速度门控（|α|≤此值才开补偿）
      tau_bias_ki: 0.0        # 偏置补偿积分增益
      tau_bias_limit: 0.0     # 偏置补偿限幅 N·m
      tau_meas_lpf_alpha: 0.0 # 实测力矩低通系数（0~1）
      theta_deadband: 0.0     # 角度误差死区 rad
      torque_soft_limit: 0.0  # 软限幅 N·m
      torque_min: 0.0         # 硬限幅下限 N·m
      torque_max: 0.0         # 硬限幅上限 N·m
      torque_slew_rate: 0.0   # 力矩斜率限幅速率 N·m/s
      eso_enable: true        # ESO 观测使能
      eso_comp_enable: true   # ESO 补偿接入使能（false=只观测不补偿）
      torque_slew_enable: true # 斜率限幅使能
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdint>

#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"

/**
 * @brief Yaw 轴 LQR + 三阶 ESO 控制器（单头文件 C++ 类实现）
 *
 * 由南理工 yaw_auto_lqr_eso_controller.c 折叠而来：原 C 的配置结构体、运行时
 * 状态、20+ 输出中间量全部收进本类，作为 private 成员与方法，不再有独立 .c/.h。
 * 控制律与 C 版逐行一致（见 Calc()）。
 *
 *   tau = 前馈(J·α_ref + B·ω_ref + 库仑)
 *         − k_theta·e_theta − k_omega·e_omega − k_i·∫e_theta   (LQR+积分)
 *         + ESO补偿(−g·z3/b0, 门控) + 偏置补偿
 *       → 软限幅 → 硬限幅 → 斜率限幅
 *
 * 两个关键适配点（相比原 C 代码的直搬）：
 *   1. 主接口只返回一个力矩值 tau_cmd；20+ 中间量留在 last_output() 供 Ozone
 * 读。
 *   2. 角度是 CycleValue 环绕（存储 [0,2π)），C 算法的 e_theta 与 ESO
 * 都假设绝对连续 角度，所以本类内部先 unwrap 成连续角再喂进去，避免跨 ±π
 * 时误差跳 2π。
 */
class LqrEso : public LibXR::Application {
 public:
  /* 参数结构体：由 manifest/yaml 绑定，genmain 生成构造实参（字段名不变） */
  typedef struct {
    float J;                    /* 转动惯量 kg·m² */
    float B;                    /* 粘滞阻尼 N·m·s/rad */
    float k_theta;              /* 角度反馈增益（LQR Kp） */
    float k_omega;              /* 角速度反馈增益（LQR Kd） */
    float k_i;                  /* 角度误差积分增益 */
    float theta_integral_limit; /* 积分限幅，抗饱和 */
    float tau_coulomb;          /* 库仑摩擦力矩幅值 */
    float coulomb_smooth;       /* 库仑摩擦 tanh 平滑速度 */
    float eso_bandwidth;        /* ESO 观测器带宽 w0 */
    float eso_comp_gain;        /* ESO 扰动补偿接入增益 g（0~1） */
    float eso_comp_limit;       /* ESO 补偿限幅 */
    float eso_omega_gate;       /* 转速门控：|ω|≤此值才开补偿 */
    float eso_alpha_gate;       /* 加速度门控：|α|≤此值才开补偿 */
    float tau_bias_ki;          /* 偏置补偿积分增益 */
    float tau_bias_limit;       /* 偏置补偿限幅 */
    float tau_meas_lpf_alpha;   /* 实测力矩低通系数 */
    float theta_deadband;       /* 角度误差死区 */
    float torque_soft_limit;    /* 软限幅 */
    float torque_min;           /* 硬限幅下限 */
    float torque_max;           /* 硬限幅上限 */
    float torque_slew_rate;     /* 力矩斜率限幅速率 */
    bool eso_enable;            /* ESO 观测使能 */
    bool eso_comp_enable;       /* ESO 补偿接入使能 */
    bool torque_slew_enable;    /* 斜率限幅使能 */
  } Param;

  /* 上一拍输出中间量（Ozone 调试用），字段与旧 LqrEsoOutput_t
   * 对应（去单位后缀） */
  struct Output {
    float theta_ref;
    float omega_ref;
    float alpha_ref;
    float e_theta;
    float e_omega;
    float tau_ff_alpha;
    float tau_ff_viscous;
    float tau_ff_coulomb;
    float tau_ff;
    float tau_i;
    float tau_lqr;
    float tau_eso_raw;
    float tau_eso_active;
    float tau_bias;
    float tau_pre_limit;
    float tau_cmd_before_slew;
    float tau_cmd;
    uint8_t eso_comp_active;
    uint8_t soft_limit_active;
    uint8_t hard_limit_active;
    uint8_t slew_limit_active;
  };

  LqrEso(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
         const Param& param)
      : param_(param) {
    UNUSED(hw);
    UNUSED(app);
    last_ = Output{};  // last_output() 首次读前全 0（对应旧 memset）
  }

  /**
   * @brief 一步 LQR-ESO 计算
   * @param target     目标角度（CycleValue，±π 环绕）
   * @param pos        当前角度（AHRS 欧拉角 yaw）
   * @param vol        当前角速度（AHRS 陀螺）
   * @param target_vol 目标角速度
   * @param target_ddot 目标角加速度前馈
   * @param tau_meas   实测力矩（DMMotor 反馈，N·m）
   * @param dt_s       控制周期（实测）
   * @return 输出力矩指令 tau_cmd（N·m，已过软/硬/斜率限幅）
   */
  float Calc(const LibXR::CycleValue<float>& target, float pos, float vol,
             float target_vol, float target_ddot, float tau_meas, float dt_s);

  /** @brief 重置内部状态（模式切换 RELAX→COMMON
   * 时调用），清积分/偏置/滤波并触发热启动 */
  void Reset(float theta, float omega) {
    eso_z1_ = theta;
    eso_z2_ = omega;
    eso_z3_ = 0.0f;
    theta_integral_ = 0.0f;
    tau_bias_ = 0.0f;
    tau_meas_filt_ = 0.0f;
    tau_cmd_last_ = 0.0f;
    feedback_ready_ = false;
    unwrap_init_ = false;  // 下次 Calc 重新建立连续角基准
  }

  /** @brief 上一拍完整输出（20+ 中间量，Ozone 调试用，不进主接口） */
  const Output& last_output() const { return last_; }

  void OnMonitor() override {}

 private:
  /* ---- 常量（与 C 版宏一致）---- */
  static constexpr float kMinJ = 1.0e-6f;
  static constexpr float kMinDt = 1.0e-5f;
  static constexpr float kMaxDt = 0.05f;
  static constexpr float kDefaultDt = 0.001f;

  /* ---- 静态工具（与 C 版逐行一致）---- */
  static float Abs(float v) { return v >= 0.0f ? v : -v; }
  static float Clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }
  static float ClampAbs(float v, float lim) {
    if (lim <= 0.0f) return v;
    return Clamp(v, -lim, lim);
  }
  static float Deadband(float v, float db) {
    const float a = Abs(v);
    if (db <= 0.0f) return v;
    if (a <= db) return 0.0f;
    return v > 0.0f ? v - db : v + db;
  }
  static float SanitizeDt(float dt) {
    if (dt < kMinDt || dt > kMaxDt) return kDefaultDt;
    return dt;
  }
  static bool GatePass(float v, float lim) {
    if (lim <= 0.0f) return true;
    return Abs(v) <= lim;
  }

  /* ---- unwrap（环绕角 → 连续角）---- */
  void Unwrap(const LibXR::CycleValue<float>& target, float pos) {
    if (!unwrap_init_) {
      theta_ref_cont_ = static_cast<float>(target);
      theta_meas_cont_ = static_cast<float>(LibXR::CycleValue<float>(pos));
      unwrap_init_ = true;
      return;
    }
    theta_ref_cont_ += WrapDelta(static_cast<float>(target), theta_ref_cont_);
    theta_meas_cont_ += WrapDelta(pos, theta_meas_cont_);
  }

  /* raw 相对 last 的最小环绕增量，落在 [-π, π) */
  static float WrapDelta(float raw, float last) {
    return LibXR::CycleValue<float>(raw) - LibXR::CycleValue<float>(last);
  }

  /* ---- ESO / 补偿 ---- */
  void UpdateEsoGains();
  void UpdateEso(float theta_meas, float u, float dt);
  float CoulombComp(float omega_ref);
  float CalcEsoComp();

  /* ---- 状态成员（跨拍保留）---- */
  Param param_;                  /* 配置（构造后只读，算法直接读它） */
  Output last_;                  /* 调试观测窗口 */
  float eso_z1_ = 0.0f;          /* ESO 状态：θ 估计 */
  float eso_z2_ = 0.0f;          /* ESO 状态：ω 估计 */
  float eso_z3_ = 0.0f;          /* ESO 状态：总扰动估计 */
  float eso_b0_ = 0.0f;          /* b0 = 1/J */
  float eso_w0_ = 0.0f;          /* 观测带宽 */
  float eso_beta1_ = 0.0f;       /* 3·w0 */
  float eso_beta2_ = 0.0f;       /* 3·w0² */
  float eso_beta3_ = 0.0f;       /* w0³ */
  float theta_integral_ = 0.0f;  /* 角度误差积分 */
  float tau_bias_ = 0.0f;        /* 偏置补偿 */
  float tau_meas_filt_ = 0.0f;   /* 实测力矩低通 */
  float tau_cmd_last_ = 0.0f;    /* 上一拍下发力矩（ESO 的 u） */
  bool feedback_ready_ = false;  /* 首拍热启动标志 */
  float theta_meas_cont_ = 0.0f; /* unwrap 后的连续测量角 */
  float theta_ref_cont_ = 0.0f;  /* unwrap 后的连续目标角 */
  bool unwrap_init_ = false;     /* unwrap 是否已建立基准 */
};

/* ================= 方法体（原 yaw_auto_lqr_eso_controller.c
 * 逐行转录）================= */

inline void LqrEso::UpdateEsoGains() {
  if (param_.J <= kMinJ || !param_.eso_enable) {
    eso_b0_ = 0.0f;
    eso_w0_ = 0.0f;
    eso_beta1_ = 0.0f;
    eso_beta2_ = 0.0f;
    eso_beta3_ = 0.0f;
    return;
  }
  const float w0 = param_.eso_bandwidth > 0.0f ? param_.eso_bandwidth : 0.0f;
  eso_b0_ = 1.0f / param_.J;
  eso_w0_ = w0;
  eso_beta1_ = 3.0f * w0;
  eso_beta2_ = 3.0f * w0 * w0;
  eso_beta3_ = w0 * w0 * w0;
}

inline void LqrEso::UpdateEso(float theta_meas, float u, float dt) {
  if (!param_.eso_enable || param_.J <= kMinJ || eso_w0_ <= 0.0f) {
    return;
  }
  const float damping = param_.B / param_.J;
  const float e = theta_meas - eso_z1_;  // 观测误差：测量角 − 估计角
  const float z1_dot = eso_z2_ + eso_beta1_ * e;
  const float z2_dot =
      -damping * eso_z2_ + eso_b0_ * u + eso_z3_ + eso_beta2_ * e;
  const float z3_dot = eso_beta3_ * e;
  eso_z1_ += dt * z1_dot;
  eso_z2_ += dt * z2_dot;
  eso_z3_ += dt * z3_dot;
}

inline float LqrEso::CoulombComp(float omega_ref) {
  if (param_.coulomb_smooth <= 1.0e-6f || param_.tau_coulomb == 0.0f) {
    return 0.0f;
  }
  return param_.tau_coulomb * tanhf(omega_ref / param_.coulomb_smooth);
}

inline float LqrEso::CalcEsoComp() {
  if (!param_.eso_enable || eso_b0_ <= 1.0e-6f ||
      param_.eso_comp_gain == 0.0f) {
    return 0.0f;
  }
  const float tau_eso = -param_.eso_comp_gain * eso_z3_ / eso_b0_;
  return ClampAbs(tau_eso, param_.eso_comp_limit);
}

inline float LqrEso::Calc(const LibXR::CycleValue<float>& target, float pos,
                          float vol, float target_vol, float target_ddot,
                          float tau_meas, float dt) {
  Unwrap(target, pos);  // 环绕角 → 连续角

  Output out{};  // 零初始化，对应 C 版 memset(&out, 0, sizeof(out))
  dt = SanitizeDt(dt);
  UpdateEsoGains();

  // 首拍热启动：初始化滤波/ESO 状态，本拍只回传参考、不出力矩
  if (!feedback_ready_) {
    feedback_ready_ = true;
    tau_meas_filt_ = tau_meas;
    eso_z1_ = theta_meas_cont_;
    eso_z2_ = vol;
    eso_z3_ = 0.0f;
    out.theta_ref = theta_ref_cont_;
    out.omega_ref = target_vol;
    out.alpha_ref = target_ddot;
    last_ = out;
    return last_.tau_cmd;  // = 0
  }

  const float tau_meas_alpha = Clamp(param_.tau_meas_lpf_alpha, 0.0f, 1.0f);
  tau_meas_filt_ += tau_meas_alpha * (tau_meas - tau_meas_filt_);

  UpdateEso(theta_meas_cont_, tau_cmd_last_, dt);

  const float e_theta =
      Deadband(theta_meas_cont_ - theta_ref_cont_, param_.theta_deadband);
  const float e_omega = vol - target_vol;

  theta_integral_ += e_theta * dt;
  theta_integral_ = ClampAbs(theta_integral_, param_.theta_integral_limit);

  out.theta_ref = theta_ref_cont_;
  out.omega_ref = target_vol;
  out.alpha_ref = target_ddot;
  out.e_theta = e_theta;
  out.e_omega = e_omega;
  out.tau_ff_alpha = param_.J * target_ddot;
  out.tau_ff_viscous = param_.B * target_vol;
  out.tau_ff_coulomb = CoulombComp(target_vol);
  out.tau_ff = out.tau_ff_alpha + out.tau_ff_viscous + out.tau_ff_coulomb;
  out.tau_i = -param_.k_i * theta_integral_;
  out.tau_lqr = out.tau_ff - param_.k_theta * e_theta -
                param_.k_omega * e_omega + out.tau_i;
  out.tau_eso_raw = CalcEsoComp();
  if (param_.eso_comp_enable && GatePass(vol, param_.eso_omega_gate) &&
      GatePass(target_ddot, param_.eso_alpha_gate)) {
    out.eso_comp_active = 1U;
    out.tau_eso_active = out.tau_eso_raw;
  }

  const float tau_without_bias = out.tau_lqr + out.tau_eso_active;
  const float tau_track_err = tau_without_bias - tau_meas_filt_;
  tau_bias_ += param_.tau_bias_ki * tau_track_err * dt;
  tau_bias_ = ClampAbs(tau_bias_, param_.tau_bias_limit);

  out.tau_bias = tau_bias_;
  out.tau_pre_limit = tau_without_bias + tau_bias_;
  float tau_cmd = out.tau_pre_limit;

  // 软限幅
  if (param_.torque_soft_limit > 0.0f) {
    const float limited = ClampAbs(tau_cmd, param_.torque_soft_limit);
    if (limited != tau_cmd) {
      out.soft_limit_active = 1U;
    }
    tau_cmd = limited;
  }

  // 硬限幅
  if (param_.torque_min < param_.torque_max) {
    const float limited = Clamp(tau_cmd, param_.torque_min, param_.torque_max);
    if (limited != tau_cmd) {
      out.hard_limit_active = 1U;
    }
    tau_cmd = limited;
  }

  out.tau_cmd_before_slew = tau_cmd;
  // 斜率限幅
  if (param_.torque_slew_enable && param_.torque_slew_rate > 0.0f) {
    const float max_delta = param_.torque_slew_rate * dt;
    const float limited =
        Clamp(tau_cmd, tau_cmd_last_ - max_delta, tau_cmd_last_ + max_delta);
    if (limited != tau_cmd) {
      out.slew_limit_active = 1U;
    }
    tau_cmd = limited;
  }

  out.tau_cmd = tau_cmd;
  tau_cmd_last_ = tau_cmd;
  last_ = out;
  return tau_cmd;
}
