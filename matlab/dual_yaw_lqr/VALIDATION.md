# Validation

Validated with MATLAB R2024a, Simulink and Control System Toolbox on 2026-11-09.

Run from this directory:

```matlab
validate_dual_yaw_lqr
run_dual_yaw_lqr
```

Checks passed:

- Absolute/relative coordinate transformation produces equivalent dynamics.
- Each integrator plant matches the exact state-space constant-input response:
  article model maximum error 8.33e-16, screenshot model 4.41e-15, physical model 1.25e-15.
- Continuous and discrete LQR designs have stable unsaturated linear poles.
- Both controllers recover from a nonzero initial state while respecting torque limits;
  these cases also exercise saturation.
- Analytic chirp velocities match numerical differentiation, and feedforward is finite.
- The saved SLX runs after resetting the MATLAB search path, without project scripts
  or base-workspace initialization.

The default article-style demonstration has large-yaw phase lag because its speed
reference is deliberately zero. Rapid small-yaw resets can saturate the 1.2 N*m limit.
The figures show these effects. These are simulation checks using illustrative
controller weights and article CAD inertias, rather than hardware validation.
