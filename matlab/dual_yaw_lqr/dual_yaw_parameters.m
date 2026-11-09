function p = dual_yaw_parameters(model_mode, controller_mode)
%DUAL_YAW_PARAMETERS Configuration for the article's two-stage yaw LQR model.
%   P = DUAL_YAW_PARAMETERS() uses the article's relative-coordinate plant.
%   P = DUAL_YAW_PARAMETERS('screenshot') reproduces the diagram literally.
%   P = DUAL_YAW_PARAMETERS('physical') uses relative bearing friction.
%   P = DUAL_YAW_PARAMETERS(MODE, 'discrete') designs a sampled LQR.
%
%   J1/J2 below are the article's CAD inertias, NOT identified robot values.
%   The torque limits and 1-ms step are reported in the article. Damping is
%   unknown and set to zero. Q/R and waveform settings are DEMO values;
%   neither the original Q/R nor the waveform implementation was published.
%   After editing fields of P, call P = DUAL_YAW_DESIGN(P) to update A/B/K.

if nargin < 1 || isempty(model_mode)
    model_mode = 'article';
end
if nargin < 2 || isempty(controller_mode)
    controller_mode = 'continuous';
end

p.model_mode = model_mode;
p.controller_mode = controller_mode;
p.parameter_note = ['J1/J2: article CAD estimates, not identified values. ', ...
    'Limits and 1-ms step: article settings. B1/B2: unknown, zero assumption. ', ...
    'Q/R and waveform parameters: demonstration values, not original tuning.'];
p.source_url = 'https://bbs.robomaster.com/wiki/36827278/1647282?source=7#head8';

% Inertias [kg*m^2]. J1 excludes the upper/small-yaw inertia J2.
p.J1 = 0.178592;
p.J2 = 0.044374;
% Viscous damping [N*m*s/rad]; its interpretation depends on model_mode.
p.B1 = 0;
p.B2 = 0;

p.Ts = 0.001;
p.solver_step = 0.001;
p.stop_time = 10;
% Article simulation torque limits [N*m], not a verification of motor ratings.
p.torque_limit = [7; 1.2];
% Dead-zone width is not specified in the source; leave it disabled.
p.dead_zone = [0; 0];
p.Q = diag([120, 1.5, 300, 2]);
p.R = diag([1, 0.6]);
% State order is interleaved [q1; q1_dot; q2; q2_dot] for EVERY mode.
% In 'article', q2 is RELATIVE angle; in other modes it is ABSOLUTE angle.
p.x0 = zeros(4, 1);
p.enable_feedforward = false;

% DEMO waveform settings: original amplitudes/frequencies were not disclosed.
% Article style: big sine with zero speed target, small smooth-reset saw.
% Optional 'chirp': consistent angle/velocity reference for both coordinates,
% useful for checking inverse-dynamics feedforward. For chirp only, the big
% coordinate equals ref_big_ratio times the small coordinate.
p.ref_kind = 'article';
p.ref_amplitude = deg2rad(15);
p.ref_f1 = 0.5;
p.ref_f2 = 1;
p.ref_big_ratio = 1;
p.ref_start = 1;
p.ref_ramp = 0.5;
p.ref_reset_ratio = 0.2;

p = dual_yaw_design(p);
end
