function [x_ref, u_ff] = dual_yaw_reference(p, t)
%DUAL_YAW_REFERENCE Analytic trajectories and optional model feedforward.
%   Outputs are N-by-4/N-by-2, suitable for timeseries(X_REF,T(:)) and
%   timeseries(U_FF,T(:)). State order is always [q1,q1_dot,q2,q2_dot].
%   In 'article' mode q2 is RELATIVE; in other plant modes it is ABSOLUTE.
%
%   ref_kind='article': q1 is sinusoidal, with its controller SPEED TARGET
%   forced to ZERO as the article describes. q2 is a saw with a linear scan
%   and a quintic C2 reset, with exact analytic velocity. Its amplitude marks
%   the scan endpoints; the smooth reset has slight corner overshoot because
%   its endpoint velocities match the nonzero linear scan velocity.
%   ref_kind='chirp': frequency sweeps from ref_f1 to ref_f2, and all returned
%   speeds are exact position derivatives. q1 = ref_big_ratio*q2.
%
%   Both modes start through a quintic C2 envelope. If enabled, feedforward
%   ALWAYS uses true trajectory derivatives, including q1's true velocity;
%   it never substitutes the deliberately zeroed article speed target.

validateattributes(t, {'numeric'}, {'real', 'finite', 'vector'}, ...
    mfilename, 't');
validateattributes(p.ref_amplitude, {'numeric'}, ...
    {'real', 'finite', 'scalar'}, mfilename, 'p.ref_amplitude');
validateattributes(p.ref_big_ratio, {'numeric'}, ...
    {'real', 'finite', 'scalar'}, mfilename, 'p.ref_big_ratio');
validateattributes(p.ref_f1, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'}, mfilename, 'p.ref_f1');
validateattributes(p.ref_f2, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'}, mfilename, 'p.ref_f2');
validateattributes(p.ref_start, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'}, mfilename, 'p.ref_start');
validateattributes(p.stop_time, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'}, mfilename, 'p.stop_time');
validateattributes(p.ref_ramp, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'}, mfilename, 'p.ref_ramp');
ref_kind = validatestring(p.ref_kind, {'article', 'chirp'}, ...
    mfilename, 'p.ref_kind');
t = double(t(:));
% A short plant/recovery test may finish before reference activation.
if all(t <= p.ref_start)
    x_ref = zeros(numel(t), 4);
    u_ff = zeros(numel(t), 2);
    return;
end
if strcmp(ref_kind, 'chirp') && p.stop_time <= p.ref_start
    error('dual_yaw:InvalidReferenceTiming', ...
        'An active chirp requires stop_time > ref_start.');
end
tau = max(t-p.ref_start, 0);
s = min(tau/p.ref_ramp, 1);
[envelope, envelope_dot, envelope_ddot] = quintic_step(s);
envelope_dot = envelope_dot/p.ref_ramp;
envelope_ddot = envelope_ddot/p.ref_ramp^2;
ramping = t > p.ref_start & tau < p.ref_ramp;
envelope_dot(~ramping) = 0;
envelope_ddot(~ramping) = 0;

switch ref_kind
    case 'article'
        phase_big = 2*pi*p.ref_f1*tau;
        rate_big = 2*pi*p.ref_f1;
        raw_q1 = p.ref_big_ratio*p.ref_amplitude*sin(phase_big);
        raw_v1 = p.ref_big_ratio*p.ref_amplitude*rate_big*cos(phase_big);
        raw_a1 = -p.ref_big_ratio*p.ref_amplitude*rate_big^2*sin(phase_big);
        [raw_q2, raw_v2, raw_a2] = smooth_reset_saw(p, tau);
    case 'chirp'
        sweep_rate = (p.ref_f2-p.ref_f1)/(p.stop_time-p.ref_start);
        phase = 2*pi*(p.ref_f1*tau+0.5*sweep_rate*tau.^2);
        phase_dot = 2*pi*(p.ref_f1+sweep_rate*tau);
        phase_ddot = 2*pi*sweep_rate;
        raw_q2 = p.ref_amplitude*sin(phase);
        raw_v2 = p.ref_amplitude*cos(phase).*phase_dot;
        raw_a2 = p.ref_amplitude*(-sin(phase).*phase_dot.^2 + ...
            cos(phase)*phase_ddot);
        raw_q1 = p.ref_big_ratio*raw_q2;
        raw_v1 = p.ref_big_ratio*raw_v2;
        raw_a1 = p.ref_big_ratio*raw_a2;
end

q1 = envelope.*raw_q1;
v1 = envelope_dot.*raw_q1+envelope.*raw_v1;
a1 = envelope_ddot.*raw_q1+2*envelope_dot.*raw_v1+envelope.*raw_a1;
q2 = envelope.*raw_q2;
v2 = envelope_dot.*raw_q2+envelope.*raw_v2;
a2 = envelope_ddot.*raw_q2+2*envelope_dot.*raw_v2+envelope.*raw_a2;
speed_target_big = v1;
if strcmp(ref_kind, 'article')
    speed_target_big = zeros(size(v1));
end
x_ref = [q1, speed_target_big, q2, v2];

if ~p.enable_feedforward
    u_ff = zeros(numel(t), 2);
    return;
end
switch p.model_mode
    case 'article'
        % Generalized relative coordinates: M*q_ddot + D*q_dot = tau.
        tau_big = (p.J1+p.J2)*a1+p.J2*a2+p.B1*v1;
        tau_small = p.J2*(a1+a2)+p.B2*v2;
    case 'screenshot'
        tau_small = p.J2*a2+p.B2*v2;
        tau_big = p.J1*a1+p.B1*v1+tau_small;
    case 'physical'
        tau_small = p.J2*a2+p.B2*(v2-v1);
        tau_big = p.J1*a1+p.J2*a2+p.B1*v1;
    otherwise
        error('dual_yaw:UnknownPlantMode', ...
            'model_mode must be article, screenshot or physical.');
end
u_ff = [tau_big, tau_small];
end

function [value, derivative, second_derivative] = quintic_step(s)
value = 10*s.^3-15*s.^4+6*s.^5;
derivative = 30*s.^2-60*s.^3+30*s.^4;
second_derivative = 60*s-180*s.^2+120*s.^3;
end

function [position, velocity, acceleration] = smooth_reset_saw(p, tau)
% Exact linear scan followed by a quintic Hermite reset. At each join both
% velocity and acceleration match, including at the periodic cycle boundary.
validateattributes(p.ref_reset_ratio, {'numeric'}, ...
    {'real', 'finite', 'scalar', '>', 0, '<', 1}, ...
    mfilename, 'p.ref_reset_ratio');
period = 1/p.ref_f2;
scan_time = (1-p.ref_reset_ratio)*period;
reset_time = p.ref_reset_ratio*period;
cycle_time = mod(tau, period);
scan_velocity = 2*p.ref_amplitude/scan_time;
position = -p.ref_amplitude+scan_velocity*cycle_time;
velocity = scan_velocity*ones(size(tau));
acceleration = zeros(size(tau));
resetting = cycle_time >= scan_time;
s = (cycle_time(resetting)-scan_time)/reset_time;
[smooth, smooth_dot, smooth_ddot] = quintic_step(s);
linear_increment = scan_velocity*reset_time;
correction = -2*p.ref_amplitude-linear_increment;
position(resetting) = p.ref_amplitude+linear_increment*s+correction*smooth;
velocity(resetting) = scan_velocity+correction*smooth_dot/reset_time;
acceleration(resetting) = correction*smooth_ddot/reset_time^2;
end
